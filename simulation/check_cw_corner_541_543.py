"""Archived corner projections through production acquisition; no robot motion.

Two fresh-frame simulations at each sampled geometry exercise voting. They
are not two complete archived images: original frames were not saved.
"""
import math
import re
import shutil
import subprocess

from check_cw_start_planner import ROOT
from check_far_obstacle_acquisition import fixture_source
from check_later_laps import later_lap_fixture
from check_connector_servo_resume import block


def main():
    rows = []
    for n in range(541, 544):
        p = ROOT/f'simulation/evidence/parking_exit_diagnostics/20261027_log_{n}_cw.txt'
        for line in p.read_text().splitlines():
            if '[DISCOVERY_TRACE]' not in line or 'obs_geom=' not in line:
                continue
            fields = dict(t.split('=', 1) for t in line.split() if '=' in t)
            g = list(map(float, fields['obs_geom'].split(',')))
            if g[3] > -1000 or not 230 <= float(fields['obs_range']) <= 600:
                continue
            pose = list(map(float, fields['pose'].split(',')))
            # Reconstruct centroid from the logged bearing (the bounding-box
            # midpoint in obs_geom is not the component's weighted centroid).
            cx = round(164.4-248.9*math.tan(math.radians(g[2])))
            raw = list(map(int, fields['raw'].split(',')))
            rows.append((n, *pose, cx, int(g[1]), raw[2], raw[3]))
    assert len(rows) == 3, rows
    code = fixture_source() + r'''
int main(){int id,foot,cx,w,ht;float x,y,h;int total=0;
 while(std::cin>>id>>x>>y>>h>>cx>>foot>>w>>ht){
  for(int mirror:{0,1})for(auto color:{ColorType::GREEN,ColorType::RED}){
   for(auto &s:seats){s={};s.x=s.y=9000;}
   float sign=mirror?-1.f:1.f;
   seats[6].x=sign*-900;seats[7].x=sign*-1100;
   seats[6].y=seats[7].y=-500;
   seats[6].headingDeg=seats[7].headingDeg=mirror?90:-90;
   seats[6].pathDistanceMm=seats[7].pathDistanceMm=775;
   hostPose={sign*x,y,mirror?180-h:h};clockMs+=1000;injectionCount=0;
   Blob b{};b.found=true;b.color=color;b.centerX=mirror?int(lroundf(328.8f-cx)):cx;
   b.minX=b.centerX-w/2;b.maxX=b.minX+w-1;
   b.maxY=foot;b.minY=foot-ht+1;b.centerY=(b.minY+b.maxY)/2;b.area=w*ht/2;
   auto first=obstacle_path_observe(&b);clockMs+=76;
   auto second=obstacle_path_observe(&b);++total;
   if(first.status!=OBSTACLE_OBSERVATION_VOTE||first.seatId!=7||
      !second.confirmed||second.seatId!=7||injectionCount!=1){
    std::cout<<"FAIL "<<id<<" "<<mirror<<" status "<<first.status<<" "<<second.status
     <<" projection "<<first.sightingXmm<<" "<<first.sightingYmm<<"\n";return 1;
   }
   // The identical projection without camera geometry remains ambiguous.
   seats[7].confirmed=false;
   if(!distantSeatSideAmbiguous(first.sightingXmm,first.sightingYmm,first.rangeMm,7))return 2;
   // Moving the sighting onto the midpoint must still be rejected even with
   // the camera pose. Widely separated bearings cannot justify a central hit.
   if(!distantSeatSideAmbiguous(sign*-1000,-500,first.rangeMm,7,first.cameraXmm,first.cameraYmm))return 3;
  }
 }
 seats[10]={};seats[11]={};seats[10].x=-900;seats[11].x=-1100;
 seats[10].y=seats[11].y=500;seats[10].headingDeg=seats[11].headingDeg=90;
 if(!distantSeatSideAmbiguous(-1035,620,547,11,-1020.1118f,73.5051f))return 7;
 std::cout<<"CORNER ASSOCIATION "<<total<<" PASS: three archived geometries, mirrored, both colours; midpoint/legacy/514 guards\n";
}
'''
    dest = ROOT/'local_workspace/cw-corner-541-543'
    dest.mkdir(exist_ok=True)
    cpp, exe = dest/'observe.cpp', dest/'observe.exe'
    cpp.write_text(code)
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'simulation/host'),
                    '-I', str(ROOT/'include'), str(cpp), '-o', str(exe)], check=True)
    inp = '\n'.join(' '.join(map(str, row)) for row in rows)+'\n'
    (dest/'input.txt').write_text(inp)
    result = subprocess.run([str(exe)], input=inp, text=True, capture_output=True)
    (dest/'output.txt').write_text(result.stdout+result.stderr)
    print(result.stdout)
    if result.returncode:
        raise SystemExit(result.returncode)

    source = (ROOT/'src/obstacle_path.cpp').read_text()
    geometry = later_lap_fixture().split('int main()')[0]
    geometry = geometry.replace('struct Discovery {bool observedClear=false;};',
        'struct Discovery {bool observedClear=false;bool seatObservedClear[2]={};};')
    geometry += '\nbool earlyMiddleViewActive[OBSTACLE_SEAT_COUNT]={};\n'
    for sig in ('bool earlyMiddleViewEligible(', 'bool earlyMiddleViewPathSafe(', 'void rebuildLivePath()',
                'bool cornerViewSeatMayBeOccupied(', 'bool cornerViewSweepSafe('):
        geometry += block(source, source.index(sig))+'\n'
    old = subprocess.check_output(['git', 'show', 'cc0bc59:src/obstacle_path.cpp'], text=True)
    geometry += block(old, old.index('bool cornerViewSweepSafe(')).replace(
        'bool cornerViewSweepSafe(', 'bool frozenCornerSweepSafe(', 1)+'\n'
    geometry += r'''
int main(){int total=0;float minimumWall=1e9f,minimumPillar=1e9f;
 for(int dir:{-1,1})for(float y:{-1250.f,-1000.f,-750.f,-500.f,0.f,500.f,1000.f})
 for(float x:{-1250.f,-1000.f,-750.f,0.f,750.f,1000.f,1250.f})
 for(float h:{0.f,45.f,90.f,135.f,180.f,225.f,270.f,315.f})for(float travel:{-240.f,-110.f,0.f,15.f,220.f}){
  routeTurnSign=dir;resetState();baseline();
  for(int i=0;i<24;++i){seats[i].confirmed=i%4==0;discoveryStations[i/2].seatObservedClear[i%2]=i%3==0;}
  PositionEstimate p{x,y,h};
  if(cornerViewSweepSafe(p,travel)!=frozenCornerSweepSafe(p,travel))return 4;
  ++total;
 }
 std::cout<<"RECOVERY GEOMETRY "<<total<<" equivalent decisions\n";
 const float poses[3][3]={{-562.6f,-730.9f,173.93f},{-562.5f,-727.8f,173.88f},{-557.5f,-727.f,174.07f}};
 int routes=0;
 for(const auto &start:poses)for(int mirror:{0,1})for(float dx:{-10.f,0.f,10.f})
 for(float dy:{-10.f,0.f,10.f})for(float dh:{-1.f,0.f,1.f}){
  routeTurnSign=mirror?1:-1;resetState();baseline();
  for(auto &s:seats)s.confirmed=s.injected=false;
  seats[mirror?5:4].confirmed=seats[mirror?5:4].injected=true;seats[mirror?5:4].red=true;
  // Reflection swaps LEFT/RIGHT seat numbering, but keeps colour/passing rule.
  const int green=mirror?6:7;
  seats[green].confirmed=seats[green].injected=true;seats[green].red=false;rebuildLivePath();
  float x=(mirror?-start[0]:start[0])+dx,y=start[1]+dy;
  float h=((mirror?180-start[2]:start[2])+dh)*PI/180;
  progressIndex=nearestPathIndex(livePath,x,y,0,pathLength);
  bool passed=false;
  for(int step=0;step<900;++step){
   progressIndex=nearestPathIndex(livePath,x,y,progressIndex,OBSTACLE_PATH_PROGRESS_WINDOW);
   for(int seat:{0,green}){ObstacleClearanceSample c{};calculateClearanceAtPose(seats[seat],x,y,h*180/PI,c);
    minimumWall=fminf(minimumWall,c.wallMm);
    if(seat==green)minimumPillar=fminf(minimumPillar,c.pillarMm);
    if(c.wallMm<30||(seat==green&&c.pillarMm<30)){
     std::cout<<"ROUTE FAIL "<<mirror<<" "<<x<<" "<<y<<" "<<c.wallMm<<" "<<c.pillarMm<<"\n";return 5;
    }
   }
   if(y>-200){passed=true;break;}
   const float speed=livePath[progressIndex].speedMmS;
   float look=firstLapLookahead(speed);
   if(nearCorner(baselinePath[progressIndex].distanceMm))look*=OBSTACLE_LOOKAHEAD_CORNER_SCALE;
   PositionEstimate p{x,y,h*180/PI};auto t=findLookahead(livePath,p,look);
   float tx=t.x-x,ty=t.y-y,k=2*(-tx*sinf(h)+ty*cosf(h))/fmaxf(1,tx*tx+ty*ty);
   int servo=int(clampFloat(pursuitServoForCurvature(k),-42,42));
   k=abs(servo)<2?0:-1.f/Ackermann::getTurnRadius(servo);
   float next=h+2*k;
   if(fabsf(k)>1e-6){x+=(sinf(next)-sinf(h))/k;y+=(cosf(h)-cosf(next))/k;}
   else{x+=2*cosf(h);y+=2*sinf(h);}h=next;
  }
  if(!passed)return 6;++routes;
 }
 std::cout<<"EARLY CORNER JOINS "<<routes<<" PASS minwall/pillar "<<minimumWall<<" "<<minimumPillar<<"\n";
}
'''
    cpp, exe = dest/'geometry.cpp', dest/'geometry.exe'
    cpp.write_text(geometry)
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'simulation/host'),
                    '-I', str(ROOT/'include'), str(cpp), '-o', str(exe)], check=True)
    result = subprocess.run([str(exe)], text=True, capture_output=True)
    (dest/'geometry_output.txt').write_text(result.stdout+result.stderr)
    print(result.stdout)
    raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
