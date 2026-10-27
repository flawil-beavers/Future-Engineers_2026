"""Production acquisition at log_544's rejected final-corner RED geometries."""
import math
import shutil
import subprocess

from check_connector_servo_resume import ROOT, block
from check_far_obstacle_acquisition import fixture_source
from check_later_laps import later_lap_fixture


def main():
    rows = []
    src = ROOT/'simulation/evidence/parking_exit_diagnostics/20261027_log_544_ccw.txt'
    for line in src.read_text().splitlines():
        if '[RED SEAT]' not in line or 'decision=not_upcoming' not in line:
            continue
        f = dict(t.split('=', 1) for t in line.split() if '=' in t)
        x, y, h = map(float, f['pose'].split(','))
        bearing = float(f['bearing/range'].split('/')[0])
        cx = round(164.4-248.9*math.tan(math.radians(bearing)))
        foot = int(f['image_x/foot'].split('/')[1])
        # Production baseline at the logged progress indices 113 and 114.
        forward = 1035.398 if x < -925 else 985.398
        rows.append((x, y, h, cx, foot, forward))
    assert len(rows) == 2, rows
    fixture = fixture_source()
    main = r'''
int main(){float x,y,h,forward;int cx,foot,total=0;
 while(std::cin>>x>>y>>h>>cx>>foot>>forward){
  for(int mirror:{0,1})for(auto color:{ColorType::RED,ColorType::GREEN}){
   for(auto &s:seats){s={};s.x=s.y=9000;}
   seats[0].x=seats[1].x=mirror?500:-500;seats[0].y=-1100;seats[1].y=-900;
   seats[0].headingDeg=seats[1].headingDeg=mirror?180:0;
   seats[0].pathDistanceMm=seats[1].pathDistanceMm=forward;
   hostPose={mirror?-x:x,y,mirror?180-h:h};injectionCount=0;clockMs+=1000;
   Blob b{};b.found=true;b.color=color;b.centerX=mirror?int(lroundf(328.8f-cx)):cx;
   b.minX=b.centerX-10;b.maxX=b.centerX+10;b.minY=80;b.maxY=foot;b.area=650;
   auto a=obstacle_path_observe(&b);clockMs+=76;auto c=obstacle_path_observe(&b);++total;
   if(a.status!=OBSTACLE_OBSERVATION_VOTE||a.seatId!=1||!c.confirmed||c.seatId!=1){
    std::cout<<"FAIL visible next-corner seat status "<<a.status<<" "<<c.status<<"\n";return 1;
   }
   // A full-loop/remote station still cannot become upcoming through this exception.
   seats[1].confirmed=seats[1].injected=false;seats[1].redVotes=seats[1].greenVotes=0;
   seats[0].pathDistanceMm=seats[1].pathDistanceMm=3000;clockMs+=1000;
   if(obstacle_path_observe(&b).seatId!=-1)return 2;
  }
 }
 std::cout<<"CORNER RED 544 "<<total<<" PASS: two archived poses, both colours/mirrors; remote station rejected\n";
}
'''
    dest = ROOT/'local_workspace/start_544_546'
    dest.mkdir(exist_ok=True)
    data = '\n'.join(' '.join(map(str, row)) for row in rows)+'\n'
    def run(name, code):
        cpp, exe = dest/(name+'.cpp'), dest/(name+'.exe')
        cpp.write_text(code+main)
        subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'simulation/host'),
                        '-I', str(ROOT/'include'), str(cpp), '-o', str(exe)], check=True)
        r = subprocess.run([str(exe)], input=data, text=True, capture_output=True)
        (dest/(name+'.txt')).write_text(r.stdout+r.stderr)
        return r
    current = run('red_corner', fixture)
    print(current.stdout)
    if current.returncode:
        raise SystemExit(current.returncode)
    old = subprocess.check_output(['git', 'show', 'cc0bc59:src/obstacle_path.cpp'], text=True)
    new = (ROOT/'src/obstacle_path.cpp').read_text()
    negative = fixture.replace(block(new, new.index('ObstacleObservationResult obstacle_path_observe(')),
                               block(old, old.index('ObstacleObservationResult obstacle_path_observe(')))
    previous = run('red_corner_old', negative)
    assert previous.returncode == 1, previous.stdout
    print('PASS negative control: old code rejects the visible RED as not upcoming')
    geometry = later_lap_fixture().split('int main()')[0]
    geometry += '\nbool earlyMiddleViewActive[OBSTACLE_SEAT_COUNT]={};\n'
    for sig in ('bool earlyMiddleViewEligible(', 'bool earlyMiddleViewPathSafe(', 'void rebuildLivePath()'):
        geometry += block(new, new.index(sig))+'\n'
    geometry += r'''
int main(){int total=0;float minWall=1e9f,minPillar=1e9f;
 const float starts[4][3]={{-929,-303,286.6f},{-919,-353,286.6f},
                         {-981,-91,275.8f},{-974,-151,279.3f}};
 for(const auto &p:starts)for(float dx:{-10.f,0.f,10.f})for(float dy:{-10.f,0.f,10.f})for(float dh:{-1.f,0.f,1.f}){
  routeTurnSign=1;resetState();baseline();
  for(auto &s:seats)s.confirmed=s.injected=false;
  for(int i:{5,7,10,14,19,22}){seats[i].confirmed=seats[i].injected=true;seats[i].red=i==10||i==19;}
  seats[1].confirmed=true;seats[1].red=true;seats[1].injected=false;
  rebuildLivePath();float x=p[0]+dx,y=p[1]+dy,h=(p[2]+dh)*PI/180;
  progressIndex=nearestPathIndex(baselinePath,x,y,0,pathLength);bool passed=false;
  for(int step=0;step<1300;++step){
   progressIndex=nearestPathIndex(livePath,x,y,progressIndex,OBSTACLE_PATH_PROGRESS_WINDOW);
   if(!seats[1].injected&&cyclicDistanceForward(baselinePath[progressIndex].distanceMm,seats[1].pathDistanceMm)<=OBSTACLE_PARKING_CCW_STORED_SEAT_APPROACH_MM){
    seats[1].injected=true;rebuildLivePath();
   }
   for(int seat:{1,22}){
   ObstacleClearanceSample c{};calculateClearanceAtPose(seats[seat],x,y,h*180/PI,c);
   minWall=fminf(minWall,c.wallMm);minPillar=fminf(minPillar,c.pillarMm);
   if(c.wallMm<30||c.pillarMm<30){std::cout<<"FAIL late RED bypass "<<x<<" "<<y<<" clearance "<<c.wallMm<<" "<<c.pillarMm<<"\n";return 8;}}
   if(x>-200&&y<-700){passed=true;break;}
   float look=firstLapLookahead(livePath[progressIndex].speedMmS);
   if(nearCorner(baselinePath[progressIndex].distanceMm))look*=OBSTACLE_LOOKAHEAD_CORNER_SCALE;
   PositionEstimate pose{x,y,h*180/PI};auto t=findLookahead(livePath,pose,look);
   float tx=t.x-x,ty=t.y-y,k=2*(-tx*sinf(h)+ty*cosf(h))/fmaxf(1,tx*tx+ty*ty);
   int servo=int(clampFloat(pursuitServoForCurvature(k),-42,42));k=abs(servo)<2?0:-1.f/Ackermann::getTurnRadius(servo);
   float next=h+2*k;
   if(fabsf(k)>1e-6){x+=(sinf(next)-sinf(h))/k;y+=(cosf(h)-cosf(next))/k;}
   else{x+=2*cosf(h);y+=2*sinf(h);}h=next;
  }
  if(!passed)return 9;++total;
 }
 std::cout<<"LATE RED BYPASS "<<total<<" PASS minwall/pillar "<<minWall<<" "<<minPillar<<"\n";
}
'''
    cpp, exe = dest/'red_bypass.cpp', dest/'red_bypass.exe'
    cpp.write_text(geometry)
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'simulation/host'),
                    '-I', str(ROOT/'include'), str(cpp), '-o', str(exe)], check=True)
    r = subprocess.run([str(exe)], text=True, capture_output=True)
    (dest/'red_bypass.txt').write_text(r.stdout+r.stderr)
    print(r.stdout)
    raise SystemExit(r.returncode)


if __name__ == '__main__':
    main()
