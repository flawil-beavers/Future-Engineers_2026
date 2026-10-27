"""Archived GREEN projections through production association and route geometry.

No original images are available: geometry duplicates represent two hypothetical
fresh accepted components, not a replay of the original video.
"""
import math
import shutil
import subprocess
from check_connector_servo_resume import ROOT, block
from check_far_obstacle_acquisition import fixture_source
from check_later_laps import later_lap_fixture


def run(name, code, data='', expected=0):
    dest=ROOT/'local_workspace/cw-556-558'
    dest.mkdir(exist_ok=True)
    cpp,exe=dest/(name+'.cpp'),dest/(name+'.exe')
    cpp.write_text(code)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2',
                    '-I',str(ROOT/'simulation/host'),'-I',str(ROOT/'include'),
                    str(cpp),'-o',str(exe)],check=True)
    r=subprocess.run([str(exe)],input=data,text=True,capture_output=True)
    (dest/(name+'.txt')).write_text(r.stdout+r.stderr)
    if r.returncode!=expected:
        print(r.stdout+r.stderr)
        raise SystemExit(r.returncode or 99)
    if expected==0:
        print(r.stdout)
    else:
        print('PASS negative control:',name,'reproduces failure')


def main():
    data=[]
    for n,t in ((556,63055),(558,32660)):
        s=(ROOT/f'simulation/evidence/parking_exit_diagnostics/20261027_log_{n}_cw.txt').read_text()
        line=next(l for l in s.splitlines() if f'[GREEN SEAT PRIMARY] t={t} ' in l)
        f=dict(p.split('=',1) for p in line.split() if '=' in p)
        x,y,h=map(float,f['pose'].split(','))
        bearing=float(f['bearing/range'].split('/')[0])
        cx=round(164.4-248.9*math.tan(math.radians(bearing)))
        foot=int(f['image_x/foot'].split('/')[1])
        data.append((x,y,h,cx,foot))
    source=fixture_source()
    main=r'''
int main(){float x,y,h;int cx,foot,total=0;
 while(std::cin>>x>>y>>h>>cx>>foot){
  for(int mirror:{0,1})for(float dx:{-3.f,0.f,3.f})
  for(float dy:{-3.f,0.f,3.f})for(float dh:{-.5f,0.f,.5f}){
   for(auto &s:seats){s={};s.x=s.y=9000;}
   const float sign=mirror?-1.f:1.f;
   seats[6].x=sign*-900;seats[7].x=sign*-1100;
   seats[6].y=seats[7].y=-500;
   seats[6].headingDeg=seats[7].headingDeg=mirror?90:90;
   seats[6].pathDistanceMm=seats[7].pathDistanceMm=1000;
   hostPose={sign*x+dx,y+dy,(mirror?180-h:h)+dh};
   clockMs+=1000;injectionCount=0;
   Blob b{};b.found=true;b.color=ColorType::GREEN;
   b.centerX=mirror?int(lroundf(328.8f-cx)):cx;
   b.minX=b.centerX-10;b.maxX=b.centerX+10;b.minY=80;b.maxY=foot;b.area=650;
   auto a=obstacle_path_observe(&b);clockMs+=76;auto c=obstacle_path_observe(&b);
   if(a.seatId!=7||a.status!=OBSTACLE_OBSERVATION_VOTE||!c.confirmed||c.seatId!=7){
    std::cout<<"ROW FAIL "<<x<<" "<<mirror<<" "<<dx<<" "<<dy<<" "<<dh<<" status "<<a.status<<" seat "<<int(a.seatId)<<"\n";return 1;}
   ++total;
  }
 }
 std::cout<<"GREEN ROW "<<total<<" archived/mirrored perturbed cases PASS\n";
}
'''
    inp='\n'.join(' '.join(map(str,r)) for r in data)+'\n'
    run('rows',source+main,inp)
    run('rows_without_resolution',source.replace('const bool bearingResolved=bearingSeat>=0;',
        'const bool bearingResolved=false;')+main,inp,expected=1)

    code=source+r'''
int main(){
 for(auto &s:seats){s={};s.x=s.y=9000;}
 seats[18].x=900;seats[19].x=1100;seats[18].y=seats[19].y=500;
 seats[20].x=900;seats[21].x=1100;seats[20].y=seats[21].y=0;
 seats[22].x=900;seats[23].x=1100;seats[22].y=seats[23].y=-500;
 seats[19].confirmed=seats[19].injected=true;seats[19].red=true;
 parkingEntryConnectorActive=true;hostPose={873,531,267.6f};
 Blob b{};b.found=true;b.color=ColorType::GREEN;
 b.centerX=139;b.minX=129;b.maxX=149;b.minY=80;b.maxY=118;b.area=600;
 for(int frame=0;frame<4;++frame){clockMs+=76;auto r=obstacle_path_observe(&b);
  if(r.confirmed||seats[20].confirmed)return 2;}
 // Arbitrary training layouts remain explicitly available in CHECK_ALL.
 sectionLayoutMode=1;clockMs+=1000;obstacle_path_observe(&b);clockMs+=76;
 if(!obstacle_path_observe(&b).confirmed||!seats[20].confirmed)return 3;
 std::cout<<"556 MIDDLE alias rejected at512mm; CHECK_ALL retains arbitrary layout\n";
}
'''
    run('middle_alias',code)

    code=source+r'''
Blob component(float bearing,int foot){
 Blob b{};b.found=true;b.color=ColorType::RED;
 b.centerX=int(lroundf(164.4f-248.9f*tanf(bearing*PI/180)));
 b.minX=b.centerX-10;b.maxX=b.centerX+10;b.minY=80;b.maxY=foot;b.area=600;
 return b;
}
void clear(){for(auto &s:seats){s={};s.x=s.y=9000;}clockMs+=1000;}
bool confirm(Blob b){obstacle_path_observe(&b);clockMs+=76;
 return obstacle_path_observe(&b).confirmed;}
int main(){
 // CCW560--562: distant RED9 arrives before GREEN7 during the connector.
 clear();routeTurnSign=1;parkingEntryConnectorActive=true;
 seats[8].x=seats[9].x=800;seats[8].y=0;seats[9].y=200;
 seats[8].pathDistanceMm=seats[9].pathDistanceMm=775;
 hostPose={800-646.1538f-OBSTACLE_CAMERA_LOCAL_X_MM,200,0};
 Blob b=component(0,104);
 if(confirm(b)||seats[9].confirmed)return 8;
 // Compatible far middle acquisition on the normal route is retained.
 parkingEntryConnectorActive=false;
 if(!confirm(b)||!seats[9].confirmed)return 9;
 clear();parkingEntryConnectorActive=true;sectionLayoutMode=1;
 seats[9].x=800;seats[9].y=200;seats[9].pathDistanceMm=775;
 if(!confirm(b))return 10;
 // Archived CCW561 false RED23: low foot and ray misses the near seat.
 clear();sectionLayoutMode=0;
 seats[22].x=-1100;seats[23].x=-900;
 seats[22].y=seats[23].y=-500;
 seats[22].headingDeg=seats[23].headingDeg=270;
 seats[22].pathDistanceMm=seats[23].pathDistanceMm=775;
 hostPose={-1086,202,299};b=component(-5.6f,114);
 if(confirm(b)||seats[23].confirmed)return 11;
 // Rejecting false RED23 must leave the opposite real GREEN22 acquirable.
 hostPose={-1100,-50,270};b=component(0,155);b.color=ColorType::GREEN;
 if(!confirm(b)||!seats[22].confirmed)return 13;
 // CCW562 genuine close RED10 has a larger pose/ray error; retain it.
 clear();seats[10].x=1100;seats[11].x=900;
 seats[10].y=seats[11].y=500;
 seats[10].headingDeg=seats[11].headingDeg=90;
 seats[10].pathDistanceMm=seats[11].pathDistanceMm=775;
 hostPose={1051,-95,55.5f};b=component(26.9f,150);
 if(!confirm(b)||!seats[10].confirmed)return 12;
 std::cout<<"CCW connector alias/false RED23 rejected; compatible far/CHECK_ALL/close RED10 retained\n";
}
'''
    run('ccw_aliases',code)
    run('ccw_without_ray_guard',code.replace(
        'if (error>10.0f)', 'if (false)'),expected=11)

    path=(ROOT/'src/obstacle_path.cpp').read_text()
    geometry=later_lap_fixture().split('int main()')[0]
    geometry+='\nbool earlyMiddleViewActive[OBSTACLE_SEAT_COUNT]={};\n'
    for sig in ('bool earlyMiddleViewEligible(', 'bool earlyMiddleViewPathSafe(',
                'void rebuildLivePath()', 'bool traversingInjectedAvoidance('):
        geometry+=block(path,path.index(sig))+'\n'
    geometry+=r'''
int main(){int count=0;float minWall=1e9,minPillar=1e9;
 const float starts[][3]={{-349,-807,142},{-405,-781,148.4f}};
 for(const auto &a:starts)for(int mirror:{0})for(float dx:{-10.f,0.f,10.f})
 for(float dy:{-10.f,0.f,10.f})for(float dh:{-1.f,0.f,1.f}){
  routeTurnSign=mirror?1:-1;resetState();baseline();
  for(auto &s:seats)s.confirmed=s.injected=false;
  const int red=mirror?5:4,green=mirror?6:7;
  seats[red].confirmed=seats[red].injected=seats[red].red=true;
  seats[green].confirmed=seats[green].injected=true;seats[green].red=false;
  rebuildLivePath();float x=(mirror?-a[0]:a[0])+dx,y=a[1]+dy;
  float h=((mirror?180-a[2]:a[2])+dh)*PI/180;
  ObstacleClearanceSample initial{};calculateClearanceAtPose(seats[red],x,y,h*180/PI,initial);
  const float priorReserve=fminf(30.f,initial.pillarMm);
  progressIndex=nearestPathIndex(livePath,x,y,0,pathLength);bool passed=false;
  for(int step=0;step<1200;++step){
   progressIndex=nearestPathIndex(livePath,x,y,progressIndex,OBSTACLE_PATH_PROGRESS_WINDOW);
   for(int seat:{red,green}){
    ObstacleClearanceSample c{};calculateClearanceAtPose(seats[seat],x,y,h*180/PI,c);
    minWall=fminf(minWall,c.wallMm);if(seat==green)minPillar=fminf(minPillar,c.pillarMm);
    // Some +/-10mm seeds already have <30mm to the previous RED. Preserve
    // that initial reserve within the 1mm tolerance of this 2mm plant step;
    // the upcoming GREEN and walls must retain a full30mm throughout.
    if(c.wallMm<30||c.pillarMm<(seat==red?priorReserve-1.f:30.f)){
     std::cout<<"EARLY ROUTE FAIL "<<x<<","<<y<<" "<<c.wallMm<<" "<<c.pillarMm<<" prior "<<priorReserve<<"\n";return 4;}
   }
   if(y>-200){passed=true;break;}
   float look=firstLapLookahead(livePath[progressIndex].speedMmS);
   if(nearCorner(baselinePath[progressIndex].distanceMm))look*=OBSTACLE_LOOKAHEAD_CORNER_SCALE;
   PositionEstimate p{x,y,h*180/PI};auto t=findLookahead(livePath,p,look);
   float tx=t.x-x,ty=t.y-y,k=2*(-tx*sinf(h)+ty*cosf(h))/fmaxf(1,tx*tx+ty*ty);
   int servo=int(clampFloat(pursuitServoForCurvature(k),-42,42));
   k=abs(servo)<2?0:-1.f/Ackermann::getTurnRadius(servo);float next=h+2*k;
   if(fabsf(k)>1e-6){x+=(sinf(next)-sinf(h))/k;y+=(cosf(h)-cosf(next))/k;}
   else{x+=2*cosf(h);y+=2*sinf(h);}h=next;
  }
  if(!passed)return 5;++count;
 }
 routeTurnSign=-1;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
 seats[19].confirmed=seats[19].injected=true;seats[19].red=true;
 float current=seats[22].pathDistanceMm-650;
 if(traversingInjectedAvoidance(current))return 6;
 seats[20].confirmed=seats[20].injected=true;
 if(!traversingInjectedAvoidance(current))return 7;
 std::cout<<"EARLY GREEN ROUTES "<<count<<" PASS min wall/pillar "<<minWall<<" "<<minPillar<<"\n";
 std::cout<<"PHANTOM20 blocked discovery aim; without alias the aim remains available\n";
}
'''
    run('routes',geometry)


if __name__=='__main__':
    main()
