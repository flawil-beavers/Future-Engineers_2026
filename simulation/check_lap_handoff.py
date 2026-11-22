"""Extract the real lap-1 -> lap-2 handoff; no firmware upload or motion.

Same-pose command continuity, distance/state/safety caps, and full B-layout
rollouts with slow outgoing discovery speeds. Hardware latency remains untested.
"""
import shutil
import subprocess
from check_cw_start_planner import ROOT
from check_later_laps import later_lap_fixture

code = later_lap_fixture().split('int main()')[0] + r'''
#include <cassert>
void layoutB(int dir){
 routeTurnSign=dir;resetState();baseline();int cw[24]={};
 for(int i:{0,4,7,10,15,19,22})cw[i]=(i==4||i==10||i==19)?2:1;
 int colors[24]={};for(int i=0;i<24;++i){
  int j=dir<0?i:((4-i/6)%4)*6+(2-(i%6)/2)*2+((i%2)^1);colors[j]=cw[i];}
 for(int i=0;i<24;++i){seats[i].confirmed=seats[i].injected=colors[i]!=0;seats[i].red=colors[i]==2;}
 learned();joinSameColourStraightEnds(livePath,true);roundKnownCornerPairs(livePath,true);
}
int main(){
 for(int startMiddle:{0,1,2})for(int dir:{-1,1})for(float speed:{135.f,180.f,260.f}){
  layoutB(dir);
  if(startMiddle){
   for(auto &s:seats)s.confirmed=s.injected=false;
   const int middle=dir<0?2:3;
   seats[middle].confirmed=seats[middle].injected=true;seats[middle].red=startMiddle==2;
   learned();
  }
  lap1CommandValid=true;lap1SpeedMmS=speed;
  lap1LookaheadMm=firstLapLookahead(speed);
  if(nearCorner(baselinePath[0].distanceMm))lap1LookaheadMm*=OBSTACLE_LOOKAHEAD_CORNER_SCALE;
  PositionEstimate pose{livePath[0].x,livePath[0].y,connectorRouteHeading(livePath,0)};
  auto outgoing=findLookahead(livePath,pose,lap1LookaheadMm);
  lapBoundaryPending=true;assert(completePendingLap());assert(lapHandoffActive);
  float look=280;if(nearCorner(baselinePath[0].distanceMm))look*=OBSTACLE_LOOKAHEAD_CORNER_SCALE;
  auto normal=laterLapLookaheadTarget(optimizedPath,pose,progressIndex,look);
  float w=lapHandoffWeight();assert(w==0);
  auto first=lapHandoffTarget(optimizedPath,pose,look,w);
  assert(hypotf(first.x-outgoing.x,first.y-outgoing.y)<.001f);
  assert(lapHandoffSpeed(520,w)==speed);
  float previous=0;
  for(int mm=0;mm<=400;++mm){
   simulatedEncoderMm=mm;w=lapHandoffWeight();assert(w>=previous&&w<=1);
   assert(lapHandoffSpeed(100,w)<=100); // never raise a lower route/safety cap
   assert(lapHandoffSpeed(520,w)>=speed&&lapHandoffSpeed(520,w)<=520);
   auto target=lapHandoffTarget(optimizedPath,pose,look,w);
   assert(hypotf(target.x-(outgoing.x+w*(normal.x-outgoing.x)),
                target.y-(outgoing.y+w*(normal.y-outgoing.y)))<.001f);
   previous=w;
  }
  assert(w==1&&!lapHandoffActive);
  auto end=lapHandoffTarget(optimizedPath,pose,look,w);
  assert(hypotf(end.x-normal.x,end.y-normal.y)<.001f);
  simulatedEncoderMm=0;assert(lapHandoffWeight()==1); // no rearm after wrap
  lapHandoffActive=true;lapHandoffTravelMm=100;
  simulatedEncoderMm=100;float before=lapHandoffWeight();
  for(int i=0;i<1000;++i)assert(lapHandoffWeight()==before); // waiting
  simulatedEncoderMm=90;assert(lapHandoffWeight()==before); // reverse/noise
  completedLaps=2;assert(lapHandoffWeight()==1&&!lapHandoffActive);
  lapHandoffActive=true;completedLaps=1;runtimeTestMode=true;
  assert(lapHandoffWeight()==1&&!lapHandoffActive);
  layoutB(dir);simulatedLap1SpeedMmS=speed;
  float wall,pillar;assert(driveTwoLaps(0,wall,pillar));assert(wall>=40&&pillar>=30);
  std::cout<<"HANDOFF PASS dir/speed/old_target_jump_mm/new_jump_mm/wall/pillar "
   <<dir<<" "<<speed<<" "<<hypotf(outgoing.x-normal.x,outgoing.y-normal.y)
   <<" 0 "<<wall<<" "<<pillar<<"\n";
 }
 resetState();assert(!lapHandoffActive&&!lap1CommandValid&&lapHandoffWeight()==1);
}
'''
dest = ROOT/'local_workspace/lap-handoff'
dest.mkdir(parents=True, exist_ok=True)
cpp, exe = dest/'check.cpp', dest/'check.exe'
cpp.write_text(code)
subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'include'),
                '-I', str(ROOT/'local_workspace/later-laps'), str(cpp), '-o', str(exe)], check=True)
run = subprocess.run([str(exe)], capture_output=True, text=True)
(dest/'output.txt').write_text(run.stdout+run.stderr)
print(run.stdout+run.stderr)
run.check_returncode()
