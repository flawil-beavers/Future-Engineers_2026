"""Measure command discontinuities at an identical CAD pose on physical layout B.

These are commanded logical servo angles, not observed wheel motion or proof
of the reported jerk's cause. Does not modify firmware or operate the robot.
Run with the IDE Python and host g++; generated files stay in local_workspace.
"""
import sys, subprocess, shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'simulation'))
from check_later_laps import later_lap_fixture
code=later_lap_fixture().split('int main()')[0]+r'''
float servo(const PathPoint *route, PositionEstimate p, int index,float look){
 progressIndex=index;auto target=findLookahead(route,p,look);
 float h=p.heading_deg*PI/180,dx=target.x-p.x_mm,dy=target.y-p.y_mm;
 return clampFloat(pursuitServoForCurvature(2*(-dx*sinf(h)+dy*cosf(h))/fmaxf(1,dx*dx+dy*dy)),-42,42);
}
int main(){
 for(int dir:{-1,1}){
  routeTurnSign=dir;resetState();baseline();int cw[24]={};
  for(int i:{0,4,7,10,15,19,22})cw[i]=(i==4||i==10||i==19)?2:1;
  int cols[24]={};for(int i=0;i<24;++i){int j=dir<0?i:((4-i/6)%4)*6+(2-(i%6)/2)*2+((i%2)^1);cols[j]=cw[i];}
  for(int i=0;i<24;++i){seats[i].confirmed=seats[i].injected=cols[i]!=0;seats[i].red=cols[i]==2;}
  learned();joinSameColourStraightEnds(livePath,true);roundKnownCornerPairs(livePath,true);buildOptimizedPath();
  for(float speed:{135.f,180.f,260.f}){
   PositionEstimate p{livePath[0].x,livePath[0].y,connectorRouteHeading(livePath,0)};
   auto a=servo(livePath,p,0,firstLapLookahead(speed));auto b=servo(optimizedPath,p,0,280);
   auto ta=findLookahead(livePath,p,firstLapLookahead(speed)),tb=findLookahead(optimizedPath,p,280);
   std::cout<<"SEAM dir/speed/servo_before_after/target_jump_mm "<<dir<<" "<<speed<<" "<<a<<" "<<b<<" "<<hypotf(ta.x-tb.x,ta.y-tb.y)<<"\n";
  }
  float maxGate=0,maxStep=0;int gateIndex=0,stepIndex=0;
  for(int i=1;i<pathLength;++i){
   PositionEstimate p{(optimizedPath[i-1].x+optimizedPath[i].x)*.5f,(optimizedPath[i-1].y+optimizedPath[i].y)*.5f,
    atan2f(optimizedPath[i].y-optimizedPath[i-1].y,optimizedPath[i].x-optimizedPath[i-1].x)*180/PI};
   bool before=nearCorner(baselinePath[i-1].distanceMm),after=nearCorner(baselinePath[i].distanceMm);
   if(before!=after){float delta=fabsf(servo(optimizedPath,p,i,280*(before?.65f:1))-servo(optimizedPath,p,i,280*(after?.65f:1)));
    if(delta>maxGate){maxGate=delta;gateIndex=i;}}
   float look=280*(after?.65f:1),delta=fabsf(servo(optimizedPath,p,i-1,look)-servo(optimizedPath,p,i,look));
   if(delta>maxStep){maxStep=delta;stepIndex=i;}
  }
  std::cout<<"DISCONTINUITY dir/gate_deg/index/waypoint_deg/index "<<dir<<" "<<maxGate<<" "<<gateIndex<<" "<<maxStep<<" "<<stepIndex<<"\n";
 }
}
'''
p=ROOT/'local_workspace/later-lap-direct-lanes';p.mkdir(parents=True,exist_ok=True);cpp=p/'transitions.cpp';exe=p/'transitions.exe';cpp.write_text(code)
subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),'-I',str(ROOT/'local_workspace/later-laps'),str(cpp),'-o',str(exe)],check=True)
r=subprocess.run([str(exe)],capture_output=True,text=True,check=True);(p/'transitions.txt').write_text(r.stdout);print(r.stdout)
