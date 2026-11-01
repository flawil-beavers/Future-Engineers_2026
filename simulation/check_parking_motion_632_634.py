"""Recorded parking poses: completion with observed radii and settling error.

The pose estimator is not independent physical ground truth. This regression
checks the production state machine against a separate kinematic plant only.
"""
from check_final_parking_sequence import fixture_source
from check_returned_603_and_parking_practice import run


if __name__ == '__main__':
    run('632-634-motion', fixture_source()+r'''
using namespace parking_module;
void plant(PositionEstimate &p,int steer,float ds,float scale){
 const float radius=Ackermann::getTurnRadius(float(steer));
 const float k=abs(steer)<2?0:-1.f/(radius*scale);
 const float h=p.heading_deg*PI/180.f,next=h+k*ds;
 if(fabsf(k)>1e-6f){p.x_mm+=(sinf(next)-sinf(h))/k;p.y_mm+=(cosf(h)-cosf(next))/k;}
 else{p.x_mm+=ds*cosf(h);p.y_mm+=ds*sinf(h);}
 p.heading_deg=parking_module::wrap180(next*180.f/PI);
}
int main(){int total=0,passed=0;
 struct Case{float gap,x,y,h,radius;};
 // 632 first settled arc and the later repeated-clearance pose;
 // 634 first settled arc; 633 full-entry capture (mirrored for CW).
 for(auto c:{Case{252.1f,143.8f,246.4f,56.4f,1.169f},
              Case{252.1f,66.7f,116.8f,44.8f,1.215f},
              Case{249.f,161.4f,246.7f,54.9f,1.108f},
              Case{256.6f,270.3f,293.7f,-.2f,1.163f}})
 for(int dir:{-1,1})for(float yaw:{-1.5f,0.f,1.5f})for(float overrun:{0.f,1.5f}){
  ::routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  final_parking_reset();parking_module::routeTurnSign=dir;scanPhase=5;measuredGapMm=c.gap;
  localToField(c.x,c.y,c.h+yaw,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
  truePose=hostPose;hostTime=1000;encoder=0;hostGyroHealthy=true;
  entryRadiusScale=c.radius;entryRadiusMeasured=c.y<250;
  stop(false);robot_logger.lines.str("");robot_logger.lines.clear();robot_logger.writes=0;
  state=FP_REPLAN_BRAKE;stateStartMs=0;entryReplanReason="recorded_pose";
  bool collision=false;
  for(int tick=0;tick<6000&&!final_parking_complete()&&!final_parking_aborted();++tick){
   const bool moving=dc_state==DC_ENABLED;const int oldCommand=command,oldSteer=steeringCommand;
   final_parking_update(dir);
   float distance=dc_state==DC_ENABLED?command*.02f:0;
   if(moving&&dc_state!=DC_ENABLED)distance=oldCommand<0?-overrun:overrun;
   const int steer=dc_state==DC_ENABLED?steeringCommand:oldSteer;
   while(fabsf(distance)>.0001f){float ds=(distance<0?-1:1)*fminf(1,fabsf(distance));
    distance-=ds;encoder+=ds;plant(hostPose,steer,ds,c.radius);truePose=hostPose;
    if(!parkingEntryPoseSafe(truePose,steer,0))collision=true;
   }hostTime+=20;
  }
  bool ok=final_parking_complete()&&finalFootprintContained()&&!collision;
  ++total;passed+=ok;
  if(!ok)std::cout<<"RETURNED FAIL "<<c.x<<"/"<<c.y<<"/"<<dir<<"/"<<yaw<<"/"<<overrun
    <<" collision="<<collision<<"\n"<<robot_logger.lines.str();
 }
 std::cout<<"RETURNED632-634 "<<passed<<"/"<<total<<"\n";
 return passed==total?0:1;
}
''')
