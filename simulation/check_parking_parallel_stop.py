"""Mirrored rearward capture and stop at a contained pose away from target X."""
from check_final_parking_sequence import fixture_source
from check_returned_603_and_parking_practice import run

run('parking-parallel-stop', fixture_source()+r'''
#include <cassert>
using namespace parking_module;
void setup(int dir,float x,float y,float yaw){
 ::routeTurnSign=dir;resetState();baseline();
 for(auto&s:seats)s.confirmed=s.injected=false;
 final_parking_reset();parking_module::routeTurnSign=dir;
 scanPhase=5;measuredGapMm=247.5f;
 localToField(x,y,yaw,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
 truePose=hostPose;hostTime=1000;encoder=0;hostGyroHealthy=true;
 stop(false);robot_logger.writes=0;robot_logger.lines.str("");
 adaptiveEntryCount=1;adaptiveEntry[0]={1,0,100};segmentIndex=0;
 stateStartMs=0;stateStartDistance=0;entryStartHeading=hostPose.heading_deg;
}
int main(){int n=0;
 static_assert(OBSTACLE_PARKING_EXIT_SPEED==80,"Exit speed unchanged");
 assert(fabsf(OBSTACLE_FINAL_PARKING_CAPTURE_LOCAL_X_MM-260.26f)<.001f);
 assert(OBSTACLE_FINAL_PARKING_TARGET_LOCAL_X_MM==50);
 for(int dir:{-1,1}){
  setup(dir,60,127,0);final_parking_start_entry_practice(dir);
  auto p=entryLocalPose(hostPose);assert(fabsf(p.x_mm-260.26f)<.001f);
  for(float x:{45.f,70.f,100.f}){
   setup(dir,x,127,0);state=FP_SEGMENT_DRIVE;
   servo_disabled=false;steer(0);set_speed(60);final_parking_update(dir);
   assert(state==FP_SEGMENT_BRAKE&&dc_state==DC_HOLDING&&command==0);
   hostTime+=400;final_parking_update(dir);assert(state==FP_VERIFY);
   hostTime+=400;final_parking_update(dir);
   assert(final_parking_complete()&&robot_logger.writes==1);
   assert(fabsf(entryLocalPose(hostPose).x_mm-x)<.001f);++n;
   setup(dir,x,127,0);state=FP_SEGMENT_SETTLE;
   final_parking_update(dir);assert(state==FP_VERIFY&&command==0);++n;
  }
  // Do not stop on alignment alone outside the bay, or with an invalid yaw.
  for(auto p:{PositionEstimate{60,260,0},PositionEstimate{60,127,5}}){
   setup(dir,p.x_mm,p.y_mm,p.heading_deg);state=FP_SEGMENT_DRIVE;
   final_parking_update(dir);assert(state!=FP_SEGMENT_BRAKE&&!final_parking_complete());++n;
  }
 }
 std::cout<<"PARALLEL STOP "<<n<<" PASS; mirrored capture shift PASS\n";
}
''')
