"""Real servo-write age and parking gates; no actuator-duration reduction."""
from check_final_parking_sequence import fixture_source
from check_returned_603_and_parking_practice import run

run('parking-steering-wait', fixture_source()+r'''
#include <cassert>
using namespace parking_module;
void setup(int dir){
 ::routeTurnSign=dir;resetState();baseline();for(auto&s:seats)s.confirmed=s.injected=false;
 final_parking_reset();parking_module::routeTurnSign=dir;scanPhase=5;measuredGapMm=247.5f;
 localToField(60,127,0,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
 truePose=hostPose;hostTime=1000;hostGyroHealthy=true;stop(false);
 robot_logger.writes=0;robot_logger.lines.str("");
}
int main(){
 setup(1);assert(!steering_command_settled(0,400));
 set_steering(0);steer(0);assert(!steering_command_settled(0,400)); // disabled write
 servo_disabled=false;steer(0);hostTime=1399;assert(!steering_command_settled(0,400));
 hostTime=1400;assert(steering_command_settled(0,400));
 stop(true);set_steering(50);steer(50);assert(!steering_command_settled(50,400));
 assert(steering_command_settled(0,400)); // requested != applied
 servo_disabled=false;steer(50);hostTime=1799;assert(!steering_command_settled(50,400));
 hostTime=1800;assert(steering_command_settled(50,400));
 steer(50);assert(steering_command_settled(50,400)); // repeat does not reset clock
 servo_last_command_ms=UINT32_MAX-199;hostTime=200;
 assert(steering_command_settled(50,400)); // unsigned timer wrap
 for(int dir:{-1,1}){
  setup(dir);localToField(270.26f,294.6f,0,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);truePose=hostPose;
  adaptiveEntryCount=1;adaptiveEntry[0]={-1,0,10};segmentIndex=0;
  state=FP_SEGMENT_SETTLE;stateStartMs=0; // old state age must NOT bypass new servo write
  final_parking_update(dir);assert(state==FP_SEGMENT_SETTLE&&dc_state==DC_DISABLED);
  hostTime=1399;final_parking_update(dir);assert(state==FP_SEGMENT_SETTLE);
  hostTime=1400;final_parking_update(dir);assert(state==FP_SEGMENT_DRIVE);
  stop(false);state=FP_SEGMENT_SETTLE;stateStartMs=hostTime;
  final_parking_update(dir);assert(state==FP_SEGMENT_DRIVE); // same physical angle
  setup(dir);servo_disabled=false;steer(0);hostTime=2000;stop(false);state=FP_VERIFY;
  final_parking_update(dir);assert(final_parking_complete()&&robot_logger.writes==1);
  setup(dir);servo_disabled=false;steer(50);hostTime=2000;stop(false);state=FP_VERIFY;
  final_parking_update(dir);assert(!final_parking_complete());
  hostTime=2399;final_parking_update(dir);assert(!final_parking_complete());
  hostTime=2400;final_parking_update(dir);assert(final_parking_complete());
  setup(dir);state=FP_CAPTURE_BRAKE;stateStartMs=1000;hostTime=1299;
  assert(!parkingBrakeReady(300));hostTime=1300;measured_speed=6;
  assert(!parkingBrakeReady(300));measured_speed=0;assert(parkingBrakeReady(300));
  setup(dir);state=FP_SEGMENT_BRAKE;stateStartMs=1000;
  assert(!parkingEntryBrakeReady());hostTime=1149;assert(!parkingEntryBrakeReady());
  hostTime=1150;assert(parkingEntryBrakeReady()); // observed stillness, fresh samples
  setup(dir);state=FP_SEGMENT_BRAKE;stateStartMs=1000;hostSpeedSamples=false;
  assert(!parkingEntryBrakeReady());hostTime=1200;assert(!parkingEntryBrakeReady());
  hostTime=1300;assert(parkingEntryBrakeReady());hostSpeedSamples=true; // old fallback
  setup(dir);state=FP_SEGMENT_BRAKE;stateStartMs=1000;assert(!parkingEntryBrakeReady());
  hostTime=1120;encoder+=.6f;assert(!parkingEntryBrakeReady());
  hostTime=1200;assert(!parkingEntryBrakeReady());
  hostTime=1240;assert(parkingEntryBrakeReady());
  setup(dir);state=FP_SEGMENT_BRAKE;stateStartMs=1000;assert(!parkingEntryBrakeReady());
  hostTime=1120;hostPose.heading_deg+=.3f;assert(!parkingEntryBrakeReady());
  hostTime=1200;assert(!parkingEntryBrakeReady());
  final_parking_sensor_hold();assert(!entryBrakeEvidenceValid);
  setup(dir);state=FP_SEGMENT_BRAKE;stateStartMs=1000;assert(!parkingEntryBrakeReady());
  hostTime=1200;measured_speed=6;assert(!parkingEntryBrakeReady());
  measured_speed=NAN;assert(!parkingEntryBrakeReady());
 }
 std::cout<<"SERVO AGE + CW/CCW ENTRY/VERIFY + BRAKE GATES PASS\n";
}
''')
