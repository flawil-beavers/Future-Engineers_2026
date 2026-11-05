"""Replay logged outer-line anchors with ideal independent radius variation.

These are observed pose inputs, not a replay of real traction or ToF truth.
"""
from check_final_parking_sequence import fixture_source
from check_returned_603_and_parking_practice import run

run('parking-returned-645-650', fixture_source()+r'''
#include <cassert>
using namespace parking_module;
int main(){int count=0;float longest=0,largestSteer=0;
 for(int sample=0;sample<3;++sample)
 for(float error:{-5.f,0.f,5.f})for(float yaw:{-1.f,0.f,1.f})
 for(float scale:{1.f,1.15f}){
  int dir=sample==2?1:-1;
  ::routeTurnSign=dir;resetState();baseline();for(auto&s:seats)s.confirmed=s.injected=false;
  final_parking_reset();parking_module::routeTurnSign=dir;outerSensor=dir>0?TOF_RIGHT:TOF_LEFT;
  hostPose={765.3f,(sample==0?-1200.2f:sample==1?-1235.f:-1127.4f)+error,
           (sample==2?359.1f:179.3f)+yaw};
  truePose=hostPose;hostTime=1000;encoder=0;hostGyroHealthy=true;framePeriod=30;
  stop(false);state=FP_SCAN_ALIGN_DRIVE;stateStartMs=scanAlignStartMs=hostTime;
  scanWallRegistered=true;scanAlignSign=1;scanAlignTurns=0;
  bool collision=false;
  for(int tick=0;tick<1200&&state!=FP_SCAN_SETTLE&&!final_parking_aborted();++tick){
   final_parking_update(dir);
   if(dc_state==DC_ENABLED){
    largestSteer=fmaxf(largestSteer,fabsf(steeringCommand));
    float ds=command*.02f;encoder+=ds;
    advanceEntryModel(hostPose,steeringCommand,ds,scale);truePose=hostPose;
    if(!parkingEntryPoseSafe(truePose,steeringCommand))collision=true;
   }hostTime+=20;
  }
  ++count;longest=fmaxf(longest,(hostTime-1000)*.001f);
  if(state!=FP_SCAN_SETTLE||collision||!linePoseReady(hostPose,OBSTACLE_FINAL_PARKING_SCAN_Y_MM)){
   std::cout<<"ALIGN FAIL "<<sample<<" "<<error<<" "<<yaw<<" "<<scale
    <<" pose "<<hostPose.x_mm<<","<<hostPose.y_mm<<","<<hostPose.heading_deg
    <<" state "<<int(state)<<" collision "<<collision<<"\n";return 1;
  }
 }
 std::cout<<"RETURNED ALIGN "<<count<<" PASS longest ideal s "<<longest
  <<" max steer "<<largestSteer<<"\n";
}
''')
