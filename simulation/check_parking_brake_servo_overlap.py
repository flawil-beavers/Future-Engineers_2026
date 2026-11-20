"""Check servo/brake overlap without reducing400ms servo-write age.
Production swept transition rejects unsafe wheels before writing the next angle.
"""
from check_final_parking_sequence import fixture_source
from check_returned_603_and_parking_practice import run
s=fixture_source()+r'''
using namespace parking_module;
void setup(){::routeTurnSign=1;resetState();baseline();for(auto &s:seats)s.confirmed=false;
 final_parking_reset();parking_module::routeTurnSign=1;scanPhase=5;measuredGapMm=249.6;
 hostTime=1000;hostGyroHealthy=true;encoder=0;entrySafetyMargin=3;
 adaptiveEntryCount=2;adaptiveEntry[0]={-1,50,20};adaptiveEntry[1]={-1,0,20};segmentIndex=0;
 localToField(350,400,0,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);truePose=hostPose;
 servo_disabled=false;set_steering(50);steer(50);stop(true);
}
int main(){setup();prepositionEntrySteeringDuringBrake(hostPose);
 if(dc_state!=DC_HOLDING||servo_last_command!=0)return 1;
 hostTime=1150;stop(false);segmentIndex=1;beginEntrySettle();
 if(servo_last_command_ms!=1000)return 2;
 hostTime=1399;final_parking_update(1);if(state!=FP_SEGMENT_SETTLE)return 3;
 hostTime=1400;final_parking_update(1);if(state!=FP_SEGMENT_DRIVE)return 4;
 setup();localToField(110,128,0,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
 adaptiveEntry[0]={-1,0,20};adaptiveEntry[1]={-1,50,20};
 servo_disabled=false;set_steering(0);steer(0);stop(true);
 if(!parkingEntryPoseSafe(hostPose,0)||parkingEntryPoseSafe(hostPose,50))return 5;
 prepositionEntrySteeringDuringBrake(hostPose);
 if(dc_state!=DC_HOLDING||servo_last_command!=0)return 6;
 std::cout<<"BRAKE/SERVO OVERLAP PASS:400ms retained,150ms overlap; unsafe transition rejected\n";return 0;
}
'''
run('parking-brake-servo-overlap',s)
