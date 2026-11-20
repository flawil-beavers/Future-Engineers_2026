"""Production scan safety and bounded rescan from logs5/6 numeric poses.
Independent CAD/ideal ToF plant; no original per-object frames or physical truth.
"""
from check_final_parking_sequence import fixture_source
from check_returned_603_and_parking_practice import run
source=fixture_source()+r'''
using namespace parking_module;
void setup(){
 ::routeTurnSign=-1;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
 final_parking_reset();parking_module::routeTurnSign=-1;outerSensor=TOF_LEFT;
 hostTime=1000;hostGyroHealthy=true;encoder=0;framePeriod=30;stop(false);
 robot_logger.lines.str("");robot_logger.lines.clear();robot_logger.writes=0;
}
void invalidScan(){
 state=FP_SCAN_BRAKE;scanPhase=5;measuredFixedInsideX=556.1;measuredMovingInsideX=333.6;
 latestWallPose=hostPose;latestWallRange=expectedOuterWallRange(outerSensor,hostPose);
 stateStartMs=hostTime-500;stoppedWallSequence=0;stoppedWallFrames=0;
}
int main(){
 setup();hostPose=truePose={638.3,-1228.7,181.7};scanPhase=1;
 set_steering(0);bool oldSafe=scanCommandSafe(hostPose,1);
 hostPose.y_mm+=10;truePose=hostPose;bool newSafe=scanCommandSafe(hostPose,1);
 std::cout<<"LOG5 NEUTRAL SWEEP old/new="<<oldSafe<<"/"<<newSafe<<"\n";
 if(oldSafe||!newSafe)return 1;
 setup();hostPose={348.8,-1228.6+10.0,178.6};truePose=hostPose; // No independently measured global X bias is available.
 invalidScan();bool collision=false,retried=false,captured=false;
 for(int tick=0;tick<5000&&!final_parking_aborted();++tick){
  final_parking_update(-1);retried|=scanGeometryRetries==1;
  if(scanPhase==5 && state==FP_CAPTURE_SETTLE){captured=true;break;}
  if(dc_state==DC_ENABLED){
   if(!parkingEntryPoseSafe(truePose,steeringCommand,0))collision=true;
   float ds=command*.02f;encoder+=ds;
   advanceParkingPose(hostPose,steeringCommand,ds);advanceParkingPose(truePose,steeringCommand,ds);
  }hostTime+=20;
 }
 std::cout<<"LOG6 RESCAN retried/captured/collision/gap="<<retried<<"/"<<captured<<"/"<<collision<<"/"<<measuredGapMm<<"\n";
 if(!retried||!captured||collision){std::cout<<robot_logger.lines.str();return 2;}
 // A second bad measurement is never widened or silently used for entry.
 hostPose=truePose={348.8,OBSTACLE_FINAL_PARKING_SCAN_Y_MM,180};stop(false);invalidScan();
 for(int tick=0;tick<10&&!final_parking_aborted();++tick){final_parking_update(-1);hostTime+=30;}
 if(!final_parking_aborted()||dc_state==DC_ENABLED||scanGeometryRetries!=1)return 3;
 std::cout<<"PARKING SCAN RETRY PASS; repeated invalid gap remains rejected\n";return 0;
}
'''
run('parking-cw-log5-6-scan',source)
