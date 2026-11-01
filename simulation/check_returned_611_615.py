"""Returned611--615 regressions; actual production branches, host only."""
from check_connector_servo_resume import block
from check_final_parking_sequence import ROOT, fixture_source
from check_returned_603_and_parking_practice import run


def main():
    path=(ROOT/'src/obstacle_path.cpp').read_text()
    function=block(path,path.index('void updateCornerViewExtra('))
    tracking=function[function.index('    const PositionEstimate pose'):function.index('    switch (cornerViewPhase)')]
    run('611-tracking',r'''
#include <cmath>
#include <cstdint>
#include <iostream>
using std::isfinite;
struct PositionEstimate{float x_mm=0,y_mm=0,heading_deg=0;};
PositionEstimate actual,cornerViewOrigin,cornerViewExtraOrigin,cornerViewExtraReturnPose;
enum{CORNER_VIEW_OBSERVE,CORNER_VIEW_EXTRA_SETTLE,CORNER_VIEW_EXTRA_FORWARD,CORNER_VIEW_EXTRA_BRAKE,CORNER_VIEW_EXTRA_REVERSE,CORNER_VIEW_REBASE_BRAKE};
int cornerViewPhase=CORNER_VIEW_EXTRA_REVERSE,cornerViewExtraSteering=20,cornerViewStation=11;
float encoder=0,cornerViewExtraReturnEncoder=0,cornerViewExtraStartEncoder=0,
 cornerViewOriginEncoder=0,cornerViewDistanceMm=170,cornerViewMeasuredReverseMm=170;
uint32_t cornerViewPhaseMs=0;int cornerViewStoppedFrames=5;
struct Station{int clearFrames[2]={1,1};}discoveryStations[12];
bool locked=false,braked=false;
struct Log{void println(const char*){}}Serial;
uint32_t millis(){return 1000;}
PositionEstimate get_position_struct(){return actual;}
float get_distance(){return encoder;}
float wrap180(float a){while(a>180)a-=360;while(a<-180)a+=360;return a;}
PositionEstimate cornerViewArcPose(PositionEstimate p,int,float){return p;}
void lockCornerView(const char*){locked=true;}
void holdCornerViewCentered(){braked=true;}
void tick(){
''' +tracking+r'''
}
int main(){int checks=0;
 for(float xy:{0.f,5.f,15.f,24.f})for(float dh:{-11.f,-6.f,6.f,11.f}){
  actual={xy,0,dh};cornerViewOrigin={};encoder=-90;locked=braked=false;
  cornerViewPhase=CORNER_VIEW_EXTRA_REVERSE;tick();++checks;
  if(locked||!braked||cornerViewPhase!=CORNER_VIEW_REBASE_BRAKE||cornerViewOriginEncoder!=-90||
     cornerViewDistanceMm!=0||cornerViewStoppedFrames!=0)return 1;
 }
 for(auto p:{PositionEstimate{26,0,6},PositionEstimate{0,0,13},PositionEstimate{NAN,0,0}}){
  actual=p;locked=braked=false;cornerViewPhase=CORNER_VIEW_EXTRA_REVERSE;tick();++checks;
  if(!locked)return 2;
 }
 actual={0,0,2};locked=braked=false;cornerViewPhase=CORNER_VIEW_EXTRA_REVERSE;tick();++checks;
 if(locked||braked||cornerViewPhase!=CORNER_VIEW_EXTRA_REVERSE)return 3;
 std::cout<<"611 TRACKING "<<checks<<" PASS; mild drift reobserves, severe/invalid stops\n";
}
''')
    run('612-615-parking',fixture_source()+r'''
using namespace parking_module;
int main(){int checks=0,quick=0;
 for(int dir:{-1,1}){
  ::routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=false;
  final_parking_reset();parking_module::routeTurnSign=dir;scanPhase=5;measuredGapMm=244;
  hostTime=1000;hostGyroHealthy=true;dc_state=DC_DISABLED;
  localToField(270,296,0,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);truePose=hostPose;
  outerSensor=dir>0?TOF_RIGHT:TOF_LEFT;
  const float y=hostPose.y_mm;
  for(int i=0;i<20;++i){hostTime+=40;truePose.y_mm+=.1f;
   if(updateOuterWallReference(false))return 4;++checks;}
  if(hostPose.y_mm!=y)return 5;
  //Actual capture-limit dispatch must brake/replan, rather than abort.
  state=FP_CAPTURE_DRIVE;captureFieldMotionSign=1;
  captureTargetX=hostPose.x_mm+100;stateStartDistance=encoder=0;
  stateStartMs=hostTime-OBSTACLE_FINAL_PARKING_CAPTURE_TIMEOUT_MS-1;
  final_parking_update(dir);++checks;
  if(state!=FP_REPLAN_BRAKE||final_parking_aborted()){
   std::cout<<"CAPTURE FAIL dir/state="<<dir<<"/"<<int(state)<<"\n"<<robot_logger.lines.str();return 6;}
  for(float x:{265.f,270.f,275.f})for(float yaw:{-1.f,0.f,1.f}){
   localToField(x,296,yaw,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
   truePose=hostPose;robot_logger.lines.str("");
   entryRadiusScale=1;entryRadiusMeasured=false;
   if(!planParkingEntry(hostPose)){std::cout<<"PLAN FAIL "<<dir<<"/"<<x<<"/"<<yaw<<"\n";return 7;}
   if(robot_logger.lines.str().find("Checked quick recipe")!=std::string::npos)++quick;
   PositionEstimate local=entryLocalPose(adaptiveEndpoints[adaptiveEntryCount-1]);
   if(fabsf(local.x_mm-OBSTACLE_FINAL_PARKING_TARGET_LOCAL_X_MM)>1 || local.y_mm>131){std::cout<<"END FAIL "<<local.x_mm<<"/"<<local.y_mm<<"\n";return 8;}
   ++checks;
  }
 }
 if(quick!=18)return 9;
 std::cout<<"612--615 PARKING "<<checks<<" PASS; quick recipes="<<quick<<"/18\n";
}
''')


if __name__=='__main__':
    main()
