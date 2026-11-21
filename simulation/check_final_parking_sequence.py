"""Actual final-parking state machine with CAD motion and ideal cone ToF.
Run check_later_laps.py first to regenerate the shared production fixture.
Exercises brake/servo resume, approach, dual-marker scan, capture, seven
segments and final containment. Uses independent physical/estimated poses,
initial lateral bias of +/-40 mm and scan-stage X registration errors of
+/-180 mm (injected after the swept approach),
and mixed wall/marker returns. Nominal longitudinal poses must park; large
longitudinal perturbations may be safely rejected, but must never collide or
claim completion outside the bay. Hardware lag/slip/reflections are excluded.
"""
import shutil,subprocess,sys
from check_cw_start_planner import ROOT
from check_connector_servo_resume import block
from check_later_laps import later_lap_fixture

def fixture_source():
    source=(ROOT/'src/final_parking.cpp').read_text()
    fixture=later_lap_fixture().split('int main()')[0]
    # Parking owns the hardware encoder adapter. Keep a declaration for the
    # preceding route functions instead of defining a second get_distance().
    fixture=fixture.replace('float get_distance(){return simulatedEncoderMm;}',
                            'float get_distance();')
    fixture=fixture.replace('void resetState(){',
        'void resetHostServo();\nvoid resetState(){resetHostServo();')
    path=(ROOT/'src/obstacle_path.cpp').read_text()
    fixture+=block(path,path.index('bool obstacle_path_parking_approach_target('))
    fixture+=r'''
#include <sstream>
PositionEstimate hostPose,truePose;float encoder=0;unsigned long hostTime=0;
unsigned long framePeriod=30;
enum {DC_DISABLED,DC_ENABLED,DC_HOLDING};int dc_state=DC_HOLDING,command=0,steeringCommand=0,set_degree=0;
bool servo_disabled=true;float current_speed=0,measured_speed=0;
float last_loop_time=.01f;
struct Logger {std::ostringstream lines;int writes=0;
 size_t remaining()const{return 196608;}
 template<class T>void print(T x){lines<<x;}template<class T>void print(T x,int){lines<<x;}
 template<class T>void println(T x){lines<<x<<"\n";}template<class T>void println(T x,int){lines<<x<<"\n";}void println(){lines<<"\n";}
 void write_to_usb(){++writes;}} robot_logger;
unsigned long speed_measurement_count=0;bool hostSpeedSamples=true;
unsigned long millis(){return hostTime;}float get_distance(){if(hostSpeedSamples)speed_measurement_count=hostTime/50;return encoder;}
PositionEstimate get_position_struct(){return hostPose;}
bool hostGyroHealthy=true;bool gyro_is_healthy(){return hostGyroHealthy;}
void position_apply_xy_correction(float x,float y){hostPose.x_mm+=x;hostPose.y_mm+=y;}
void position_reset(float x,float y,float h){hostPose.x_mm=x;hostPose.y_mm=y;hostPose.heading_deg=h;}
void stop(bool hold){dc_state=hold?DC_HOLDING:DC_DISABLED;command=0;current_speed=measured_speed=0;servo_disabled=true;}
void set_speed(int speed){command=speed;dc_state=DC_ENABLED;current_speed=measured_speed=speed;}
void set_steering(int angle){steeringCommand=set_degree=angle;}
struct HostServo {void write(int angle){steeringCommand=angle-SERVO_CENTER;}} servo;
@@STEER@@
void resetHostServo(){servo_last_command=1000;servo_last_command_ms=0;}
struct ObstacleSeatInfo {bool confirmed=false;float xMm=0,yMm=0;};
uint8_t obstacle_path_seat_count(){return 24;}
bool obstacle_path_get_seat(uint8_t i,ObstacleSeatInfo &s){s.confirmed=seats[i].confirmed;s.xMm=seats[i].x;s.yMm=seats[i].y;return true;}
void update_gyro(){}
bool obstacle_path_sample_pose_clearance(uint8_t i,float x,float y,float h,ObstacleClearanceSample &s){return calculateClearanceAtPose(seats[i],x,y,h,s);}
enum TofSensor {TOF_LEFT,TOF_RIGHT,TOF_REAR};
struct TofObjectDiagnostic{int16_t distance_mm=0;bool hardware_valid=true,filter_accepted=true;};
struct TofDiagnosticSnapshot{uint8_t stored_object_count=0;TofObjectDiagnostic objects[4];uint32_t sequence=0,sampled_ms=0;float selected_raw_distance_mm=0,filtered_distance_mm=0,selected_signal_mcps=1,selected_sigma_mm=1;};
bool get_tof_diagnostic_snapshot(TofSensor,TofDiagnosticSnapshot &);
namespace parking_module {
void final_parking_sensor_hold();
'''
    motor=(ROOT/'src/motor_control.cpp').read_text()
    fixture=fixture.replace('@@STEER@@',motor[motor.index('static int servo_last_command'):motor.index('void set_steering(int angle)')])
    # Arduino/sensor headers are supplied by the host adapter above.
    fixture+=source[source.index('#define Serial robot_logger'):]
    # Preserve the old radial-cone model only as a host sensor adapter and a
    # negative control. Production now projects marker edges onto their plane.
    fixture+=r'''
BeamFootprint beamFootprint(TofSensor sensor,float rangeMm,const PositionEstimate &pose){
 const float localX=sensor==TOF_LEFT?OBSTACLE_TOF_LEFT_LOCAL_X_MM:OBSTACLE_TOF_RIGHT_LOCAL_X_MM;
 const float localY=sensor==TOF_LEFT?OBSTACLE_TOF_LEFT_LOCAL_Y_MM:OBSTACLE_TOF_RIGHT_LOCAL_Y_MM;
 const float h=pose.heading_deg*PI/180;
 const float sx=pose.x_mm+cosf(h)*localX-sinf(h)*localY;
 const float ray=h+(sensor==TOF_LEFT?.5f*PI:-.5f*PI);
 const float half=.5f*OBSTACLE_PARKING_EXIT_TOF_DETECTION_FOV_DEG*PI/180;
 const float x1=sx+rangeMm*cosf(ray-half),x2=sx+rangeMm*cosf(ray+half);
 return {fminf(x1,x2),fmaxf(x1,x2)};
}
}
#undef Serial
'''
    fixture+=r'''
float hostWallRangeOverride=NAN;
float hostRearRange=NAN;
bool get_tof_diagnostic_snapshot(TofSensor sensor,TofDiagnosticSnapshot &s){
 s.sequence=hostTime/framePeriod;s.sampled_ms=s.sequence*framePeriod;
 if(sensor==TOF_REAR){
  if(!isfinite(hostRearRange))return false;
  s.selected_raw_distance_mm=hostRearRange;
  return true;
 }
 if(isfinite(hostWallRangeOverride)){
  s.selected_raw_distance_mm=hostWallRangeOverride;s.stored_object_count=1;
  s.objects[0].distance_mm=static_cast<int16_t>(hostWallRangeOverride);return true;
 }
 const float markerRange=parking_module::expectedOuterWallRange(sensor,truePose)-200;
 auto footprint=parking_module::beamFootprint(sensor,markerRange,truePose);
 bool fixed=footprint.maximumX>=480&&footprint.minimumX<=500;
 bool moving=footprint.maximumX>=212.5f&&footprint.minimumX<=232.5f;
 s.selected_raw_distance_mm=(fixed||moving)?markerRange:parking_module::expectedOuterWallRange(sensor,truePose);
 s.stored_object_count=1;s.objects[0].distance_mm=static_cast<int16_t>(s.selected_raw_distance_mm);
 if(fixed||moving){s.stored_object_count=2;s.objects[1].distance_mm=static_cast<int16_t>(parking_module::expectedOuterWallRange(sensor,truePose));s.selected_raw_distance_mm=s.objects[1].distance_mm;}
 return true;
}
'''
    return fixture


def main():
    fixture=fixture_source()
    fixture+=r'''
bool checkLoggedCapture(int dir){
 namespace park=parking_module;
 routeTurnSign=dir;resetState();baseline();park::final_parking_reset();park::routeTurnSign=dir;
 for(auto &seat:seats)seat.confirmed=seat.injected=false;
 const int cw[]={0,4,7,10,14,21},ccw[]={1,5,8,15,19,22};
 for(int i=0;i<6;++i)seats[dir<0?cw[i]:ccw[i]].confirmed=true;
 hostPose=dir<0?PositionEstimate{244.4f,-1179.9f,180.1f}:PositionEstimate{145.8f,-1231.6f,2.1f};
 park::measuredGapMm=dir<0?233.8f:259.4f;
 park::scanPhase=5;
 park::captureTargetX=dir<0?219.74f:480.86f;
 park::captureFieldMotionSign=dir;
 if(!park::chooseRegisteredCapture(hostPose) || park::captureAlignPending!=(dir<0))
  return false;
 bool align=park::captureAlignPending;
 int sign=align?1:park::captureFieldMotionSign;
 for(int mm=0;mm<=650;++mm){
  if(align && hostPose.x_mm>=park::captureTargetX+80.0f &&
      park::linePoseReady(hostPose,park::captureLineY)){align=false;sign=-1;}
  if(!align && fabsf(hostPose.x_mm-park::captureTargetX)<=1.5f){
   std::cout<<"LOGGED CAPTURE dir/travel/line_y "<<dir<<"/"<<mm<<"/"<<park::captureLineY<<"\n";
   return park::fixedEntryCaptureReady(hostPose);
  }
  int steering=0;
  if(!park::captureSteering(hostPose,sign,steering))return false;
  park::advanceParkingPose(hostPose,steering,park::motorDirectionForFieldMotion(sign));
 }
 return false;
}

bool checkMixedStoppedWallLog23(){
 namespace park=parking_module;
 routeTurnSign=-1;resetState();baseline();park::final_parking_reset();
 for(auto &seat:seats)seat.confirmed=seat.injected=false;
 park::routeTurnSign=-1;park::outerSensor=TOF_LEFT;
 park::measuredFixedInsideX=496.0f;park::state=park::FP_SCAN_DRIVE;park::scanPhase=3;
 framePeriod=30;hostTime=89405;hostPose={393.5f,-1208.8f,178.0f};
 hostWallRangeOverride=252.0f;
 if(!park::updateOuterWallReference(false))return false;
 park::measuredMovingInsideX=255.3f;park::state=park::FP_SCAN_BRAKE;park::scanPhase=5;
 hostPose={258.1f,-1206.8f,180.5f};
 hostWallRangeOverride=241.5f;hostTime=92040;
 const bool mixedFan=!park::registeredRailFanClear(hostPose,true);
 const bool refreshed=park::refreshStoppedScanWallReference();
 hostWallRangeOverride=NAN;
 if(!mixedFan||!refreshed||park::latestWallRange!=252.0f||!park::applyScanLocalization()||
    !park::chooseRegisteredCapture(hostPose)||park::captureAlignPending)return false;
 std::cout<<"LOG23 CLEAR GAP WALL PASS; direct capture without alignment return\n";
 return true;
}

bool checkCcwRailApproachLog28(){
 namespace park=parking_module;
 routeTurnSign=1;resetState();baseline();park::final_parking_reset();
 for(auto &seat:seats)seat.confirmed=seat.injected=false;
 for(int i:{1,5,9,14,19,22}){seats[i].confirmed=seats[i].injected=true;seats[i].red=i!=1&&i!=9;}
 learned();
 park::routeTurnSign=1;park::outerSensor=TOF_RIGHT;
 PositionEstimate arrival{-172.1f,-881.8f,317.7f};
 if(!park::chooseParkingApproach(arrival)||!park::approachSmoothCcw)
  return false;
 bool outer=false;int maximum=0;PositionEstimate sample=arrival;int result=0;
 for(int n=0;n<1200&&result==0;++n){
  if(outer){
   int steering=0;
   if(!park::parkingApproachSteering(sample,true,steering))return false;
   maximum=std::max(maximum,abs(steering));
  }
  result=park::advanceApproachPreflight(sample,outer);
 }
 if(result!=1||maximum>22)return false;

 hostTime=1000;framePeriod=30;encoder=0;stop(true);
 hostPose={685.9f,-1209.8f,359.2f};
 truePose=hostPose;
 truePose.y_mm+=175.0f-park::expectedOuterWallRange(TOF_RIGHT,hostPose);
 park::state=park::FP_APPROACH_BRAKE;park::stateStartMs=hostTime;
 robot_logger.writes=0;robot_logger.lines.str("");robot_logger.lines.clear();
 bool continued=false;
 float clearX=0.0f;
 for(int n=0;n<4000&&park::state!=park::FP_FIXED_SETTLE&&!park::final_parking_aborted();++n){
  // Keep the short return ambiguous until the whole fan clears the rails.
  hostWallRangeOverride=park::scanWallRegistered?NAN:175.0f;
  park::final_parking_update(1);steer(set_degree);
  const bool clearing=park::state==park::FP_APPROACH_WALL_CLEAR;
  continued|=clearing;
  if(clearX==0 && park::scanWallRegistered)clearX=hostPose.x_mm;
  if(dc_state==DC_ENABLED){
   if(clearing && (command<=0||abs(set_degree)>10))return false;
   if(!park::parkingEntryPoseSafe(truePose,steeringCommand))return false;
   const float ds=command*.02f;encoder+=ds;
   park::advanceParkingPose(hostPose,steeringCommand,ds);
   park::advanceParkingPose(truePose,steeringCommand,ds);
  }
  hostTime+=20;
 }
 hostWallRangeOverride=NAN;
 hostPose=truePose;
 const bool ok=continued&&clearX>685.9f&&park::state==park::FP_FIXED_SETTLE&&
  park::fixedEntryCaptureReady(hostPose)&&robot_logger.writes==0;
 std::cout<<"LOG28 progressive approach max_servo="<<maximum<<" "
  <<(ok?"PASS":"FAIL")<<" clear_x="<<clearX<<" state="<<int(park::state)<<"\n";
 if(!ok)std::cout<<robot_logger.lines.str();
 return ok;
}

bool checkRollingWallLog40(){
 namespace park=parking_module;
 routeTurnSign=1;resetState();baseline();park::final_parking_reset();
 for(auto &seat:seats)seat.confirmed=seat.injected=false;
 park::routeTurnSign=1;park::outerSensor=TOF_RIGHT;park::approachSmoothCcw=true;
 park::approachOuterShiftX=350;park::state=park::FP_APPROACH_OUTER;
 hostTime=1000;park::stateStartMs=hostTime;framePeriod=30;encoder=0;stop(true);
 hostPose={817.4f,-1212.3f,358.9f};truePose=hostPose;truePose.y_mm-=66;
 hostWallRangeOverride=185.0f;
 park::final_parking_update(1);
 const bool waiting=park::state==park::FP_APPROACH_OUTER && !park::scanWallRegistered;
 hostTime+=60;
 park::final_parking_update(1);
 hostWallRangeOverride=NAN;
 const bool corrected=park::scanWallRegistered && hostPose.y_mm<-1270.0f &&
   park::state==park::FP_APPROACH_OUTER && dc_state==DC_ENABLED;
 if(waiting&&corrected)std::cout<<"LOG40 CLEAR FAN WALL REGISTERED WHILE MOVING PASS\n";
 return waiting&&corrected;
}


void startCapture(int dir,float gap,float headingError,float lineOffset=0){
 namespace park=parking_module;
 routeTurnSign=dir;resetState();baseline();park::final_parking_reset();park::routeTurnSign=dir;
 for(auto &seat:seats)seat.confirmed=seat.injected=false;
 hostTime=1000;encoder=0;stop(true);
 robot_logger.lines.str("");robot_logger.lines.clear();robot_logger.writes=0;
 park::scanPhase=5;park::outerSensor=dir<0?TOF_LEFT:TOF_RIGHT;
 park::measuredGapMm=gap;
 park::captureLineY=park::parkingScanLineY();
 park::captureTargetX=dir>0?park::movingInsideFaceX()+park::captureRearAxleLocalX()
  :OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM-park::captureRearAxleLocalX();
 truePose=hostPose={park::captureTargetX,park::captureLineY+lineOffset,
  park::baseHeadingDeg()+headingError};
 park::state=park::FP_CAPTURE_BRAKE;park::stateStartMs=hostTime;
}

bool checkCaptureTolerance(int dir,float headingError){
 namespace park=parking_module;
 startCapture(dir,247.5f,headingError);
 if(headingError==0){
  const float rearX=hostPose.x_mm-dir*OBSTACLE_FINAL_PARKING_REAR_OVERHANG_MM;
  const float entryFace=dir>0?OBSTACLE_PARKING_FIXED_INNER_FACE_X_MM:park::movingInsideFaceX();
  if(fabsf(rearX-entryFace)>.01f)return false;
 }
 hostTime+=OBSTACLE_FINAL_PARKING_HOLD_BRAKE_MS;
 park::final_parking_update(dir);
 const bool accepted=fabsf(headingError)<=10.0f;
 if(!accepted)return park::final_parking_aborted()&&dc_state==DC_DISABLED;
 if(park::state!=park::FP_FIXED_SETTLE||park::segmentIndex!=0)return false;
 for(int tick=0;tick<100;++tick){
  park::final_parking_update(dir);steer(set_degree);
  if(dc_state==DC_ENABLED)return command<0&&steeringCommand==50*dir;
  hostTime+=20;
 }
 return false;
}

bool checkRearGuard(int dir){
 namespace park=parking_module;
 startCapture(dir,247.5f,0);
 hostRearRange=20.0f;
 for(int tick=0;tick<100&&!park::final_parking_aborted();++tick){
  park::final_parking_update(dir);steer(set_degree);
  if(dc_state==DC_ENABLED)return false;
  hostTime+=20;
 }
 hostRearRange=NAN;
 return park::final_parking_aborted()&&robot_logger.writes==1;
}

void advanceMeasuredEntry(PositionEstimate &pose,int steering,float distance,float radiusScale){
 if(steering==0){parking_module::advanceParkingPose(pose,0,distance);return;}
 const float curvature=-1.0f/(Ackermann::getTurnRadius(steering)*radiusScale);
 const float heading=pose.heading_deg*PI/180.0f;
 const float next=heading+curvature*distance;
 pose.x_mm+=(sinf(next)-sinf(heading))/curvature;
 pose.y_mm+=(cosf(heading)-cosf(next))/curvature;
 pose.heading_deg=parking_module::wrap180(next*180.0f/PI);
}

bool checkFourStrokes(int dir,bool practice){
 namespace park=parking_module;
 startCapture(dir,261.0f,0,-9.0f);
 if(practice){
  const auto measuredCapture=hostPose;
  park::final_parking_start_entry_practice(dir);
  if(!park::fixedEntryCaptureReady(hostPose)||
     fabsf(park::entryLocalPose(hostPose).x_mm-park::captureRearAxleLocalX())>0.01f)return false;
  // Place the model at the same measured normal capture, while entering
  // through the separate entry-only command.
  hostPose=measuredCapture;
  park::measuredGapMm=261.0f;
  park::captureTargetX=measuredCapture.x_mm;
 }
 truePose=hostPose;
 const int directions[]={-1,-1,-1,1};
 const int steers[]={50,0,-50,50};
 const float targets[]={47.0f,47.0f,22.5f,0.0f};
 int finished=0;
 for(int tick=0;tick<3000&&!park::final_parking_complete()&&!park::final_parking_aborted();++tick){
  const auto previous=park::state;
  const int step=park::segmentIndex;
  // Real DC_HOLDING retains the final drive profile while the encoder stops.
  if(previous==park::FP_VERIFY)current_speed=35.0f;
  park::final_parking_update(dir);steer(set_degree);
  if(previous==park::FP_FIXED_DRIVE&&park::state==park::FP_FIXED_BRAKE){
   if(step!=finished++||fabsf(park::entryLocalPose(hostPose).heading_deg-targets[step])>0.7f)return false;
   if(step==1&&fabsf(park::distanceSince(park::stateStartDistance)-125.0f)>1.0f)return false;
  }
  if(dc_state==DC_ENABLED){
   if(step>=4||command*directions[step]<=0||steeringCommand!=steers[step]*dir||
      !park::parkingEntryPoseSafe(hostPose,steeringCommand,0))return false;
   const float ds=command*.02f;encoder+=ds;
   advanceMeasuredEntry(hostPose,steeringCommand,ds,1.22f);
  }
  hostTime+=20;
 }
 const bool ok=park::final_parking_complete()&&park::finalFootprintContained()&&finished==4&&
  robot_logger.writes==1&&dc_state==DC_HOLDING&&set_degree==0;
 std::cout<<"FIXED FOUR STROKES dir="<<dir<<" practice="<<practice<<" "<<(ok?"PASS":"FAIL")<<"\n";
 if(!ok)std::cout<<robot_logger.lines.str();
 if(!ok)return false;
 for(int tick=0;tick<100;++tick){
  park::final_parking_update(dir);
  if(dc_state!=DC_HOLDING||robot_logger.writes!=1)return false;
 }
 return true;
}

int main(){int total=0,bad=0,captured=0,safeRejected=0;
 // Registration must not accept a single piece, wrong gap, or excessive
 // correction simply because the longitudinal allowance is now larger.
 for(int fault=0;fault<5;++fault){
  routeTurnSign=-1;hostPose={300,-1205.4f,180};
  parking_module::outerSensor=TOF_LEFT;
  parking_module::scanPhase=fault==1?4:5;
  parking_module::measuredFixedInsideX=fault==3?229.f:300.f;
  parking_module::measuredMovingInsideX=parking_module::measuredFixedInsideX-247.5f+(fault==2?20.f:0.f);
  parking_module::latestWallPose=hostPose;
  parking_module::latestWallRange=parking_module::expectedOuterWallRange(TOF_LEFT,hostPose)+(fault==4?26.f:0.f);
  bool ok=parking_module::applyScanLocalization();
  if(ok!=(fault==0)||(fault==0&&fabsf(hostPose.x_mm-480.f)>.01f))return 1;
 }
 std::cout<<"REGISTRATION 5 FAILED 0\n";
 if(!checkMixedStoppedWallLog23())return 1;
 if(!checkCcwRailApproachLog28()||!checkRollingWallLog40())return 1;
 if(!checkLoggedCapture(-1)||!checkLoggedCapture(1))return 1;

 for(int dir:{-1,1}){
  for(float yaw:{-11.0f,-10.0f,0.0f,10.0f,11.0f})
   if(!checkCaptureTolerance(dir,yaw))return 1;
  if(!checkRearGuard(dir)||!checkFourStrokes(dir,false)||!checkFourStrokes(dir,true))return 1;
 }
 const int configs[11][3]={{0,0,0},{1,0,0},{2,0,0},{0,1,0},{0,2,0},{0,0,1},{0,0,2},{1,0,1},{1,0,2},{2,0,1},{2,0,2}};
 for(unsigned long period:{30UL,100UL})for(int dir:{-1,1})for(int layout=0;layout<11;++layout)for(float poseBias:{-40.f,0.f,40.f})for(float xBias:{-180.f,0.f,180.f}){
  framePeriod=period;
  routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  for(int st=0;st<3;++st){int i=2*st+(dir>0),c=configs[layout][st];seats[i].confirmed=seats[i].injected=c;seats[i].red=c==2;}
  learned();joinSameColourStraightEnds(livePath,true);roundKnownCornerPairs(livePath,true);buildOptimizedPath();
  int index=3;for(int i=0;i<pathLength;++i)if(baselinePath[i].distanceMm>=corners[3].pathEndMm && finalCornerVehicleClear(PositionEstimate{optimizedPath[i].x,optimizedPath[i].y,connectorRouteHeading(optimizedPath,i)})){index=i;break;}
  hostPose.x_mm=optimizedPath[index].x;hostPose.y_mm=optimizedPath[index].y;hostPose.heading_deg=connectorRouteHeading(optimizedPath,index);
  truePose=hostPose;hostPose.y_mm+=poseBias;
  parking_module::final_parking_reset();hostTime=0;encoder=0;stop(true);robot_logger.lines.str("");robot_logger.lines.clear();robot_logger.writes=0;
  bool frozenServo=false,physicalCollision=false,xBiasInjected=false;
  for(int step=0;step<20000&&parking_module::state!=parking_module::FP_FIXED_SETTLE&&
      !parking_module::final_parking_aborted();++step){
   if(!xBiasInjected&&parking_module::state==parking_module::FP_SCAN_SETTLE){hostPose.x_mm+=xBias;xBiasInjected=true;}
   parking_module::final_parking_update(dir);
   steer(set_degree); // Service the servo write as drive_loop does.
   if(dc_state==DC_ENABLED){
    if(servo_disabled&&steeringCommand!=0)frozenServo=true;
    if(!parking_module::parkingEntryPoseSafe(truePose,servo_disabled?0:steeringCommand)){
     if(!physicalCollision)std::cout<<"COLLISION state/segment/true/estimated/margin="<<int(parking_module::state)<<"/"<<int(parking_module::segmentIndex)<<"/"<<truePose.x_mm<<"/"<<truePose.y_mm<<"/"<<truePose.heading_deg<<"/"<<hostPose.x_mm<<"/"<<hostPose.y_mm<<"/"<<3.0f<<"\n";
     physicalCollision=true;}
    float ds=command*.02f;encoder+=ds;
    parking_module::advanceParkingPose(hostPose,servo_disabled?0:steeringCommand,ds);
    parking_module::advanceParkingPose(truePose,servo_disabled?0:steeringCommand,ds);
   }
   hostTime+=20;
  }
  bool completedSafely=parking_module::state==parking_module::FP_FIXED_SETTLE&&
   parking_module::fixedEntryCaptureReady(hostPose)&&!frozenServo&&!physicalCollision&&robot_logger.writes==0;
  bool rejectedSafely=xBias!=0&&parking_module::final_parking_aborted()&&!frozenServo&&!physicalCollision&&dc_state!=DC_ENABLED;
  captured+=completedSafely;safeRejected+=rejectedSafely;
  bool ok=completedSafely||rejectedSafely;
  ++total;if(!ok){++bad;std::cout<<"SEQUENCE FAIL "<<dir<<" "<<layout<<" period "<<period<<" bias "<<poseBias<<" xbias "<<xBias<<" collision "<<physicalCollision<<" capture_ready "<<parking_module::fixedEntryCaptureReady(hostPose)<<" frozen "<<frozenServo<<" true_xyh="<<truePose.x_mm<<"/"<<truePose.y_mm<<"/"<<truePose.heading_deg<<" estimated="<<hostPose.x_mm<<"/"<<hostPose.y_mm<<"/"<<hostPose.heading_deg<<"\n"<<robot_logger.lines.str();}
 }
 std::cout<<"FINAL SEQUENCE "<<total<<" CAPTURED "<<captured<<" SAFE_REJECTED "<<safeRejected<<" FAILED "<<bad<<"\n";return bad?1:0;
}
'''
    if '--quick' in sys.argv:
        fixture=fixture.replace('  framePeriod=period;',
            '  if(xBias!=0 || !((dir==-1&&layout==2&&poseBias==-40&&period==30)||(dir==1&&layout==3&&poseBias==40&&period==30)||(dir==-1&&layout==3&&poseBias==40&&period==100)))continue;\n  framePeriod=period;')
    if '--slow-cw' in sys.argv:
        fixture=fixture.replace('  framePeriod=period;',
            '  if(dir!=-1||period!=100||layout!=0||poseBias!=0||xBias!=0)continue;\n  framePeriod=period;')
    destination=ROOT/'local_workspace'/('final-parking-sequence-quick' if '--quick' in sys.argv else 'final-parking-sequence');destination.mkdir(exist_ok=True)
    cpp=destination/'check.cpp';exe=destination/'check.exe';cpp.write_text(fixture)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),'-I',str(ROOT/'local_workspace/later-laps'),str(cpp),'-o',str(exe)],check=True)
    r=subprocess.run([str(exe)],capture_output=True,text=True);(destination/'output.txt').write_text(r.stdout)
    print('\n'.join(line for line in r.stdout.splitlines() if line.startswith(('REGISTRATION','LOG23','LOG28','LOG40','LOGGED CAPTURE','FIXED FOUR STROKES','COLLISION','SEQUENCE FAIL','FINAL SEQUENCE'))))
    raise SystemExit(r.returncode)

if __name__=='__main__':main()
