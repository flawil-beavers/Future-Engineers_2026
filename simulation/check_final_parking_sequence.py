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
import shutil,subprocess
from check_cw_start_planner import ROOT
from check_connector_servo_resume import block

def main():
    source=(ROOT/'src/final_parking.cpp').read_text()
    fixture=(ROOT/'local_workspace/later-laps/check.cpp').read_text().split('int main()')[0]
    path=(ROOT/'src/obstacle_path.cpp').read_text()
    fixture+=block(path,path.index('bool obstacle_path_parking_approach_target('))
    fixture+=r'''
#include <sstream>
PositionEstimate hostPose,truePose;float encoder=0;unsigned long hostTime=0;
unsigned long framePeriod=30;
enum {DC_DISABLED,DC_ENABLED,DC_HOLDING};int dc_state=DC_HOLDING,command=0,steeringCommand=0,set_degree=0;
bool servo_disabled=true;float current_speed=0,measured_speed=0;
struct Logger {std::ostringstream lines;int writes=0;
 template<class T>void print(T x){lines<<x;}template<class T>void print(T x,int){lines<<x;}
 template<class T>void println(T x){lines<<x<<"\n";}template<class T>void println(T x,int){lines<<x<<"\n";}void println(){lines<<"\n";}
 void write_to_usb(){++writes;}} robot_logger;
unsigned long millis(){return hostTime;}float get_distance(){return encoder;}
PositionEstimate get_position_struct(){return hostPose;}
bool hostGyroHealthy=true;bool gyro_is_healthy(){return hostGyroHealthy;}
void position_apply_xy_correction(float x,float y){hostPose.x_mm+=x;hostPose.y_mm+=y;}
void position_reset(float x,float y,float h){hostPose.x_mm=x;hostPose.y_mm=y;hostPose.heading_deg=h;}
void stop(bool hold){dc_state=hold?DC_HOLDING:DC_DISABLED;command=0;current_speed=measured_speed=0;servo_disabled=true;}
void set_speed(int speed){command=speed;dc_state=DC_ENABLED;current_speed=measured_speed=speed;}
void set_steering(int angle){steeringCommand=set_degree=angle;}
void steer(int angle){if(!servo_disabled)steeringCommand=set_degree=angle;}
struct ObstacleSeatInfo {bool confirmed=false;};
uint8_t obstacle_path_seat_count(){return 24;}
bool obstacle_path_get_seat(uint8_t i,ObstacleSeatInfo &s){s.confirmed=seats[i].confirmed;return true;}
bool obstacle_path_sample_pose_clearance(uint8_t i,float x,float y,float h,ObstacleClearanceSample &s){return calculateClearanceAtPose(seats[i],x,y,h,s);}
enum TofSensor {TOF_LEFT,TOF_RIGHT};
struct TofObjectDiagnostic{int16_t distance_mm=0;bool hardware_valid=true,filter_accepted=true;};
struct TofDiagnosticSnapshot{uint8_t stored_object_count=0;TofObjectDiagnostic objects[4];uint32_t sequence=0,sampled_ms=0;float selected_raw_distance_mm=0;};
bool get_tof_diagnostic_snapshot(TofSensor,TofDiagnosticSnapshot &);
namespace parking_module {
void final_parking_sensor_hold();
'''
    # Arduino/sensor headers are supplied by the host adapter above.
    fixture+=source[source.index('#define Serial robot_logger'):]
    fixture+='\n}\n#undef Serial\n'
    fixture+=r'''
bool get_tof_diagnostic_snapshot(TofSensor sensor,TofDiagnosticSnapshot &s){
 s.sequence=hostTime/framePeriod;s.sampled_ms=s.sequence*framePeriod;
 const float markerRange=parking_module::expectedOuterWallRange(sensor,truePose)-200;
 auto footprint=parking_module::beamFootprint(sensor,markerRange,truePose);
 bool fixed=footprint.maximumX>=480&&footprint.minimumX<=500;
 bool moving=footprint.maximumX>=212.5f&&footprint.minimumX<=232.5f;
 s.selected_raw_distance_mm=(fixed||moving)?markerRange:parking_module::expectedOuterWallRange(sensor,truePose);
 s.stored_object_count=1;s.objects[0].distance_mm=static_cast<int16_t>(s.selected_raw_distance_mm);
 if(fixed||moving){s.stored_object_count=2;s.objects[1].distance_mm=static_cast<int16_t>(parking_module::expectedOuterWallRange(sensor,truePose));s.selected_raw_distance_mm=s.objects[1].distance_mm;}
 return true;
}
int main(){int total=0,bad=0,parked=0,safeRejected=0;
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
  bool frozenServo=false,physicalCollision=false,xBiasInjected=false,gyroPauseInjected=false;
  for(int step=0;step<20000&&!parking_module::final_parking_complete()&&!parking_module::final_parking_aborted();++step){
   if(!xBiasInjected&&parking_module::state==parking_module::FP_SCAN_SETTLE){hostPose.x_mm+=xBias;xBiasInjected=true;}
   if(!gyroPauseInjected&&parking_module::state==parking_module::FP_SEGMENT_DRIVE){
    auto savedState=parking_module::state;auto savedStart=parking_module::stateStartMs;
    hostGyroHealthy=false;parking_module::final_parking_update(dir);
    if(parking_module::final_parking_aborted()||parking_module::state!=savedState||dc_state!=DC_DISABLED)return 1;
    hostTime+=1500;hostGyroHealthy=true;parking_module::final_parking_sensor_resume();
    if(parking_module::stateStartMs!=savedStart+1500)return 1;
    gyroPauseInjected=true;
   }
   parking_module::final_parking_update(dir);
   if(dc_state==DC_ENABLED){
    if(servo_disabled&&steeringCommand!=0)frozenServo=true;
    if(!parking_module::parkingEntryPoseSafe(truePose,servo_disabled?0:steeringCommand))physicalCollision=true;
    float ds=command*.02f;encoder+=ds;
    parking_module::advanceParkingPose(hostPose,servo_disabled?0:steeringCommand,ds);
    parking_module::advanceParkingPose(truePose,servo_disabled?0:steeringCommand,ds);
   }
   hostTime+=20;
  }
  const PositionEstimate saved=hostPose;hostPose=truePose;
  bool realContained=parking_module::finalFootprintContained();hostPose=saved;
  bool completedSafely=parking_module::final_parking_complete()&&!frozenServo&&!physicalCollision&&realContained&&robot_logger.writes==1;
  bool rejectedSafely=xBias!=0&&parking_module::final_parking_aborted()&&!frozenServo&&!physicalCollision&&dc_state!=DC_ENABLED;
  parked+=completedSafely;safeRejected+=rejectedSafely;
  bool ok=completedSafely||rejectedSafely;
  ++total;if(!ok){++bad;std::cout<<"SEQUENCE FAIL "<<dir<<" "<<layout<<" period "<<period<<" bias "<<poseBias<<" xbias "<<xBias<<" collision "<<physicalCollision<<" contained "<<realContained<<" frozen "<<frozenServo<<"\n"<<robot_logger.lines.str();}
 }
 std::cout<<"FINAL SEQUENCE "<<total<<" PARKED "<<parked<<" SAFE_REJECTED "<<safeRejected<<" FAILED "<<bad<<"\n";return bad?1:0;
}
'''
    destination=ROOT/'local_workspace/final-parking-sequence';destination.mkdir(exist_ok=True)
    cpp=destination/'check.cpp';exe=destination/'check.exe';cpp.write_text(fixture)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),'-I',str(ROOT/'local_workspace/later-laps'),str(cpp),'-o',str(exe)],check=True)
    r=subprocess.run([str(exe)],capture_output=True,text=True);(destination/'output.txt').write_text(r.stdout)
    print('\n'.join(line for line in r.stdout.splitlines() if line.startswith(('REGISTRATION','SEQUENCE FAIL','FINAL SEQUENCE'))))
    raise SystemExit(r.returncode)

if __name__=='__main__':main()
