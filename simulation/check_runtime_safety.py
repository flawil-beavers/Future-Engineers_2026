"""Source-backed host regressions for runtime safety; generated files stay ignored.
No upload, sensor access or robot motion. Run with Python and a host g++ on PATH.
"""
from pathlib import Path
import shutil
import subprocess
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'local_workspace/runtime-safety'
OUT.mkdir(parents=True, exist_ok=True)

def function(file, signature):
    text = (ROOT / file).read_text()
    start = text.index(signature)
    opening = text.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]

def check(name, source, extra=()):
    cpp, exe = OUT / (name+'.cpp'), OUT / (name+'.exe')
    cpp.write_text(source)
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-Wall', '-Wextra',
        '-I', str(OUT), '-I', str(ROOT/'include'), str(cpp), *map(str,extra),
        '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)

common = '''#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <iostream>
#include "runtime_safety.h"
#include "run_telemetry.h"
using namespace RuntimeSafety;
struct Printer {template<class T> void print(T,int=0){} template<class T> void println(T,int=0){} void println(){}} Serial;
'''
check('inputs', common+r'''
int main(){
 EnableSwitchFilter f;f.reset(false,0);
 assert(f.update(true,200000,100000)==0);
 assert(f.update(true,300000,100000)==1);
 assert(f.update(false,300001,100000)==-1);
 assert(f.update(false,500000,100000)==0);
 assert(f.update(true,500001,100000)==0);
 assert(f.update(false,550000,100000)==0);
 assert(f.update(true,600000,100000)==0);
 assert(f.update(true,699999,100000)==0);
 assert(f.update(true,700000,100000)==1);
 f.reset(false,0xfffffff0U);assert(f.update(true,0xfffffff0U,32)==0);
 assert(f.update(true,16,32)==1);
 char c=0;int v=0;char *a=nullptr;
 for(const char *bad:{"", "   ", "dnan", "d999999999999999999999999999", "d80oops", "d80 4"}){
  char b[80];strcpy(b,bad);assert(!legacyCommand(b,c,v,a));
 }
 for(const char *good:{"d80", " d -80", "O3", "X-1", "r", "c0", "q120", "s-50"}){
  char b[80];strcpy(b,good);assert(legacyCommand(b,c,v,a));
 }
 assert(sampleFresh(249,0,true,30000,250));assert(!sampleFresh(251,0,true,30000,250));
 assert(sampleFresh(350,0,true,300000,250));assert(!sampleFresh(401,0,true,300000,250));
 assert(sampleFresh(16,0xfffffff0U,true,30000,250));assert(!sampleFresh(10,0,false,30000,250));
 float yaw=123;assert(!quaternionYaw(NAN,0,0,0,yaw));assert(!quaternionYaw(0,0,0,0,yaw));
 assert(quaternionYaw(1,0,0,0,yaw)&&fabs(yaw)<.001);
 assert(quaternionYaw(.70710678f,0,0,.70710678f,yaw)&&fabs(yaw-90)<.001);
 assert(!quaternionYaw(1e30f,0,0,1e30f,yaw));
 assert(quaternionYaw(1.01f,0,0,0,yaw)&&fabs(yaw)<.001);
 std::cout<<"PASS switch bounce/short-off/wrap, strict inputs, ToF age/budget, quaternion validation\n";
}
''')

gyro = common+r'''
uint32_t nowMs=0;uint32_t millis(){return nowMs;}
constexpr uint32_t GYRO_REPORT_TIMEOUT_MS=200,GYRO_RESET_RECOVERY_TIMEOUT_MS=1000;
constexpr int BNO085_INT=1,HIGH=1,LOW=0,SH2_GAME_ROTATION_VECTOR=7;
int pin=LOW,restarts=0;int digitalRead(int){return pin;}
struct Quaternion {float real=1,i=0,j=0,k=0;};
struct Event {int sensorId=7;struct {Quaternion gameRotationVector;} un;} sensor_value;
struct Bno {bool event=true,reset=false;bool getSensorEvent(Event*){return event;}
 bool wasReset(){bool r=reset;reset=false;return r;}} bno;
float current_degree=0,current_heading=0;bool gyro_stream_healthy=false;
uint32_t gyro_last_valid_report_ms=0;
bool restart_gyro_stream(){++restarts;gyro_stream_healthy=false;return true;}
'''+function('src/sensors.cpp','void update_gyro()')+'\n'+function('src/sensors.cpp','bool gyro_is_healthy()')+r'''
void tick(uint32_t delta=50){nowMs+=delta;update_gyro();}
int main(){
 update_gyro();assert(gyro_is_healthy());
 bno.event=false;for(int i=0;i<5;++i)tick();assert(restarts==1&&!gyro_is_healthy());
 bno.event=true;tick(10);assert(gyro_is_healthy());
 sensor_value.sensorId=99;for(int i=0;i<5;++i)tick();assert(restarts==2&&!gyro_is_healthy());
 sensor_value.sensorId=7;tick(10);assert(gyro_is_healthy());
 pin=HIGH;for(int i=0;i<5;++i)tick();assert(restarts==3&&!gyro_is_healthy());
 pin=LOW;tick(10);assert(gyro_is_healthy());
 sensor_value.un.gameRotationVector.real=NAN;tick(10);
 assert(!gyro_is_healthy()&&std::isfinite(current_degree));
 sensor_value.un.gameRotationVector.real=1;tick(10);assert(gyro_is_healthy());
 int prior=restarts;pin=HIGH;tick(1600);assert(restarts==prior&&!gyro_is_healthy());
 pin=LOW;tick(10);assert(gyro_is_healthy());
 // Reset raw yaw zero without changing the continuous navigation angle.
 auto yaw=[](float deg){float rad=deg*3.141592653589793f/360;
  sensor_value.un.gameRotationVector.real=cos(rad);
  sensor_value.un.gameRotationVector.k=sin(rad);};
 yaw(90);tick(10);float before=current_degree;
 bno.reset=true;tick(10);assert(!gyro_is_healthy());
 yaw(-60);tick(10);assert(gyro_is_healthy()&&fabs(current_degree-before)<.001);
 yaw(-50);tick(10);assert(fabs(current_degree-before-10)<.001);
 bno.event=false;for(int i=0;i<22;++i)tick();
 float retained=current_degree; bno.event=true;yaw(170);tick(10);
 assert(gyro_is_healthy()&&fabs(current_degree-retained)<.001);
 yaw(-175);tick(10);assert(fabs(current_degree-retained-15)<.001);
 nowMs+=201;assert(!gyro_is_healthy());
 std::cout<<"PASS actual gyro: stuck LOW/HIGH, unrelated events, NaN, loop pause, fresh recovery\n";
}
'''
check('gyro',gyro)

serial = common+r'''
#include <string>
#include <vector>
constexpr int BUFFER_SIZE=16;
char ringBuffer[BUFFER_SIZE];int head=0,tail=0;bool discarding_message=false;
std::string input;size_t cursor=0;std::vector<std::string> commands;
struct Terminal {int available(){return cursor<input.size();}char read(){return input[cursor++];}
 template<class T>void println(T){}} terminal;
#define Serial terminal
void processMessage();
'''+function('src/serial_handler.cpp','void check_serial_available()')+r'''
void processMessage(){std::string s;while(tail!=head){char c=ringBuffer[tail];tail=(tail+1)%BUFFER_SIZE;if(c=='\n')break;s+=c;}commands.push_back(s);}
int main(){
 input=std::string(40,'x')+"d80\nd60\n";check_serial_available();
 assert(commands.size()==1&&commands[0]=="d60");
 input="d40";cursor=0;check_serial_available();assert(commands.size()==1);
 input="\n";cursor=0;check_serial_available();assert(commands.size()==2&&commands[1]=="d40");
 std::cout<<"PASS actual serial framing: discard entire overflow, partial message preserved\n";
}
'''
check('serial',serial)

mode = common+r'''
enum RobotMode {MODE_NONE,MODE_OBSTACLE_CHALLENGE,MODE_HOLD,MODE_TURN_RADIUS_CAL,MODE_MANUAL};
enum ModeResult {MODE_RESULT_RUNNING,MODE_RESULT_COMPLETED,MODE_RESULT_FAILED};
RobotMode current_mode=MODE_OBSTACLE_CHALLENGE,pending_mode=MODE_NONE;
ModeResult nextResult=MODE_RESULT_COMPLETED;bool parked=true;int stops=0;
ModeResult update_active_mode(){return nextResult;}
const char *mode_name(RobotMode){return "test";}
bool turn_radius_cal_waiting_for_right(){return false;}
bool final_parking_complete(){return parked;}
void stop_mode(RobotMode){++stops;}
'''+function('src/mode_manager.cpp','void mode_update()')+r'''
int main(){
 mode_update();assert(current_mode==MODE_HOLD&&stops==0&&pending_mode==MODE_NONE);
 nextResult=MODE_RESULT_RUNNING;mode_update();assert(current_mode==MODE_HOLD&&stops==0);
 current_mode=MODE_OBSTACLE_CHALLENGE;parked=false;mode_update();assert(stops==0);
 nextResult=MODE_RESULT_FAILED;mode_update();assert(current_mode==MODE_NONE&&stops==1);
 current_mode=MODE_MANUAL;nextResult=MODE_RESULT_COMPLETED;mode_update();assert(stops==2);
 std::cout<<"PASS actual dispatcher: park hold retained, normal failure/completion cleanup preserved\n";
}
'''
check('mode',mode)

pose = common+r'''
struct PositionEstimate {float x_mm=0,y_mm=0,heading_deg=0,confidence_mm=0;} pos;
bool pos_initialized=true;float current_distance=0,prev_distance=0,prev_angle=0,prev_heading_rad=0,total_distance_traveled=0;
float angle=0;float get_angle(){return angle;}
constexpr float PI=3.141592653589793f;int set_degree=0;float last_loop_time=.01;
unsigned long current_time=0,slip_log_timer=0;
bool calibration_has_data(){return false;}float get_calibrated_radius(int){return 0;}
void position_init(float,float,float){}
float wrap_to_180(float v){while(v>180)v-=360;while(v< -180)v+=360;return v;}
float wrap_to_360(float v){while(v>=360)v-=360;while(v<0)v+=360;return v;}
'''+function('src/position_estimator.cpp','void update_position()')+'\n'+function('src/position_estimator.cpp','void position_apply_xy_correction(')+r'''
int main(){
 for(int i=1;i<=100;++i){angle=i*.005f;update_position();}assert(fabs(pos.heading_deg-.5f)<.011f);
 for(int i=1;i<=100;++i){current_distance=i*.005f;update_position();}assert(fabs(total_distance_traveled-.5f)<.011f);
 auto before=pos;angle=NAN;update_position();assert(pos.x_mm==before.x_mm&&pos.heading_deg==before.heading_deg);
 float c=pos.confidence_mm;position_apply_xy_correction(100,20);assert(pos.confidence_mm==c);
 std::cout<<"PASS actual pose: tiny steps accumulate, invalid inputs rejected, correction is not accuracy proof\n";
}
'''
check('pose',pose)

(OUT/'Arduino.h').write_text('''#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <cmath>
using std::max;using std::min;
constexpr int A0=0,A1=1,A2=2;
constexpr double PI=3.141592653589793;
inline uint32_t micros(){return 0;}
''')
check('vision',r'''
#include "vision.h"
#include <cassert>
#include <iostream>
#include <vector>
int main(){
 static Vision v;v.begin();std::vector<uint8_t>b(320*240*2,255);
 assert(v.update(b.data(),320,240));auto white=v.getResult();
 assert(white.qualitySamples==200&&white.meanValue==255&&white.clippedSamples==200);
 std::fill(b.begin(),b.end(),0);assert(v.update(b.data(),320,240));
 assert(v.getResult().darkSamples==200&&v.getResult().meanValue==0);
 assert(!v.update(nullptr,320,240));assert(v.getResult().qualitySamples==0);
 for(auto dims:{std::pair<int,int>{0,240},{320,0},{320,40},{321,240},{320,241},{319,240}})
  assert(!v.update(b.data(),dims.first,dims.second));
 assert(v.update(b.data(),160,120));
 std::cout<<"PASS actual vision: dark/clipped ROI diagnostics, invalid dimensions, compatible smaller frames\n";
}
''',[ROOT/'src/vision.cpp'])
restart = common+r'''
enum {MODE_NONE,MODE_OBSTACLE_CHALLENGE};int current_mode=MODE_OBSTACLE_CHALLENGE;
int stopped=0,cancelled=0,closed=0;bool gyro_stream_healthy=true;
void mode_stop_all(){++cancelled;current_mode=MODE_NONE;}void stop(bool){++stopped;}
void final_parking_sensor_hold(){}
void sh2_close(){++closed;}void delay(int){}
constexpr int BNO085_CS=1,BNO085_INT=2,SH2_GAME_ROTATION_VECTOR=7,GYRO_REPORT_INTERVAL_US=10000;
int SPI1=0,sensor_value=0;
struct Transport {bool beginOk=true,reportOk=true;
 bool begin_SPI(int,int,int*){return beginOk;}bool enableReport(int,int){return reportOk;}
 bool getSensorEvent(int*){return false;}bool wasReset(){return false;}} bno;
'''+function('src/sensors.cpp','static bool restart_gyro_stream()')+r'''
int main(){
 assert(restart_gyro_stream());assert(cancelled==0&&stopped==1&&!gyro_stream_healthy);
 assert(current_mode==MODE_OBSTACLE_CHALLENGE);assert(restart_gyro_stream());assert(cancelled==0);
 current_mode=MODE_OBSTACLE_CHALLENGE;bno.beginOk=false;
 assert(!restart_gyro_stream());assert(cancelled==0&&!gyro_stream_healthy&&current_mode==MODE_OBSTACLE_CHALLENGE);
 std::cout<<"PASS actual gyro transport: active route retained on success/failure, automatic recovery possible\n";
}
'''
check('gyro-restart',restart)

encoder = common+r'''
volatile long encoder_pos=123;volatile int encoder_dir=-1;int mask=0,enables=0;
uint32_t __get_PRIMASK(){return mask;}void __disable_irq(){mask=1;}
void __enable_irq(){mask=0;++enables;}
struct EncoderSnapshot {long count;int direction;};
'''+function('src/motor_control.cpp','EncoderSnapshot get_encoder_snapshot()')+r'''
int main(){
 auto s=get_encoder_snapshot();assert(s.count==123&&s.direction==-1&&mask==0&&enables==1);
 mask=1;encoder_pos=-77;encoder_dir=1;s=get_encoder_snapshot();
 assert(s.count==-77&&s.direction==1&&mask==1&&enables==1);
 std::cout<<"PASS actual encoder snapshots: coherent values, prior interrupt mask preserved\n";
}
'''
check('encoder',encoder)

tof = common+r'''
enum TofSensor {TOF_LEFT,TOF_RIGHT,TOF_REAR,TOF_COUNT};
struct Diagnostic {uint32_t sequence=1,sampled_ms=0,timing_budget_us=30000;} tof_diagnostics[TOF_COUNT];
bool tof_transport_valid[TOF_COUNT]={true,true,true};
float tof_distances[TOF_COUNT]={100,9999,200},tof_raw_distances[TOF_COUNT]={100,-1,200};
float tof_signal_rates[TOF_COUNT]={1,1,1},tof_sigmas[TOF_COUNT]={2,2,2};
uint32_t nowMs=0;uint32_t millis(){return nowMs;}
'''+function('src/sensors.cpp','static bool tof_value_fresh(')+'\n'+function('src/sensors.cpp','float get_tof_distance(')+'\n'+function('src/sensors.cpp','float get_tof_raw_distance(')+r'''
int main(){
 assert(get_tof_distance(TOF_LEFT)==100);assert(get_tof_distance(TOF_RIGHT)==9999);
 nowMs=251;assert(get_tof_distance(TOF_LEFT)==-1&&get_tof_raw_distance(TOF_LEFT)==-1);
 tof_diagnostics[TOF_LEFT].timing_budget_us=300000;assert(get_tof_distance(TOF_LEFT)==100);
 tof_transport_valid[TOF_LEFT]=false;assert(get_tof_distance(TOF_LEFT)==-1);
 assert(get_tof_distance(static_cast<TofSensor>(-1))==-1);
 std::cout<<"PASS actual ToF accessors: fresh gaps preserved, stale/bus-failed values unavailable, 300ms mode supported\n";
}
'''
check('tof',tof)

rear = common+r'''
constexpr int VL53L4CX_ERROR_NONE=0,VL53L4CX_RANGESTATUS_RANGE_VALID=0,VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE=1;
constexpr float TOF_OUT_OF_RANGE_MM=9999,REAR_TOF_MAX_RELIABLE_DISTANCE_MM=600,TOF_MAX_DELTA_MM=50;
constexpr uint32_t TOF_READY_POLL_INTERVAL_US=2000,MEASUREMENT_TIMEOUT_MS=250;
enum {REAR_TOF_RPC_RUNNING=1,REAR_TOF_RPC_BUS_FAILED=3};
struct Candidate {uint8_t RangeStatus=0;int16_t RangeMilliMeter=200;uint32_t SignalRateRtnMegaCps=65536,SigmaMilliMeter=65536;};
struct VL53L4CX_MultiRangingData_t {uint8_t NumberOfObjectsFound=1;Candidate RangeData[4];};
struct Sensor {int readError=0,clearError=0,nextDistance=200;
 int VL53L4CX_GetMeasurementDataReady(uint8_t*r){*r=1;return 0;}
 int VL53L4CX_GetMultiRangingData(VL53L4CX_MultiRangingData_t*d){*d=VL53L4CX_MultiRangingData_t{};d->RangeData[0].RangeMilliMeter=nextDistance;return readError;}
 int VL53L4CX_ClearInterruptAndStartMeasurement(){return clearError;}} rearSensor;
struct Frame {int status=0;float filtered_distance_mm=0,raw_distance_mm=0,signal_mcps=0,sigma_mm=0;uint32_t sequence=0;} frame;
float previousDistanceMm=-1;uint32_t lastReadyPollUs=0,lastMeasurementMs=0,nowUs=3000;
uint32_t micros(){return nowUs;}uint32_t millis(){return nowUs/1000;}void publishFrame(){}
'''+function('src/m4/rear_tof_m4.cpp','void pollSensor()')+r'''
int main(){
 pollSensor();assert(frame.status==REAR_TOF_RPC_RUNNING&&frame.raw_distance_mm==200);
 nowUs+=3000;rearSensor.readError=1;pollSensor();
 assert(frame.status==REAR_TOF_RPC_BUS_FAILED&&frame.raw_distance_mm==-1);
 nowUs+=3000;rearSensor.readError=0;pollSensor();assert(frame.status==REAR_TOF_RPC_RUNNING);
 nowUs+=3000;rearSensor.clearError=1;pollSensor();assert(frame.status==REAR_TOF_RPC_BUS_FAILED);
 rearSensor.clearError=0;rearSensor.nextDistance=500;nowUs+=300000;pollSensor();
 assert(frame.status==REAR_TOF_RPC_RUNNING&&frame.filtered_distance_mm==500);
 std::cout<<"PASS actual M4 ToF: read failure is not erased by successful restart, recovery preserved\n";
}
'''
check('rear',rear)
side = common+r'''
constexpr int VL53L4CX_ERROR_NONE=0,VL53L4CX_RANGESTATUS_RANGE_VALID=0,VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE=1;
constexpr float TOF_OUT_OF_RANGE_MM=9999,TOF_MAX_RELIABLE_DISTANCE_MM=600,TOF_MAX_LONG_DISTANCE_MM=1200,TOF_MAX_DELTA_MM=50;
constexpr int TOF_DIAGNOSTIC_MAX_OBJECTS=4;
enum TofSensor {TOF_LEFT,TOF_RIGHT,TOF_REAR,TOF_COUNT};
struct Candidate {uint8_t RangeStatus=0;int16_t RangeMilliMeter=200;uint32_t SignalRateRtnMegaCps=65536,SigmaMilliMeter=65536;};
struct VL53L4CX_MultiRangingData_t {uint8_t NumberOfObjectsFound=1;Candidate RangeData[4];};
struct VL53L4CX {int readyError=0,readError=0,clearError=0;bool ready=true;
 VL53L4CX_MultiRangingData_t data;
 int VL53L4CX_GetMeasurementDataReady(uint8_t*r){*r=ready;return readyError;}
 int VL53L4CX_GetMultiRangingData(VL53L4CX_MultiRangingData_t*d){*d=data;return readError;}
 int VL53L4CX_ClearInterruptAndStartMeasurement(){return clearError;}} sensor_left,sensor_right;
struct TofObjectDiagnostic {int16_t distance_mm=0;float signal_mcps=0,sigma_mm=0;uint8_t range_status=0;bool hardware_valid=false,filter_accepted=false;};
struct TofDiagnosticSnapshot {uint32_t sequence=0,sampled_ms=0,timing_budget_us=30000;
 uint8_t reported_object_count=0,stored_object_count=0;int8_t selected_object_index=-1;
 float filtered_distance_mm=-1,selected_raw_distance_mm=-1,selected_signal_mcps=-1,selected_sigma_mm=-1;
 TofObjectDiagnostic objects[4];} tof_diagnostics[TOF_COUNT];
bool tof_transport_valid[TOF_COUNT]={false,false,false},nav_long_range_active=false;
float tof_distances[TOF_COUNT]={-1,-1,-1},tof_raw_distances[TOF_COUNT]={-1,-1,-1};
float tof_signal_rates[TOF_COUNT]={-1,-1,-1},tof_sigmas[TOF_COUNT]={-1,-1,-1};
uint32_t nowMs=100;uint32_t millis(){return nowMs;}
'''+function('src/sensors.cpp','static bool tof_value_fresh(')+'\n'+function('src/sensors.cpp','static void read_single_tof(')+'\n'+function('src/sensors.cpp','float get_tof_distance(')+'\n'+function('src/sensors.cpp','float get_tof_raw_distance(')+r'''
void tick(){nowMs+=30;read_single_tof(sensor_left,tof_distances[TOF_LEFT]);}
int main(){
 tick();assert(get_tof_distance(TOF_LEFT)==200&&get_tof_raw_distance(TOF_LEFT)==200);
 sensor_left.data.NumberOfObjectsFound=0;tick();
 assert(get_tof_distance(TOF_LEFT)==9999&&get_tof_raw_distance(TOF_LEFT)==-1);
 sensor_left.readyError=1;tick();assert(get_tof_distance(TOF_LEFT)==-1);
 sensor_left.readyError=0;sensor_left.data.NumberOfObjectsFound=1;tick();assert(get_tof_distance(TOF_LEFT)==200);
 sensor_left.readError=1;tick();assert(get_tof_distance(TOF_LEFT)==-1&&get_tof_raw_distance(TOF_LEFT)==-1);
 sensor_left.readError=0;sensor_left.clearError=1;tick();assert(get_tof_distance(TOF_LEFT)==-1);
 sensor_left.clearError=0;tick();assert(get_tof_distance(TOF_LEFT)==200);
 sensor_left.ready=false;nowMs+=300;tick();assert(get_tof_distance(TOF_LEFT)==-1);
 sensor_left.ready=true;sensor_left.data.RangeData[0].RangeMilliMeter=400;tick();
 assert(get_tof_distance(TOF_LEFT)==400); // No stale slew baseline after the gap.
 sensor_left.data.RangeData[0].RangeMilliMeter=200;tick();
 assert(get_tof_distance(TOF_LEFT)==350&&get_tof_raw_distance(TOF_LEFT)==200);
 std::cout<<"PASS actual side ToF: empty frame clears old raw object, bus/read/restart errors, fresh recovery and stalled stream\n";
}
'''
check('side',side)
seat = common+r'''
enum {MODE_NONE,MODE_OBSTACLE_SEAT_TEST};int current_mode=MODE_OBSTACLE_SEAT_TEST,accepted=0;
const char *mode_name(int){return "SEAT";}
void obstacle_seat_test_show(){}void obstacle_seat_test_clear(){}
bool obstacle_seat_test_expect(uint8_t,uint8_t,char,float){++accepted;return true;}
'''+function('src/serial_handler.cpp','static bool handle_seat_command(')+r'''
int main(){
 assert(handle_seat_command("seat expect 0 1 L 500"));assert(accepted==1);
 assert(handle_seat_command("seat expect\t3 2 R 1000"));assert(accepted==2);
 for(const char *s:{"seat expect 256 1 L 500","seat expect -1 1 L 500",
   "seat expect 0 256 L 500","seat expect 0 1 L 500 suffix",
   "seat expect 999999999999999999999999999 1 L 500"})
  assert(handle_seat_command(s));
 assert(accepted==2);
 std::cout<<"PASS actual seat command: compatible whitespace and valid limits, no wrapped indices/overflow/trailing data\n";
}
'''
check('seat-command',seat)

parking_pause = common+r'''
uint32_t nowMs=0,stateStartMs=100,sensorHoldStartMs=0;
uint32_t millis(){return nowMs;} bool sensorHoldPending=false,active=true;
uint8_t sensorHoldTraceCount=0;bool final_parking_active(){return active;}
void traceParking(bool){}
'''+function('src/final_parking.cpp','void final_parking_sensor_hold()')+'\n'+function('src/final_parking.cpp','void final_parking_sensor_resume()')+r'''
int main(){
 nowMs=500;final_parking_sensor_hold();
 nowMs=700;final_parking_sensor_hold();nowMs=1500;final_parking_sensor_resume();
 assert(stateStartMs==1100&&!sensorHoldPending);
 final_parking_sensor_resume();assert(stateStartMs==1100);
 sensorHoldTraceCount=6;nowMs=1600;final_parking_sensor_hold();
 nowMs=2600;final_parking_sensor_resume();assert(stateStartMs==2100);
 active=false;final_parking_sensor_hold();assert(!sensorHoldPending);
 nowMs=0xfffffff0U;active=true;final_parking_sensor_hold();
 nowMs=16;final_parking_sensor_resume();assert(stateStartMs==2132);
 std::cout<<"PASS parking sensor pause: timeout preserved, repeated calls/log cap/wrap\n";
}
'''
check('parking-sensor-pause',parking_pause)

print('ALL runtime safety host regressions PASS')
