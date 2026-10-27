"""Source-backed regressions for the runtime audit fixes; never upload or drive.

Exercises actual firmware fragments with simulated hardware and faults.
All generated C++ and outputs remain in ignored local_workspace.
"""
import ast
import json
import re
import shutil
import subprocess

from check_connector_servo_resume import ROOT, block

DEST = ROOT/'local_workspace/code_audit_20261027'


def run(name, code):
    cpp, exe = DEST/f'{name}.cpp', DEST/f'{name}.exe'
    cpp.write_text(code)
    build = subprocess.run([shutil.which('g++'), '-std=c++17', '-O2',
        '-I', str(ROOT/'include'), str(cpp), '-o', str(exe)], text=True, capture_output=True)
    (DEST/f'{name}_build.txt').write_text(build.stdout+build.stderr)
    if build.returncode:
        print('\n'.join(build.stderr.splitlines()[:25]));raise SystemExit(build.returncode)
    result = subprocess.run([str(exe)], text=True, capture_output=True, check=True)
    (DEST/f'{name}_output.txt').write_text(result.stdout+result.stderr)
    print(result.stdout, end='')
    return result.stdout.splitlines()


def main():
    DEST.mkdir(parents=True, exist_ok=True)
    motor = (ROOT/'src/motor_control.cpp').read_text()
    path = (ROOT/'src/obstacle_path.cpp').read_text()
    mode = (ROOT/'src/mode_manager.cpp').read_text()
    config = (ROOT/'include/config.h').read_text()
    observations = {}
    # Actual drive profile with the delta time published by loop_updater.
    ramp = r'''
#include <cstdint>
#include <cmath>
#include <iostream>
constexpr int A0=0,A1=1,A2=2;
constexpr float PI=3.14159265358979323846f;
#include "config.h"
#include "drive_profile.h"
enum{DC_DISABLED,DC_ENABLED};int dc_state=DC_ENABLED,target_speed=260,set_degree=0;
float last_loop_time=0,current_speed=0,current_acceleration=0;
float active_acceleration_limit=0,acc=DEFAULT_ACCELERATION,target_distance=0;
bool drive_profile_restart_pending=false,speed_measurement_ready=false;
float current_distance=0,measured_speed=0,commanded_acceleration=0;
float pid_integral=0,accel_pid_integral=0,last_error=0;
uint32_t cruise_candidate_start_us=0;
void steer(int){}void service_motor_output(){}void pid_speed(){}
float constrain(float v,float lo,float hi){return fminf(hi,fmaxf(lo,v));}
''' + block(motor, motor.index('void drive_loop()')) + r'''
int main(){for(float dt:{.020f,.050f,.215f,.819f,1.2f}){
 last_loop_time=dt;current_speed=current_acceleration=target_distance=0;drive_loop();
 if(dt>.100f && (current_speed!=0 || target_distance!=0))return 1;
 std::cout<<"RAMP dt="<<dt<<" profile="<<current_speed<<" virtual_distance="<<target_distance<<"\n";
}
// A delayed active loop accounts for real measured motion without retroactive
// ramp distance. A disabled-to-driving restart starts at rest instead.
 current_distance=500;measured_speed=120;current_speed=260;last_loop_time=.819f;
 drive_loop();if(current_speed!=120||target_distance!=500)return 2;
 drive_profile_restart_pending=true;drive_loop();
 if(current_speed!=0||target_distance!=500)return 3;
 last_loop_time=.020f;drive_loop();if(current_speed<=0||target_distance>501)return 4;
 std::cout<<"PASS delayed active motion, stopped restart, next normal ramp\n";
}
'''
    observations['ramp'] = run('ramp_gap', ramp)
    tree = ast.parse((ROOT/'simulation/check_connector_servo_resume.py').read_text())
    actuator = next(ast.literal_eval(n.value) for n in ast.walk(tree)
        if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id=='fixture' for t in n.targets))
    actuator += '\n'.join(block(motor, motor.index(s)) for s in (
        'void steer(int angle)', 'void set_steering(int angle)',
        'void set_speed(int speed)', 'void stop(bool hold)'))
    start = path.index('        {\n            const int steering=cornerViewStraightSteering(pose,-1);',
                       path.index('void updateCornerView('))
    end = path.index('        return;', path.index('        set_speed(-OBSTACLE_CORNER_VIEW_REVERSE_SPEED_MM_S);', start))
    commit = path[start:end]
    return_start=path.index('        {\n            const int steering=cornerViewStraightSteering(pose,+1);',path.index('void updateCornerView('))
    return_end=path.index('        return;',path.index('        set_speed(60);',return_start))
    extra_reverse=path.index('        set_steering(cornerViewExtraSteering);',path.index('    case CORNER_VIEW_EXTRA_REVERSE:'))
    extra_reverse_end=path.index('    case CORNER_VIEW_EXTRA_OBSERVE:',extra_reverse)
    extra_forward=path.index('        servo_disabled = false;',path.index('    case CORNER_VIEW_EXTRA_FORWARD:'))
    extra_forward_end=path.index('    case CORNER_VIEW_EXTRA_BRAKE:',extra_forward)
    speed = re.search(r'constexpr[^;\n]*\bOBSTACLE_CORNER_VIEW_REVERSE_SPEED_MM_S\b[^;]*;', config).group()
    corner = actuator + '\n' + speed + r'''
bool safeCommand=true;Pose pose;
int cornerViewExtraSteering=-25;
int cornerViewStraightSteering(Pose,int){return 7;}
bool cornerViewSteeredCommandSafe(Pose,int,int){return safeCommand;}
void lockCornerView(const char*){stop(false);}
void acceptedCornerCommit(){
''' + commit + r'''
 steer(set_degree);
}
void acceptedReturn(){
''' + path[return_start:return_end] + r'''
}
void acceptedExtraReverse(){
''' + path[extra_reverse:extra_reverse_end] + r'''
}
void acceptedExtraForward(){
''' + path[extra_forward:extra_forward_end] + r'''
}
int main(){servo_disabled=false;steer(0);stop(false);acceptedCornerCommit();
 std::cout<<"CORNER after_hold motor="<<(dc_state==DC_ENABLED)<<" servo_disabled="<<servo_disabled
   <<" requested="<<set_degree<<" physical="<<servo.angle-SERVO_CENTER<<"\n";
 if(servo_disabled||dc_state!=DC_ENABLED||servo.angle!=SERVO_CENTER+7)return 5;
 stop(false);servo_disabled=false;acceptedCornerCommit();
 std::cout<<"CORNER with_restoration motor="<<(dc_state==DC_ENABLED)<<" servo_disabled="<<servo_disabled
   <<" requested="<<set_degree<<" physical="<<servo.angle-SERVO_CENTER<<"\n";
 stop(false);safeCommand=false;acceptedCornerCommit();
 if(!servo_disabled||dc_state!=DC_DISABLED)return 6;
 safeCommand=true;int index=0;
 for(auto commit:{acceptedCornerCommit,acceptedReturn,acceptedExtraReverse,acceptedExtraForward}){
  servo_disabled=false;steer(0);stop(false);commit();steer(set_degree);
  if(servo_disabled||dc_state!=DC_ENABLED||servo.angle!=SERVO_CENTER+(index<2?7:-25))return 16;
  ++index;
 }
 std::cout<<"PASS all four accepted straight/parallax recovery commits after sensor stop\n";
}
'''
    observations['corner'] = run('corner_resume', corner)
    resume = actuator + r'''
enum RobotMode{MODE_NONE,MODE_OBSTACLE_CHALLENGE,MODE_CAMERA_DISTANCE_CAL};
RobotMode current_mode=MODE_NONE,pending_mode=MODE_OBSTACLE_CHALLENGE;
bool system_enabled=false;int starts=0;bool logger_busy=true;
bool resume_after_save=false;int pending_manual_speed_mm_s=0;
struct Logger{bool is_busy(){return logger_busy;}}robot_logger;
void set_obstacle_ready_led(bool){}
bool start_mode(RobotMode){++starts;return true;}
const char* mode_name(RobotMode){return "obstacle";}
void stop_mode(RobotMode){stop(false);}
void run_telemetry_finish(const char*,const char*){}
void system_disable(){system_enabled=false;stop(false);}
''' + block(motor, motor.index('void system_enable()')) + '\n' + block(mode, mode.index('void mode_resume()')) + '\n' + block(mode, mode.index('void mode_pause()')) + '\nvoid servicePendingStart(){\n' + block(mode,mode.index('    if (resume_after_save) {',mode.index('void mode_update()'))) + '\n}\n' + r'''
int main(){mode_resume();
 if(system_enabled||starts||!resume_after_save||pending_mode!=MODE_OBSTACLE_CHALLENGE)return 7;
 mode_pause();if(resume_after_save||system_enabled)return 8;
 mode_resume();servicePendingStart();if(starts)return 15;
 logger_busy=false;servicePendingStart();servicePendingStart();
 if(!system_enabled||starts!=1||current_mode!=MODE_OBSTACLE_CHALLENGE||resume_after_save)return 9;
 std::cout<<"PASS queued USB start, LOW cancellation, completed save starts exactly once\n";
}
'''
    observations['resume'] = run('save_resume', resume)
    bus = (ROOT/'src/m4/software_i2c.cpp').read_text()
    i2c = r'''
#include <algorithm>
#include <cstdint>
#include <iostream>
constexpr int LOW=0,HIGH=1;int clockCalls=0,failAt=-1,bits=0,stops=0;
void delayMicroseconds(unsigned){}int digitalRead(int){return (0xA5>>(7-(bits++%8)))&1;}
class SoftwareI2C{public:
 static constexpr size_t BUFFER_SIZE=256;int scl_pin_=1,sda_pin_=2;unsigned half_period_us_=5;
 uint8_t rx_buffer_[BUFFER_SIZE]={};size_t rx_index_=0,rx_length_=0;
 void release(int){}void driveLow(int){}bool startCondition(){return true;}
 bool writeByte(uint8_t){return true;}void stopCondition(){++stops;}
 bool waitSclHigh(){return ++clockCalls!=failAt;}
 bool readByte(bool,uint8_t&);size_t requestFrom(uint8_t,size_t,bool);int available();
};
''' + '\n'.join(block(bus, bus.index(s)) for s in (
        'bool SoftwareI2C::readByte(', 'size_t SoftwareI2C::requestFrom(uint8_t address, size_t length, bool stop_bit)',
        'int SoftwareI2C::available()')) + r'''
int main(){SoftwareI2C bus;
 for(failAt=1;failAt<=18;++failAt){clockCalls=bits=0;stops=0;
  if(bus.requestFrom(0x29,2,true)!=0||bus.available()!=0||stops!=1)return 10;
 }
 failAt=-1;clockCalls=bits=0;
 if(bus.requestFrom(0x29,2,true)!=2||bus.available()!=2||bus.rx_buffer_[0]!=0xA5||bus.rx_buffer_[1]!=0xA5)return 11;
 std::cout<<"PASS all 16 data-bit and 2 ACK timeouts discard transaction; next request recovers\n";
}
'''
    observations['i2c'] = run('i2c_timeout', i2c)
    logger = (ROOT/'src/logger.cpp').read_text()
    probe = block(logger, logger.index('if (session_file_num < 1)'))
    finder = r'''
#include <cstdint>
#include <cstdio>
#include <iostream>
int probes=0,existing=534,session_file_num=-1,next_probe_number=1;char session_filepath[64];
enum{DC_DISABLED,DC_ENABLED};int dc_state=DC_DISABLED;void update_terminal(){}
FILE *fakeOpen(const char*,const char*){return ++probes<=existing?reinterpret_cast<FILE*>(uintptr_t(1)):nullptr;}
int fakeClose(FILE*){return 0;}
#define fopen fakeOpen
#define fclose fakeClose
void oneLoggerUpdate(){
''' + logger[logger.index('    update_terminal();',logger.index('void USBLogger::update()')):logger.index('    const unsigned long now',logger.index('void USBLogger::update()'))] + probe + r'''
}
int main(){dc_state=DC_ENABLED;oneLoggerUpdate();if(probes)return 12;
 dc_state=DC_DISABLED;int updates=0;
 while(session_file_num<1){int before=probes;oneLoggerUpdate();++updates;
  if(probes-before>4||updates>200)return 13;
 }
 if(session_file_num!=535||probes!=535)return 14;
 std::cout<<"PASS stopped incremental file search: "<<updates<<" updates, "<<probes<<" probes; moving drive does no file I/O\n";
}
'''
    observations['logger'] = run('log_file_probe', finder)
    retention = r'''
#include <cstdint>
#include <cstring>
#include <cassert>
#include <iostream>
constexpr int PA_15=15,LOW=0;void digitalWrite(int,int){}
class USBLogger{public:
 bool busy=false,buffer_overflow=false;size_t buffer_head=12,tail_reserve=0,flush_target_length=0;
 int session_file_num=12,next_probe_number=13;char log_buffer[64]="saved bytes",session_filepath[64]="log_12";
 bool is_busy(){return busy;}void clear();
};
''' + block(logger,logger.index('void USBLogger::clear()')) + r'''
int main(){USBLogger l;l.busy=true;l.clear();assert(l.buffer_head==12);
 l.busy=false;l.flush_target_length=12;l.clear();
 assert(l.buffer_head==12&&std::strcmp(l.log_buffer,"saved bytes")==0);
 l.flush_target_length=0;l.clear();assert(l.buffer_head==0&&l.session_file_num==-1&&l.next_probe_number==1);
 std::cout<<"PASS pending/failed USB snapshot survives clear; successful snapshot permits new run\n";
}
'''
    observations['retention'] = run('log_retention', retention)
    state = r'''
#include <cstdint>
#include <cassert>
#include <iostream>
#include "runtime_safety.h"
uint32_t now=0;uint32_t millis(){return now;}
enum{CORNER_VIEW_IDLE,CORNER_VIEW_REVERSE};int cornerViewPhase=CORNER_VIEW_REVERSE;
bool cornerViewSensorPaused=false;uint32_t cornerViewSensorPauseMs=0,cornerViewPhaseMs=100;
''' + '\n'.join(block(path,path.index(s)) for s in ('void obstacle_path_sensor_hold()', 'void obstacle_path_sensor_resume()')) + r'''
int main(){RuntimeSafety::FreshSampleCounter c;
 c.update(10,true);for(int i=0;i<100;++i)c.update(10,true);assert(c.count==1);
 c.update(11,true);c.update(12,true);assert(c.count==3);
 c.update(12,false);c.update(12,true);assert(c.count==0);
 c.update(13,true);assert(c.count==1);c.clearCount();c.update(13,true);assert(c.count==0);
 c.update(0xffffffffU,true);c.update(0,true);assert(c.count==2);
 now=200;obstacle_path_sensor_hold();now=300;obstacle_path_sensor_hold();
 now=1700;obstacle_path_sensor_resume();assert(cornerViewPhaseMs==1600);
 obstacle_path_sensor_resume();assert(cornerViewPhaseMs==1600);
 now=0xfffffff0U;obstacle_path_sensor_hold();now=16;obstacle_path_sensor_resume();
 assert(cornerViewPhaseMs==1632);
 std::cout<<"PASS distinct frame confirmation, stale/reset/wrap, recovery timer pause/wrap\n";
}
'''
    observations['state'] = run('state_timing', state)
    (DEST/'runtime_edges.json').write_text(json.dumps(observations, indent=2)+'\n')


if __name__ == '__main__':
    main()
