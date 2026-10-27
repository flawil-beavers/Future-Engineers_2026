"""Compare the shared motor ramp with the pre-review source at normal timing.

The historical implementation is frozen by commit, not copied/reimplemented.
Time-gap/restart behavior is tested separately by review_runtime_edges.
"""
import shutil
import subprocess
from check_connector_servo_resume import ROOT, block


def main():
    dest=ROOT/'local_workspace/drive-profile-review'
    dest.mkdir(parents=True,exist_ok=True)
    old=subprocess.check_output(['git','show','861ed8e:src/motor_control.cpp'],text=True)
    old=block(old,old.index('void drive_loop()')).replace('void drive_loop()', 'void historicalDriveLoop()')
    new=(ROOT/'src/motor_control.cpp').read_text()
    fixture=r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
constexpr int A0=0,A1=1,A2=2;constexpr float PI=3.141592653589793f;
#include "drive_profile.h"
enum{DC_DISABLED,DC_ENABLED};int dc_state=DC_ENABLED,target_speed=0,set_degree=0;
float last_loop_time=.020f,current_speed=0,current_acceleration=0;
float active_acceleration_limit=0,acc=DEFAULT_ACCELERATION,target_distance=0;
float current_distance=0,measured_speed=0,commanded_acceleration=0;
float pid_integral=0,accel_pid_integral=0,last_error=0;
bool drive_profile_restart_pending=false,speed_measurement_ready=false;
uint32_t cruise_candidate_start_us=0;
void steer(int){}void service_motor_output(){}void pid_speed(){}
float constrain(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
'''+old+'\n'+block(new,new.index('void drive_loop()'))+r'''
int main(){int checks=0;float worst=0;
 for(float dt:{.001f,.010f,.020f,.050f}){
  current_speed=current_acceleration=target_distance=0;last_loop_time=dt;
  const int targets[]={80,260,520,180,0,-80,-260,0,175,300};
  for(int tick=0;tick<10000;++tick){target_speed=targets[(tick/227)%10];
   const float limits[]={0,60,120,500,1000};acc=limits[tick/2000];
   const float v=current_speed,a=current_acceleration,x=target_distance,l=active_acceleration_limit;
   historicalDriveLoop();
   const float oldV=current_speed,oldA=current_acceleration,oldX=target_distance,oldL=active_acceleration_limit;
   current_speed=v;current_acceleration=a;target_distance=x;active_acceleration_limit=l;
   drive_loop();
   worst=fmaxf(worst,fabsf(current_speed-oldV));
   assert(fabsf(current_speed-oldV)<.001f && fabsf(current_acceleration-oldA)<.001f);
   assert(fabsf(target_distance-oldX)<.001f && fabsf(active_acceleration_limit-oldL)<.001f);
   ++checks;
  }
 }
 std::cout<<"PASS "<<checks<<" historical ramp comparisons; peak speed difference "<<worst<<" mm/s\n";
}
'''
    cpp,exe=dest/'check.cpp',dest/'check.exe';cpp.write_text(fixture)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),str(cpp),'-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],capture_output=True,text=True)
    (dest/'output.txt').write_text(result.stdout+result.stderr)
    print(result.stdout,end='');assert result.returncode==0,result.stderr


if __name__=='__main__':main()
