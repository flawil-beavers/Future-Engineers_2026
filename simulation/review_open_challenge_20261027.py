"""Source-backed Open Challenge audit; no upload or robot movement.

Compiles the complete current navigation translation unit with actual actuator
commands and ToF freshness getters. Hardware frames, heading, motion and USB
are simulated. FINDING lines deliberately reproduce unresolved source risks;
exit zero means the audit reproduced its expectations, not physical acceptance.
Generated sources, binaries and output stay in ignored local_workspace.
"""
import ast
from pathlib import Path
import re
import shutil
import subprocess

from check_connector_servo_resume import block

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'local_workspace/open_challenge_review_20261027'


def without_includes(source):
    return re.sub(r'^\s*#(?:include[^\n]*|pragma once)', '', source, flags=re.MULTILINE)


def main(extra_checks=None):
    OUT.mkdir(parents=True, exist_ok=True)
    motor = (ROOT / 'src/motor_control.cpp').read_text()
    sensors = (ROOT / 'src/sensors.cpp').read_text()
    nav = (ROOT / 'src/navigation_controller.cpp').read_text()
    mode = (ROOT / 'src/mode_manager.cpp').read_text()
    open_start = re.search(r'case MODE_OPEN_CHALLENGE:(.*?)break;',
        mode[mode.index('static bool start_mode('):], flags=re.DOTALL).group(1)
    # Reuse simulated electrical outputs; execute the actual four actuator bodies.
    tree = ast.parse((ROOT / 'simulation/check_connector_servo_resume.py').read_text())
    actuator = next(ast.literal_eval(n.value) for n in ast.walk(tree)
        if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and
            t.id == 'fixture' for t in n.targets))
    actuator = actuator[:actuator.index('struct Pose{};')]
    actuator = actuator.replace(
        'constexpr int SERVO_CENTER=80, SERVO_MAX_ANGLE=140, SERVO_MIN_ANGLE=20;', '')
    actuator = actuator.replace('void print(T){}', 'void print(T,int=0){}')
    actuator = actuator.replace('void println(T){}', 'void println(T,int=0){} void println(){}')
    actuator = actuator.replace('} Serial;', '} robot_logger;')
    prefix = r'''
#include <cmath>
#include <cstdint>
#include <cassert>
#include <iostream>
constexpr int A0=0,A1=1,A2=2;
constexpr float PI=3.14159265358979323846f;
#include "config.h"
#include "runtime_safety.h"
template<class T> T constrain(T v,T lo,T hi){return v<lo?lo:(v>hi?hi:v);}
struct TwoWire{};
using VL53L4CX_DistanceModes=int;
'''
    prefix += without_includes((ROOT / 'include/navigation_controller.h').read_text())
    prefix += without_includes((ROOT / 'include/sensors.h').read_text())
    prefix += actuator
    prefix += '\nvoid stop(bool hold=false);\n'
    prefix += '\n'.join(block(motor, motor.index(s)) for s in (
        'void steer(int angle)', 'void set_steering(int angle)',
        'void set_speed(int speed)', 'void stop(bool hold)'))
    prefix += r'''
uint32_t nowMs=0; uint32_t millis(){return nowMs;}
unsigned long current_time=0;
float last_loop_time=.010f,measured_speed=0,angle=0;
float get_angle(){return angle;} float get_distance(){return current_distance;}
TofDiagnosticSnapshot tof_diagnostics[TOF_COUNT]={};
bool tof_transport_valid[TOF_COUNT]={};
float tof_distances[TOF_COUNT]={},tof_raw_distances[TOF_COUNT]={};
float tof_signal_rates[TOF_COUNT]={},tof_sigmas[TOF_COUNT]={};
uint32_t sensors_initial_tof_timing_budget=TOF_TIMING_BUDGET_US;
uint32_t actualBudget=TOF_TIMING_BUDGET_US;
void sensors_set_tof_timing_budget(uint32_t budget){
 actualBudget=budget;
 for(int side=0;side<TOF_SIDE_COUNT;++side)tof_diagnostics[side].timing_budget_us=budget;
}
'''
    # Add save support to the same logger used by all actual source functions.
    prefix = prefix.replace('} robot_logger;',
        'int saves=0; void write_to_usb(){++saves;} } robot_logger;')
    prefix += '\n'.join(block(sensors, sensors.index(s)) for s in (
        'static bool tof_value_fresh(', 'float get_tof_distance(',
        'float get_tof_raw_distance(', 'float get_tof_signal_rate(',
        'float get_tof_sigma(', 'bool get_tof_diagnostic_snapshot('))
    driver = r'''
void frame(int side,float distance){
 auto &d=tof_diagnostics[side]; ++d.sequence;d.sampled_ms=nowMs;
 d.timing_budget_us=actualBudget;tof_transport_valid[side]=true;
 tof_distances[side]=tof_raw_distances[side]=distance;
}
void frames(float left,float right){frame(TOF_LEFT,left);frame(TOF_RIGHT,right);}
void tick(uint32_t dt=10){nowMs+=dt;navigation_update(true);steer(set_degree);}
void reset(){
 stop(false);nowMs=0;angle=0;current_distance=0;current_speed=measured_speed=0;
 actualBudget=TOF_TIMING_BUDGET_US;robot_logger.saves=0;
 for(int i=0;i<TOF_COUNT;++i){tof_diagnostics[i]={};tof_transport_valid[i]=false;}
 navigation_set_speed(300);navigation_set_target_distance(300);
 navigation_set_obstacle_mode(false);navigation_enable();
}
void selectSide(int sign){nav_following_wall=sign>0?SIDE_LEFT:SIDE_RIGHT;nav_turn_angle=90*sign;}
void knownFrames(int sign,float distance){frames(sign>0?distance:300,sign>0?300:distance);}
int main(){
 for(int sign:{-1,1}){
  reset();current_distance=301;frames(300,300);tick();
  assert(actualBudget==300000);
  const uint32_t first=nowMs;
  knownFrames(sign,9999);tick();assert(nav_state==NAV_FOLLOWING);
  for(int i=0;i<20;++i)tick();assert(nav_state==NAV_FOLLOWING);
  // Model two further 300 ms discovery acquisitions, rather than loop ticks.
  nowMs=first+300;knownFrames(sign,9999);tick();assert(nav_state==NAV_FOLLOWING);
  nowMs=first+600;knownFrames(sign,9999);tick();assert(nav_state==NAV_TURNING);
  assert(nav_turn_angle==90*sign&&nav_turn_count==1&&actualBudget==TOF_TIMING_BUDGET_US);
  std::cout<<"FINDING first corner sign="<<sign
   <<" confirmation_ms="<<nowMs-(first+10)<<" additional_travel_at_300=180mm\n";
 }
 std::cout<<"PASS independent frames, confirmed side selection, budget restoration\n";
 for(int sign:{-1,1}){
  reset();selectSide(sign);current_distance=301;
  knownFrames(sign,9999);tick();for(int i=0;i<10;++i)tick();
  assert(nav_state==NAV_FOLLOWING);nowMs+=300;
  tick();assert(nav_corner_gap_samples==0); // stale cached gap is not evidence
  tof_transport_valid[sign>0?TOF_LEFT:TOF_RIGHT]=false;
  tick();assert(nav_corner_gap_samples==0);
  for(int n=0;n<3;++n){knownFrames(sign,9999);tick(30);}
  assert(nav_state==NAV_TURNING&&nav_turn_count==1);
 }
 std::cout<<"PASS stale and transport failure cannot confirm a corner; fresh recovery\n";
 for(int sign:{-1,1}){
  reset();selectSide(sign);
  // Logical twelve-corner mission; heading/distance are injected, not integrated physics.
  for(int corner=1;corner<=12;++corner){
   current_distance=nav_start_distance+701;
   for(int n=0;n<3;++n){knownFrames(sign,9999);tick(30);}
   assert(nav_state==NAV_TURNING&&nav_turn_count==corner);
   tick();assert(set_degree==-25*sign);
   angle=nav_gyro_target+90.1f*sign;tick();
   assert(nav_state==NAV_FOLLOWING&&fabs(nav_gyro_target-90*corner*sign)<.01f);
   angle=nav_gyro_target;
  }
  current_distance=nav_start_distance+501;knownFrames(sign,300);tick();
  assert(nav_state==NAV_STOPPED&&!navigation_is_complete());
  current_speed=measured_speed=300;tick();assert(target_speed==0&&!navigation_is_complete());
  current_speed=measured_speed=0;tick();assert(navigation_is_complete());
  assert(dc_state==DC_DISABLED&&robot_logger.saves==1);tick();assert(robot_logger.saves==1);
 }
 std::cout<<"PASS CW/CCW twelve corners, gyro targets, controlled finish and single save\n";
 for(int state:{int(NAV_FOLLOWING),int(NAV_TURNING)}){
  reset();selectSide(1);nav_state=NavigationState(state);nav_turn_start_angle=angle=0;
  current_distance=100;frames(350,300);stop(false);tick();
  assert(!servo_disabled&&dc_state==DC_ENABLED&&target_speed==300);
 }
 std::cout<<"PASS actual motor/servo commands restore steering after a gyro stop in both Open moving states\n";
 reset();navigation_set_speed(OBSTACLE_CRUISE_SPEED);navigation_disable();
 startOpen();
 assert(target_speed==OBSTACLE_CRUISE_SPEED);
 std::cout<<"FINDING switching from Obstacle retains navigation speed="<<target_speed<<" instead of startup 300\n";
 for(int sign:{-1,1}){
  reset();selectSide(sign);nav_turn_count=12;nav_completed_rounds=3;
  current_distance=501;knownFrames(sign,9999);tick();
  assert(nav_state==NAV_FOLLOWING&&target_speed==300&&!navigation_is_complete());
  for(int n=0;n<2;++n){knownFrames(sign,9999);tick(30);}
  assert(nav_state==NAV_TURNING&&nav_turn_count==13&&!navigation_is_complete());
  reset();selectSide(sign);nav_turn_count=12;nav_completed_rounds=3;
  current_distance=1000;tick(); // all side frames unavailable
  assert(nav_state==NAV_FOLLOWING&&target_speed==300&&!navigation_is_complete());
 }
 std::cout<<"FINDING missing wall at finish permits thirteenth turn instead of stopping (CW/CCW)\n";
 reset();selectSide(1);nav_state=NAV_TURNING;nav_turn_angle=90;
 for(int i=0;i<1000;++i){current_distance+=3;tick();}
 assert(nav_state==NAV_TURNING&&target_speed==300);
 std::cout<<"FINDING fresh but unchanging gyro: still turning after simulated 10s/3000mm; no turn limit\n";
 reset();frames(300,300);tick();assert(actualBudget==300000);
 navigation_disable();assert(actualBudget==300000);
 std::cout<<"FINDING disabling during discovery leaves side sensors at 300000us budget\n";
}
'''
    compiler = shutil.which('g++') or shutil.which('clang++')
    if not compiler:
        raise RuntimeError('Host g++ or clang++ required')

    def run(name, code):
        source, binary = OUT / f'{name}.cpp', OUT / f'{name}.exe'
        source.write_text(code)
        result = subprocess.run([compiler, '-std=c++17', '-O2', '-I', str(ROOT / 'include'),
            str(source), '-o', str(binary)], text=True, capture_output=True)
        (OUT / f'{name}_build.txt').write_text(result.stdout + result.stderr)
        if result.returncode:
            print(result.stderr)
            raise SystemExit(result.returncode)
        result = subprocess.run([str(binary)], text=True, capture_output=True)
        (OUT / f'{name}_output.txt').write_text(result.stdout + result.stderr)
        print(result.stdout + result.stderr, end='')
        if result.returncode:
            raise SystemExit(result.returncode)

    run('open_challenge', prefix + without_includes(nav) +
        '\nvoid startOpen(){' + open_start + '\n}\n' + driver)
    # Frozen parent of the fresh-frame change: demonstrate the timing regression
    # and distinguish existing finish behavior from a newly introduced defect.
    historical = subprocess.check_output(['git', 'show',
        '861ed8e:src/navigation_controller.cpp'], text=True)
    historical_driver = driver[driver.index('void frame('):driver.index('int main(){')]
    historical_driver += r'''
int main(){
 reset();current_distance=301;frames(300,300);tick();
 frames(9999,300);tick(1);const uint32_t first=nowMs;
 tick(1);tick(1);assert(nav_state==NAV_TURNING&&nav_turn_count==1);
 std::cout<<"REFERENCE 861ed8e first corner: cached-frame confirmation_ms="<<nowMs-first<<"\n";
 reset();selectSide(1);nav_turn_count=12;nav_completed_rounds=3;current_distance=501;
 frames(9999,300);tick();tick();tick();
 assert(nav_state==NAV_TURNING&&nav_turn_count==13);
 std::cout<<"REFERENCE 861ed8e missing-wall finish defect already present\n";
}
'''
    run('historical_open', prefix + without_includes(historical) + historical_driver)
    if extra_checks is not None:
        helpers = driver[driver.index('void frame('):driver.index('int main(){')]
        extra_checks(prefix, nav, historical, helpers)


if __name__ == '__main__':
    main()
