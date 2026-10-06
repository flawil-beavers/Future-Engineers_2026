"""Host regression of the actual connector-update and motor/servo C++ code.

Arduino I/O and geometric preflight decisions are stubbed. The state transition,
stop(), set_speed(), set_steering(), and steer() bodies come from firmware source.
The test checks successful retain/replan and rejected replan without a robot.
"""
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def block(text, start):
    first = text.index('{', start)
    depth = 1
    end = first + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


def main():
    motor = (ROOT / 'src/motor_control.cpp').read_text()
    path = (ROOT / 'src/obstacle_path.cpp').read_text()
    functions = '\n'.join(block(motor, motor.index(signature)) for signature in (
        'void steer(int angle)', 'void set_steering(int angle)',
        'void set_speed(int speed)', 'void stop(bool hold)'))
    update_start = path.index('if (parkingEntryConnectorReplanPending)',
                              path.index('void obstacle_path_update'))
    update = block(path, update_start)
    fixture = r'''
#include <cassert>
#include <cstdint>
#include <iostream>
struct FakeServo { int angle=0, writes=0; void write(int a){angle=a; ++writes;} } servo;
struct FakeSerial { template<class T> void print(T){} template<class T> void println(T){} } Serial;
enum {DC_DISABLED, DC_ENABLED, DC_HOLDING, DRIVE_CRUISING,
      DRIVE_DECELERATING, DRIVE_ACCELERATING};
constexpr int SERVO_CENTER=80, SERVO_MAX_ANGLE=140, SERVO_MIN_ANGLE=20;
bool servo_disabled=false, low_speed_load_compensation_logged=false;
int set_degree=0, dc_state=DC_ENABLED, target_speed=80, last_applied_drive_phase=0,
    drive_control_phase=0;
float last_speed=0,current_speed=0,pid_integral=0,accel_pid_integral=0,
      hold_pid_integral=0,last_error=0,current_acceleration=0,commanded_acceleration=0,
      target_distance=0,current_distance=0;
unsigned cruise_candidate_start_us=0;
void set_dc(int){}
struct Pose{}; struct PathPoint{}; PathPoint points[1]; const PathPoint *path=points;
Pose get_position_struct(){return {};}
float get_distance(){return 0;}
bool retain_ok=false, replan_ok=false;
bool retainParkingConnectorForFarGreen(Pose,const PathPoint*){return retain_ok;}
bool buildParkingEntryConnector(Pose,const PathPoint*,unsigned short &i){i=1; return replan_ok;}
bool parkingEntryConnectorReplanPending=true,parkingEntryFarGreenFollowup=true,
     parkingEntryConnectorActive=true,parkingEntryTestHold=false;
int parkingEntryConnectorChangedSeat=4,parkingEntryConnectorMergeIndex=0,
    parkingEntryConnectorProgress=0,progressIndex=0;
float parkingEntryConnectorStartEncoderDistance=0;
'''
    driver = r'''
void update_fixture(){
UPDATE_BODY
set_steering(25); set_speed(175); steer(set_degree);
}
int main(){
for(int scenario=0; scenario<3; ++scenario){
    servo_disabled=false; steer(0); steer(-12);
    dc_state=DC_ENABLED; set_degree=-12; parkingEntryConnectorReplanPending=true;
    parkingEntryConnectorActive=true; parkingEntryTestHold=false;
    retain_ok=scenario==0; replan_ok=scenario==1;
    update_fixture();
    if(scenario<2){
        if(servo_disabled || dc_state!=DC_ENABLED) return 11;
        if(servo.angle!=SERVO_CENTER+25) return 12;
    } else {
        if(!servo_disabled || dc_state!=DC_DISABLED || !parkingEntryTestHold) return 13;
        if(servo.angle!=SERVO_CENTER-12) return 14;
    }
}
std::cout << "PASS: retained/replanned connectors write new steering; failed replan stays stopped\n";
}
'''.replace('UPDATE_BODY', update)
    output = ROOT / 'local_workspace/connector-servo-regression'
    output.mkdir(parents=True, exist_ok=True)
    source = output / 'fixture.cpp'
    source.write_text(fixture + functions + driver)
    compiler = shutil.which('g++') or shutil.which('clang++')
    if not compiler:
        raise RuntimeError('A host C++ compiler is required')
    binary = output / 'fixture.exe'
    subprocess.run([compiler, '-std=c++17', str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
    # The same fixture must expose the old bug when only the new restoration
    # is removed. This guards against a fixture that passes independently of it.
    source.write_text(fixture + functions + driver.replace(
        'servo_disabled = false;', '/* old connector update: servo left disabled */'))
    old_binary = output / 'old_transition.exe'
    subprocess.run([compiler, '-std=c++17', str(source), '-o', str(old_binary)], check=True)
    old_result = subprocess.run([str(old_binary)])
    if old_result.returncode != 11:
        raise AssertionError(f'Old transition should leave servo disabled, got {old_result.returncode}')
    print('PASS: removing the restoration reproduces the old disabled-servo failure')


if __name__ == '__main__':
    main()
