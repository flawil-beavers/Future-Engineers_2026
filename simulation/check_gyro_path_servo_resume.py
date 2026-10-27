"""Actual main gyro-hold and motor/path commit regression,with old negative control."""
from pathlib import Path
import ast,shutil,subprocess
from check_connector_servo_resume import block
ROOT=Path(__file__).resolve().parents[1]
tree=ast.parse((ROOT/'simulation/check_connector_servo_resume.py').read_text())
fixture=next(ast.literal_eval(n.value) for n in ast.walk(tree) if isinstance(n,ast.Assign)
             and any(isinstance(t,ast.Name) and t.id=='fixture' for t in n.targets))
motor=(ROOT/'src/motor_control.cpp').read_text()
functions='\n'.join(block(motor,motor.index(s)) for s in (
    'void steer(int angle)','void set_steering(int angle)','void set_speed(int speed)','void stop(bool hold)'))
main=(ROOT/'src/main.cpp').read_text();gyro_hold=block(main,main.index('if (!gyro_is_healthy())'))
path=(ROOT/'src/obstacle_path.cpp').read_text()
end=path.index('    set_steering(static_cast<int>(steering));',path.index('void obstacle_path_update('))
start=path.rfind('    servo_disabled = false;',0,end)
commit=path[start:end+len('    set_steering(static_cast<int>(steering));')]
driver=r'''
bool healthy=true; bool gyro_is_healthy(){return healthy;}
void final_parking_sensor_hold(){} void obstacle_path_sensor_hold(){} void run_telemetry_phase(const char*){}
void run_telemetry_tick(){} struct Logger{void update(){}}robot_logger;
float steering=30;
void iteration(bool reject){
GYRO_HOLD
if(reject){stop(false);return;}
COMMIT
set_speed(260);steer(set_degree);
}
int main(){
 servo_disabled=false;steer(0);dc_state=DC_ENABLED;
 // Reproduce one loop with stale gyro after the logged812--819ms pause.
 healthy=false;iteration(false);
 if(!servo_disabled||dc_state!=DC_DISABLED||target_speed!=0)return 10;
 // Fresh reports permit controller execution again,but only a successful
 // route command may restore the physical steering output.
 healthy=true;iteration(false);
 if(servo_disabled||dc_state!=DC_ENABLED||servo.angle!=SERVO_CENTER+30)return 11;
 steering=-25;iteration(false);
 if(servo.angle!=SERVO_CENTER-25)return 12;
 iteration(true);
 if(!servo_disabled||dc_state!=DC_DISABLED||target_speed!=0)return 13;
 healthy=false;iteration(false);
 if(!servo_disabled||dc_state!=DC_DISABLED)return 14;
 std::cout<<"PASS stale gyro brakes; fresh path restores steering; rejected/invalid path stays stopped\n";
}
'''.replace('GYRO_HOLD',gyro_hold).replace('COMMIT',commit)
dest=ROOT/'local_workspace/fix_525_527';dest.mkdir(parents=True,exist_ok=True)
for name,code in [('fixed',driver),('old',driver.replace('servo_disabled = false;','/* missing restoration */'))]:
    cpp=dest/(name+'_servo.cpp');exe=dest/(name+'_servo.exe')
    cpp.write_text(fixture+functions+code)
    subprocess.run([shutil.which('g++'),'-std=c++17',str(cpp),'-o',str(exe)],check=True)
    run=subprocess.run([str(exe)],text=True,capture_output=True)
    (dest/(name+'_servo_output.txt')).write_text(run.stdout+run.stderr)
    if name=='fixed':print(run.stdout,end='');assert run.returncode==0,run.returncode
    else:assert run.returncode==11,run.returncode;print('PASS removing restoration reproduces motor-on/frozen-servo failure')
