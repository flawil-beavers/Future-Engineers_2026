"""Check physical speed budgets and the real motor ramp on known A/B/C maps.

Run check_later_laps.py first to refresh the production fixture. Uses the real
drive_loop speed ramp with ideal speed tracking; tyre/servo response is unknown.
"""
import json
import shutil
import subprocess

from check_cw_start_planner import ROOT
from check_connector_servo_resume import block


def main():
    dest = ROOT/'local_workspace/later-lap-dynamics'
    dest.mkdir(parents=True, exist_ok=True)
    shared = ROOT/'local_workspace/later-laps'
    fixture = (shared/'check.cpp').read_text().split('int main()')[0]
    # Refresh this function if the matrix was built before a runtime-only edit.
    source = (ROOT/'src/obstacle_path.cpp').read_text()
    tracking = block(source, source.index('float laterLapTrackingSpeed('))
    if 'float laterLapTrackingSpeed(' in fixture:
        old = block(fixture, fixture.index('float laterLapTrackingSpeed('))
        fixture = fixture.replace(old, tracking)
    else:
        fixture += tracking
    fixture += r'''
#include <cassert>
enum {DC_DISABLED,DC_ENABLED};
int dc_state=DC_ENABLED,target_speed=0,set_degree=0;
float last_loop_time=.020f,current_speed=0,current_acceleration=0;
float active_acceleration_limit=0,acc=DEFAULT_ACCELERATION,target_distance=0;
void steer(int){} void service_motor_output(){} void pid_speed(){}
float constrain(float v,float lo,float hi){return fminf(hi,fmaxf(lo,v));}
'''
    motor = (ROOT/'src/motor_control.cpp').read_text()
    fixture += block(motor, motor.index('void drive_loop()'))
    fixture += r'''
void analyticChecks(){
 assert(laterLapCurvatureSpeed(0)==520);
 assert(laterLapCurvatureSpeed(NAN)==0);
 assert(laterLapCurvatureSpeed(INFINITY)==0);
 // Independent SI-unit computation: R=.200m, a=.8m/s^2.
 const float expected=1000*sqrtf(.8f*.200f/1.15f)/1.10f;
 assert(fabsf(laterLapCurvatureSpeed(.005f)-expected)<.01f);
 assert(fabsf(laterLapCurvatureSpeed(-.005f)-expected)<.01f);
 assert(laterLapTrackingSpeed(140,.005f,30)<=140);
 assert(laterLapTrackingSpeed(520,NAN,0)==0);
 assert(laterLapTrackingSpeed(NAN,0,0)==0);
 // A straight request must still respect an actually tight servo command.
 assert(laterLapTrackingSpeed(520,0,42)<400);
 // Constant-radius circle: no artificial braking at the cyclic seam.
 pathLength=40;
 for(int i=0;i<pathLength;++i){float h=2*PI*i/pathLength;
  optimizedPath[i].x=200*cosf(h);optimizedPath[i].y=200*sinf(h);
  optimizedPath[i].speedMmS=260;}
 planLaterLapSpeeds(optimizedPath);
 for(int i=0;i<pathLength;++i)assert(fabsf(laterLapSpeedLimits[i]-expected)<.3f);
 // Duplicate closing waypoint: same circle and same limits, no false stop.
 optimizedPath[40]=optimizedPath[0];pathLength=41;planLaterLapSpeeds(optimizedPath);
 for(int i=0;i<pathLength;++i)assert(fabsf(laterLapSpeedLimits[i]-expected)<.3f);
 std::cout<<"ANALYTIC PASS radius/sign/nonfinite/runtime cap/CAD/cyclic duplicate\n";
}
float elapsed=0,peakLateral=0,peakSpeed=0,peakTrackingRatio=0;
'''
    rollout = block(fixture, fixture.index('bool driveTwoLaps('))
    rollout = rollout.replace('bool driveTwoLaps(', 'bool driveDynamicLaps(')
    rollout = rollout.replace('step<5000', 'step<16000')
    start = rollout.index('  // Worst-case 20 ms travel')
    end = rollout.index('  const float next=heading+curvature*ds;', start)
    rollout = rollout[:start]+r'''
  float command=lapFinishPending ? fminf(route[progressIndex].speedMmS,
      OBSTACLE_LATER_LAP_CORNER_SPEED) : laterLapTrackingSpeed(
      laterLapSpeedLimits[progressIndex],requestedCurvature,static_cast<int>(steering));
  if(speedCap>0)command=fminf(command,speedCap);
  target_speed=static_cast<int>(command);drive_loop();
  elapsed+=last_loop_time;
  peakSpeed=fmaxf(peakSpeed,current_speed);
  peakLateral=fmaxf(peakLateral,current_speed*current_speed*fabsf(curvature));
  peakTrackingRatio=fmaxf(peakTrackingRatio,current_speed/fmaxf(1,command));
  const float ds=current_speed*last_loop_time;
'''+rollout[end:]
    fixture += rollout
    fixture += r'''
int main(int argc,char **argv){if(argc>1)last_loop_time=std::atof(argv[1]);
 analyticChecks();int direction,id=0,colors[24];float cap;int bad=0;
 while(std::cin>>direction>>cap){
  for(int &c:colors)if(!(std::cin>>c))return 3;
  routeTurnSign=direction;resetState();baseline();
  for(int i=0;i<24;++i){seats[i].confirmed=colors[i]!=0;
   seats[i].red=colors[i]==2;seats[i].injected=colors[i]!=0;}
  learned();joinSameColourStraightEnds(livePath,true);
  current_speed=260;current_acceleration=0;elapsed=peakLateral=peakSpeed=peakTrackingRatio=0;
  float wall,pillar;bool ok=driveDynamicLaps(cap,wall,pillar);
  // Even with the modeled ramp lag, keep actual lateral acceleration below
  // the chosen .8m/s^2 budget, and preserve the 30mm rollout pillar floor.
  ok=ok&&peakLateral<=OBSTACLE_LATER_LAP_LATERAL_ACCEL_MMSS&&pillar>=30;
  std::cout<<"DYNAMIC "<<id++<<" "<<ok<<" "<<elapsed<<" "<<peakLateral
   <<" "<<peakSpeed<<" "<<peakTrackingRatio<<" "<<wall<<" "<<pillar<<"\n";
  if(!ok)++bad;
 }return bad?1:0;
}
'''
    cpp, exe = dest/'check.cpp', dest/'check.exe'
    cpp.write_text(fixture)
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'include'),
                    '-I', str(shared), str(cpp), '-o', str(exe)], check=True)
    cases = json.loads((shared/'report.json').read_text())['cases'][-7:]
    inputs = '\n'.join(' '.join(map(str, [c['direction'], c['speed_cap'], *c['colors']]))
                       for c in cases)+'\n'
    failed = False
    for dt in (.010, .020, .050):
        run = subprocess.run([str(exe), str(dt)], input=inputs, text=True, capture_output=True)
        (dest/f'output_{int(dt*1000)}ms.txt').write_text(run.stdout+run.stderr)
        print('Control interval', dt, 's')
        print(run.stdout, end='')
        if run.stderr:
            print(run.stderr)
        failed |= run.returncode != 0
    raise SystemExit(1 if failed else 0)


if __name__ == '__main__':
    main()
