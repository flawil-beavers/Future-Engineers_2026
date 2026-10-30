"""Exercise real parking transitions with a still-rolling or invalid speed.

Optional --reference-fixture reproduces the earlier time-only transition.
Generated source/executables remain under local_workspace.
"""
import argparse
from pathlib import Path
import shutil
import subprocess

from check_cw_start_planner import ROOT
from check_connector_servo_resume import block
from check_final_parking_sequence import fixture_source


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference-fixture',type=Path)
    args=parser.parse_args()
    fixture=args.reference_fixture.read_text() if args.reference_fixture else fixture_source()
    # Match real motor semantics: HOLD retains the driving profile;
    # disabling clears that profile, and a new target does not teleport speed.
    old_stop=block(fixture,fixture.index('void stop(bool hold)'))
    fixture=fixture.replace(old_stop,'''void stop(bool hold){
 dc_state=hold?DC_HOLDING:DC_DISABLED;command=0;
 if(!hold)current_speed=0;servo_disabled=true;}''')
    old_set=block(fixture,fixture.index('void set_speed(int speed)'))
    fixture=fixture.replace(old_set,'''void set_speed(int speed){
 command=speed;dc_state=DC_ENABLED;}''')
    if 'bool parkingBrakeReady(' in fixture:
        fixture+='\n#define PARKING_BRAKE_GATE_PRESENT\n'
    fixture+=r'''
int main(){int total=0,bad=0;
 for(int dir:{-1,1})for(int phase:{parking_module::FP_IDLE,parking_module::FP_APPROACH_BRAKE,
     parking_module::FP_SCAN_BRAKE,parking_module::FP_CAPTURE_BRAKE,parking_module::FP_SEGMENT_BRAKE,
     parking_module::FP_SCAN_ALIGN_BRAKE,parking_module::FP_CAPTURE_ALIGN_BRAKE,
     parking_module::FP_REPLAN_BRAKE})
 for(float speed:{20.f,-20.f,NAN}){
  routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  learned();buildOptimizedPath();
  parking_module::final_parking_reset();parking_module::routeTurnSign=dir;
  parking_module::outerSensor=dir>0?TOF_RIGHT:TOF_LEFT;
  parking_module::state=static_cast<parking_module::FinalParkingState>(phase);
  hostPose=truePose=PositionEstimate{-100,-1000,dir>0?0.f:180.f};
  hostTime=300;parking_module::stateStartMs=0;hostGyroHealthy=true;
  dc_state=DC_HOLDING;command=0;current_speed=measured_speed=speed;
  robot_logger.lines.str("");robot_logger.lines.clear();robot_logger.writes=0;
  parking_module::final_parking_update(dir);++total;
  bool held=parking_module::state==phase&&!parking_module::final_parking_aborted()&&
      !parking_module::final_parking_complete()&&dc_state!=DC_ENABLED;
  // A rolling chassis must not be used as a stationary wall reference.
  if(phase==parking_module::FP_APPROACH_BRAKE)held&=parking_module::initialWallFrames==0;
  if(!held){++bad;std::cout<<"BRAKE FAIL phase "<<phase<<" speed "<<speed<<"\n";}
  // Keep the same movement after a bounded wait: abort instead of reversing
  // or waiting forever if standstill cannot be established.
  hostTime+=3000;parking_module::final_parking_update(dir);++total;
  if(!parking_module::final_parking_aborted()||dc_state==DC_ENABLED||
     robot_logger.lines.str().find("brake_not_settled")==std::string::npos){
   ++bad;std::cout<<"BRAKE TIMEOUT FAIL phase "<<phase<<"\n";}
 }
#ifdef PARKING_BRAKE_GATE_PRESENT
 // A settled handover must clear the old 520mm/s lap profile before parking.
 for(int dir:{-1,1}){
  routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  learned();buildOptimizedPath();parking_module::final_parking_reset();
  hostPose=truePose=PositionEstimate{-100,-1000,dir>0?0.f:180.f};
  hostTime=300;dc_state=DC_HOLDING;current_speed=520;measured_speed=0;
  parking_module::final_parking_update(dir);++total;
  if(dc_state!=DC_ENABLED||current_speed!=0||
      command!=dir*OBSTACLE_FINAL_PARKING_APPROACH_SPEED){++bad;
   std::cout<<"BRAKE stale profile handover FAIL "<<dir<<"\n";}
 }
 // Settled or subsequently recovered measurements release the brake gate.
 // The nonzero current_speed profile is deliberately irrelevant in HOLD.
 for(float speed:{0.f,4.f,-4.f}){
  parking_module::final_parking_reset();parking_module::state=parking_module::FP_SCAN_BRAKE;
  parking_module::stateStartMs=0;hostTime=300;current_speed=520;measured_speed=speed;
  if(!parking_module::parkingBrakeReady(300))++bad;++total;
 }
 parking_module::stateStartMs=0;hostTime=300;measured_speed=20;
 if(parking_module::parkingBrakeReady(300))++bad;
 hostTime=400;measured_speed=0;
 if(!parking_module::parkingBrakeReady(300))++bad;++total;
#endif
 std::cout<<"BRAKING "<<total<<" FAILED "<<bad<<"\n";return bad?1:0;
}
'''
    dest=ROOT/'local_workspace/final-parking-braking'
    dest.mkdir(parents=True,exist_ok=True)
    mode='reference' if args.reference_fixture else 'current'
    cpp,exe=dest/f'{mode}.cpp',dest/f'{mode}.exe'
    cpp.write_text(fixture)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),
        '-I',str(ROOT/'local_workspace/later-laps'),str(cpp),'-o',str(exe)],check=True)
    run=subprocess.run([str(exe)],capture_output=True,text=True,timeout=60)
    (dest/f'{mode}_output.txt').write_text(run.stdout+run.stderr)
    print(run.stdout,end='')
    raise SystemExit(run.returncode)


if __name__=='__main__':main()
