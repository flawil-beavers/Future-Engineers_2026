"""Actual parking-state recovery in all seven entry segments and both turns.

Includes a negative control removing only the segment steering restoration.
Ideal stopped actuator adapter; this does not measure physical servo motion.
"""
import shutil
import subprocess

from check_final_parking_sequence import ROOT, fixture_source


def main():
    fixture = fixture_source()
    driver = r'''
int main(){int checks=0;
 for(int dir:{-1,1})for(int segment=0;segment<7;++segment){
  routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  parking_module::final_parking_reset();parking_module::routeTurnSign=dir;
  parking_module::outerSensor=dir>0?TOF_RIGHT:TOF_LEFT;
  parking_module::scanPhase=5;parking_module::measuredGapMm=247.5f;
  using namespace parking_module;
  localToField(segment?SEGMENT_LOCAL_X_MM[segment-1]:OBSTACLE_FINAL_PARKING_CAPTURE_LOCAL_X_MM,
    segment?SEGMENT_LOCAL_Y_MM[segment-1]:OBSTACLE_FINAL_PARKING_CAPTURE_LOCAL_Y_MM,
    segment?SEGMENT_LOCAL_HEADING_DEG[segment-1]:0,
    hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
  truePose=hostPose;hostTime=1000;encoder=0;stateStartDistance=0;stateStartMs=hostTime;
  state=FP_SEGMENT_DRIVE;segmentIndex=segment;
  servo_disabled=false;set_steering(PARKING_SEGMENTS[segment].steering*dir);
  hostGyroHealthy=false;final_parking_update(dir);++checks;
  if(state!=FP_SEGMENT_DRIVE||!servo_disabled||dc_state!=DC_DISABLED)return 40;
  hostTime+=500;hostGyroHealthy=true;final_parking_sensor_resume();final_parking_update(dir);++checks;
  if(state!=FP_SEGMENT_DRIVE||dc_state!=DC_ENABLED||servo_disabled||
     set_degree!=PARKING_SEGMENTS[segment].steering*dir)return 41;
  // Invalid pose must still abort before a drive commit can restore steering.
  stop(false);hostPose.x_mm=NAN;final_parking_update(dir);++checks;
  if(!final_parking_aborted()||dc_state!=DC_DISABLED||!servo_disabled)return 42;
 }
 std::cout<<"PARK SEGMENT RESUME "<<checks<<" PASS: all segments, both turns, rejected pose stays stopped\n";
}
'''
    dest = ROOT/'local_workspace/parking-segment-resume'
    dest.mkdir(parents=True, exist_ok=True)
    for name in ('fixed', 'old'):
        source = fixture
        if name == 'old':
            needle = '        servo_disabled = false;\n        set_speed(segment.direction'
            assert source.count(needle) == 1
            source = source.replace(needle, '        set_speed(segment.direction')
        cpp, exe = dest/f'{name}.cpp', dest/f'{name}.exe'
        cpp.write_text(source+driver)
        build = subprocess.run([shutil.which('g++'), '-std=c++17', '-O2',
            '-I', str(ROOT/'include'), '-I', str(ROOT/'local_workspace/later-laps'),
            str(cpp), '-o', str(exe)], capture_output=True, text=True)
        (dest/f'{name}_build.txt').write_text(build.stdout+build.stderr)
        if build.returncode:
            print('\n'.join(build.stderr.splitlines()[:30]))
            raise SystemExit(build.returncode)
        run = subprocess.run([str(exe)], capture_output=True, text=True)
        (dest/f'{name}_output.txt').write_text(run.stdout+run.stderr)
        if name == 'fixed':
            assert run.returncode == 0, run.returncode
            print(run.stdout, end='')
        else:
            assert run.returncode == 41, run.returncode
            print('PASS negative control: missing restoration leaves steering disabled on resumed drive')


if __name__ == '__main__':
    main()
