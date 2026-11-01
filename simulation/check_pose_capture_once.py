"""One visualization run: cached poses, bounded memory, footer, one-shot."""
import shutil
import subprocess
from check_run_telemetry import ROOT, main as prepare
from visualize_robot_run import dense_pose_segments


def main():
    prepare()
    folder=ROOT/'local_workspace/run-telemetry-regression'
    source=folder/'pose_once.cpp'
    source.write_text(r'''
#include "run_telemetry.h"
#include "logger.h"
#include "position_estimator.h"
#include "sensors.h"
#include <cassert>
#include <iostream>
Logger robot_logger;uint32_t now=1000;unsigned reads=0;
uint32_t millis(){return now;}
PositionEstimate get_position_struct(){++reads;return {100,-1200,90};}
unsigned count(const std::string &s){unsigned n=0;size_t at=0;while((at=robot_logger.text.find(s,at))!=std::string::npos){++n;at+=s.size();}return n;}
int main(int argc,char**){
 const bool chatter=argc>1;
 run_telemetry_start();assert(run_telemetry_pose_capture_active());
 now+=199;run_telemetry_tick();assert(reads==0);
 ++now;run_telemetry_tick();assert(reads==1);
 fakeTof[TOF_LEFT]=123;fakeTof[TOF_RIGHT]=456;fakeTof[TOF_REAR]=-1;
 run_telemetry_motion("drive");run_telemetry_motion("drive");
 run_telemetry_obstacle(7,false,-1100,-500);
 run_telemetry_pose_change("rebase",0,0,0,100,-1200,90);
 std::string diagnostics(55*1024,'x');
 robot_logger.write(reinterpret_cast<const uint8_t*>(diagnostics.data()),diagnostics.size());
 for(unsigned i=0;i<1500;++i){
  if(chatter){for(int j=0;j<40;++j){now+=5;
   run_telemetry_phase("exit_11",0);run_telemetry_phase("scan_2",1);run_telemetry_tick();}}
  else now+=200;
  if(i%500==0)run_telemetry_phase("driving",i/500+1);
  run_telemetry_pose_change("correction",0,0,0,.1f,.1f,0);
  run_telemetry_tick();
 }
 run_telemetry_finish("completed","test");
 assert(!run_telemetry_pose_capture_active());
 assert(count("[RUN_POSE] v=2")>850);
 assert(count("[RUN_SYNC]")==1&&count("[RUN_OBSTACLE]")==1);
 assert(robot_logger.text.find("tof=123,456,-1")!=std::string::npos);
 assert(robot_logger.text.find("seat=7 color=GREEN")!=std::string::npos);
 assert(robot_logger.text.find("space=F")!=std::string::npos);
 assert(robot_logger.text.find("[RUN_END]")!=std::string::npos);
 assert(chatter || robot_logger.text.find("[RUN_TRUNCATED]")==std::string::npos);
 assert(reads<1600); // Phase chatter cannot bypass200/500ms pose cadence.
 assert(robot_logger.text.find("t=301200")!=std::string::npos); // Final pose survives.
 assert(robot_logger.text.size()<192*1024-768);
 unsigned saved=reads,poses=count("[RUN_POSE]"),captures=count("[RUN_CAPTURE]");
 run_telemetry_start();assert(!run_telemetry_pose_capture_active());
 now+=500;run_telemetry_tick();run_telemetry_finish("completed","second");
 assert(reads==saved&&count("[RUN_POSE]")==poses&&count("[RUN_CAPTURE]")==captures);
 std::cout<<"POSE ONCE PASS:300s +55KiB diagnostics, >850 cached poses/ToF, obstacle/sync timestamps, footer, next run inactive\n";
}
''')
    exe=folder/'pose_once.exe'
    subprocess.run([shutil.which('g++'),'-std=c++17','-DARDUINO','-DRUN_TELEMETRY_POSE_ONCE=1',
        '-I'+str(folder),'-I'+str(ROOT/'include'),str(ROOT/'src/run_telemetry.cpp'),str(source),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
    subprocess.run([str(exe),'--phase-chatter'],check=True)
    source=(ROOT/'src/obstacle.cpp').read_text()
    assert 'oc_parking_exit_state != PARKING_EXIT_DONE)\n        run_telemetry_phase' in source
    text='\n'.join([
        '[RUN_POSE] v=2 t=100 lap=1 frame=0 space=L pose=1,2,3',
        '[RUN_POSE] v=2 t=300 lap=1 frame=1 space=F pose=10,20,30',
        '[RUN_POSE] v=2 t=500 lap=1 frame=1 space=F pose=11,21,31',
        '[RUN_POSE] v=2 t=700 lap=1 frame=2 space=F pose=50,60,30',
        '[RUN_POSE] v=2 t=900 lap=2 frame=2 space=F pose=60,60,30'])
    groups=dense_pose_segments(text)
    assert list(map(len,groups[1]))==[2,1] and len(groups[2])==1
    print('POSE VISUALIZER PASS: local frame omitted, correction/lap breaks preserved')


if __name__=='__main__':
    main()
