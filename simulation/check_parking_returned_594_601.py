"""Production parking at the new wall/scan failure anchors.

Sensor values are controlled fresh accepted host frames, not raw sensor replay.
Full physical hardware/camera acceptance still requires ordinary course runs.
"""
import shutil
import subprocess

from check_final_parking_sequence import ROOT, fixture_source


def main():
    source=fixture_source()
    source=source.replace('PositionEstimate hostPose,truePose;',
        'bool forcedSnapshot=false;float forcedRange=0;PositionEstimate hostPose,truePose;')
    source=source.replace('bool get_tof_diagnostic_snapshot(TofSensor sensor,TofDiagnosticSnapshot &s){',
        '''bool get_tof_diagnostic_snapshot(TofSensor sensor,TofDiagnosticSnapshot &s){
 if(forcedSnapshot){s.sequence=hostTime/framePeriod;s.sampled_ms=s.sequence*framePeriod;
  s.stored_object_count=1;s.objects[0].distance_mm=static_cast<int16_t>(forcedRange);
  s.selected_raw_distance_mm=forcedRange;return true;}
''')
    source+=r'''
using namespace parking_module;
int main(){int checks=0;
 //600: stable +120mm wall residual at x775 is recoverable in the checked
 //outer corridor; the old x>=900 residual switch stopped this exact case.
 for(int dir:{-1,1}){
  ::routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  final_parking_reset();parking_module::routeTurnSign=dir;outerSensor=dir>0?TOF_RIGHT:TOF_LEFT;
  hostPose={775.4f,-1207.1f,dir>0?359.2f:180.8f};truePose=hostPose;truePose.y_mm+=120;
  hostTime=1000;encoder=0;hostGyroHealthy=true;framePeriod=30;stop(false);
  state=FP_APPROACH_BRAKE;stateStartMs=0;
  for(int tick=0;tick<2000&&!final_parking_aborted()&&state!=FP_SCAN_SETTLE;++tick){
   final_parking_update(dir);
   if(dc_state==DC_ENABLED){float ds=command*.02f;encoder+=ds;
    advanceParkingPose(hostPose,steeringCommand,ds);advanceParkingPose(truePose,steeringCommand,ds);
   }hostTime+=20;
  }
  if(state!=FP_SCAN_SETTLE){std::cout<<"WALL120 FAIL "<<dir<<"\n"<<robot_logger.lines.str();return 1;}++checks;
 }
 //597: three fresh stable stationary wall frames support33mm correction.
 // One frame and a still-moving edge estimate must remain insufficient.
 final_parking_reset();parking_module::routeTurnSign=-1;outerSensor=TOF_LEFT;
 hostPose={355.2f,-1204.8f,182};truePose=hostPose;hostTime=1000;stateStartMs=0;framePeriod=30;
 forcedSnapshot=true;forcedRange=292;scanPhase=5;measuredFixedInsideX=606;measuredMovingInsideX=356.4f;
 if(refreshStoppedScanWallReference())return 2;
 hostTime+=30;if(refreshStoppedScanWallReference())return 3;
 hostTime+=30;if(!refreshStoppedScanWallReference()||stoppedWallFrames<3||!applyScanLocalization())return 4;
 checks+=3;
 //601: a small scan-line correction was refused even though neutral reverse
 //has a clear complete prediction. Remain in the scan without ignoring rails.
 int alternatives=0;
 for(float dy:{0.f,-.5f,-1.f,-2.f,-3.f}){
  final_parking_reset();parking_module::routeTurnSign=1;outerSensor=TOF_RIGHT;
  hostPose={428,-1214.8f+dy,356.8f};truePose=hostPose;hostTime=2000;stateStartMs=0;
  scanPhase=2;scanArmed=scanLineEstablished=true;state=FP_SCAN_DRIVE;
  set_steering(0);if(!scanCommandSafe(hostPose,-1))continue;
  set_steering(fieldLineSteering(hostPose,-1,OBSTACLE_FINAL_PARKING_SCAN_Y_MM));
  bool correctionFits=scanCommandSafe(hostPose,-1);
  forcedRange=56;stop(false);robot_logger.lines.str("");robot_logger.lines.clear();final_parking_update(1);
  if(final_parking_aborted()||state!=FP_SCAN_DRIVE||dc_state!=DC_ENABLED||
     !scanCommandSafe(hostPose,-1))return 5;
  if(!correctionFits){if(steeringCommand!=0)return 6;++alternatives;}
  ++checks;
 }
 if(!alternatives)return 7;
 std::cout<<"RETURNED PARKING WALL/SCAN "<<checks<<" PASS\n";
}
'''
    dest=ROOT/'local_workspace/parking-returned-594-601';dest.mkdir(parents=True,exist_ok=True)
    cpp,exe=dest/'check.cpp',dest/'check.exe';cpp.write_text(source)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),
                    '-I',str(ROOT/'local_workspace/later-laps'),str(cpp),'-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],capture_output=True,text=True,timeout=180)
    (dest/'output.txt').write_text(result.stdout+result.stderr);print(result.stdout,end='')
    raise SystemExit(result.returncode)


if __name__=='__main__':main()
