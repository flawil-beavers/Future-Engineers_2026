"""CW570 bounded scout visibility and CW571 stopped parking wall reference.

Uses logged pose anchors and production geometry/state machine, with ideal
motion/ToF beyond those anchors. No claim of original image/physical replay.
"""
import shutil
import subprocess
from check_cw_start_planner import ROOT, fixture_source as scout_fixture
from check_connector_servo_resume import block
from check_final_parking_sequence import fixture_source as park_fixture

DEST=ROOT/'local_workspace/cw-570-571'

def run(name,source,expected=0):
    DEST.mkdir(parents=True,exist_ok=True)
    cpp,exe=DEST/f'{name}.cpp',DEST/f'{name}.exe';cpp.write_text(source)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),
                    '-I',str(ROOT/'local_workspace/later-laps'),str(cpp),'-o',str(exe)],check=True)
    r=subprocess.run([str(exe)],capture_output=True,text=True,timeout=180)
    (DEST/f'{name}.txt').write_text(r.stdout+r.stderr);print(r.stdout,end='')
    assert r.returncode==expected,(name,r.returncode)

def main():
    path=(ROOT/'src/obstacle_path.cpp').read_text()
    source=scout_fixture().split('int main()')[0]
    for sig in ('void seatCameraGeometry(', 'bool seatComfortablyVisible(', 'float cwScoutViewExtensionMm('):
        source+=block(path,path.rindex(sig))+'\n'
    source+=r'''
int main(){int count=0;float maxExtra=0;
 routeTurnSign=-1;baseline();
 for(float dx:{-5.f,0.f,5.f})for(float dy:{-5.f,0.f,5.f})for(float dh:{-2.f,0.f,2.f}){
  PositionEstimate p{364.9f+dx,-1276.9f+dy,91.2f+dh};
  float originalBearing,originalRange;seatCameraGeometry(0,p,originalBearing,originalRange);
  float extra=cwScoutViewExtensionMm(p,0,130.4f);
  if(extra<=0||extra>29.6f||!preflightCwStartArc(p,extra)){
   std::cout<<"SCOUT FAIL "<<dx<<" "<<dy<<" "<<dh<<" seat "<<seats[0].x<<","<<seats[0].y<<" bearing/range "<<originalBearing<<"/"<<originalRange<<" extra "<<extra<<" clear30 "<<preflightCwStartArc(p,30)<<"\n";return 1;}
  float k=-routeTurnSign/OBSTACLE_PARKING_ENTRY_SCAN_RADIUS_MM,h=p.heading_deg*PI/180,next=h-k*extra;
  PositionEstimate view=p;view.x_mm+=(sinf(next)-sinf(h))/k;view.y_mm+=(cosf(h)-cosf(next))/k;view.heading_deg=next*180/PI;
  if(!seatComfortablyVisible(0,view)||cwScoutViewExtensionMm(view,0,130.4f+extra)!=0)return 2;
  if(cwScoutViewExtensionMm(p,0,160.1f)!=0)return 3;
  parkingCwShortStart=false;if(cwScoutViewExtensionMm(p,0,130.4f)!=0)return 4;parkingCwShortStart=true;
  if(cwScoutViewExtensionMm(PositionEstimate{490,-1400,90},0,130.4f)!=0)return 5;
  maxExtra=fmaxf(maxExtra,extra);++count;
 }
 std::cout<<"570 SCOUT "<<count<<" PASS max extension mm "<<maxExtra<<"; bounds/visible/CCW/unsafe exclusions PASS\n";
}
'''
    run('570_scout',source)
    source=park_fixture()
    needle='bool get_tof_diagnostic_snapshot(TofSensor sensor,TofDiagnosticSnapshot &s){'
    source=source.replace(needle,'bool ideal_tof(TofSensor sensor,TofDiagnosticSnapshot &s){')
    source+=r'''
bool overrideWall=false;TofDiagnosticSnapshot wallFrame{};
bool get_tof_diagnostic_snapshot(TofSensor sensor,TofDiagnosticSnapshot &s){
 if(overrideWall){s=wallFrame;return true;}
 if(parking_module::state!=parking_module::FP_SCAN_BRAKE)return ideal_tof(sensor,s);
 s={};s.sequence=hostTime/30;s.sampled_ms=s.sequence*30;s.stored_object_count=1;
 s.selected_raw_distance_mm=parking_module::expectedOuterWallRange(sensor,truePose);
 s.objects[0].distance_mm=int(lroundf(s.selected_raw_distance_mm));return true;
}
int main(){int total=0;
 {
  using namespace parking_module;
  final_parking_reset();outerSensor=TOF_LEFT;
  hostPose={319.2f,-1206.5f,181.4f};truePose=hostPose;
  hostTime=1000;stateStartMs=0;overrideWall=true;
  wallFrame={};wallFrame.stored_object_count=1;
  wallFrame.objects[0].distance_mm=int(lroundf(expectedOuterWallRange(outerSensor,hostPose)));
  wallFrame.sequence=1;wallFrame.sampled_ms=200;
  if(refreshStoppedScanWallReference())return 20; // stale/pre-brake
  wallFrame.sequence=2;wallFrame.sampled_ms=950;
  wallFrame.objects[0].filter_accepted=false;
  if(refreshStoppedScanWallReference())return 21;
  wallFrame.objects[0].filter_accepted=true;
  wallFrame.sequence=3;
  if(refreshStoppedScanWallReference())return 22; // first valid
  if(refreshStoppedScanWallReference())return 23; // duplicate
  wallFrame.sequence=4;wallFrame.objects[0].distance_mm+=12;
  if(refreshStoppedScanWallReference())return 24; // unstable pair
  wallFrame.sequence=5;wallFrame.objects[0].distance_mm-=12;
  if(refreshStoppedScanWallReference())return 25; // restarts stability
  wallFrame.sequence=6;
  if(!refreshStoppedScanWallReference())return 26;
  overrideWall=false;
  std::cout<<"571 WALL stale/unaccepted/duplicate/unstable rejection and fresh pair PASS\n";
 }
 for(int dir:{-1,1})for(float dx:{-3.f,0.f,3.f})for(float dy:{-3.f,0.f,3.f})for(float dh:{-1.f,0.f,1.f}){
  ::routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  using namespace parking_module;
  final_parking_reset();parking_module::routeTurnSign=dir;outerSensor=dir<0?TOF_LEFT:TOF_RIGHT;
  hostPose={319.2f+dx,-1206.5f+dy,dir<0?181.4f+dh:-1.4f-dh};truePose=hostPose;
  truePose.x_mm-=98.2f;truePose.y_mm+=3.4f;
  scanPhase=5;measuredFixedInsideX=578.2f;measuredMovingInsideX=330.6f;
  latestWallPose=hostPose;latestWallRange=expectedOuterWallRange(outerSensor,hostPose)-17.6f;
  state=FP_SCAN_BRAKE;stateStartMs=0;hostTime=1000;encoder=0;hostGyroHealthy=true;stop(true);
  robot_logger.lines.str("");robot_logger.lines.clear();robot_logger.writes=0;
  for(int step=0;step<3500&&!final_parking_complete()&&!final_parking_aborted();++step){
   final_parking_update(dir);
   if(dc_state==DC_ENABLED){
    if(servo_disabled||!parkingEntryPoseSafe(truePose,steeringCommand))return 6;
    float ds=command*.02f;encoder+=ds;advanceParkingPose(hostPose,steeringCommand,ds);advanceParkingPose(truePose,steeringCommand,ds);
   }hostTime+=20;
  }
  if(!final_parking_complete()){std::cout<<"571 FAIL "<<dir<<" "<<dx<<" "<<dy<<" "<<dh<<"\n"<<robot_logger.lines.str();return 7;}
  const auto saved=hostPose;hostPose=truePose;bool contained=finalFootprintContained();hostPose=saved;
  if(!contained||robot_logger.writes!=1)return 8;++total;
 }
 std::cout<<"571 PARK "<<total<<" mirrored/perturbed cases PARKED with fresh stopped reference\n";
}
'''
    run('571_park',source)
    run('571_old_edge_reference',source.replace('if (!refreshStoppedScanWallReference())','if (false)'),expected=7)
    print('571 negative control: old edge reference reproduces capture_alignment_preflight as expected')

if __name__=='__main__':main()
