"""Recorded lap-to-parking arrival anchors through complete production parking.

First parking trace and confirmed map come from immutable logs564/567. Subsequent
motion/ToF are synthetic, not replayed sensor truth. Checks plausible small pose
perturbations and extra brake coast. Also reproduces early mixed-wall correction.
Run check_later_laps.py first. Outputs stay in local_workspace.
"""
import re
import shutil
import subprocess

from check_final_parking_sequence import ROOT, fixture_source


def main():
    inputs=[]
    for number,direction,suffix in ((564,-1,'cw'),(567,1,'ccw')):
        log=(ROOT/f'simulation/evidence/parking_exit_diagnostics/20261027_log_{number}_{suffix}.txt').read_text()
        seed=re.search(r'\[FINAL_PARK_TRACE\].*?state/phase=1/0 xyh=([-\d.]+)/([-\d.]+)/([-\d.]+)',log)
        assert seed
        colors=[0]*24
        for seat,color in re.findall(r'\[PATH\] Live avoidance injected seat=(\d+) color=(RED|GREEN)',log):
            colors[int(seat)]=2 if color=='RED' else 1
        inputs.append(' '.join(map(str,[direction,*seed.groups(),*colors])))
    source=fixture_source()
    needle='bool get_tof_diagnostic_snapshot(TofSensor sensor,TofDiagnosticSnapshot &s){'
    assert source.count(needle)==1
    source=source.replace(needle,'bool ideal_tof_snapshot(TofSensor sensor,TofDiagnosticSnapshot &s){')
    source+=r'''
bool mixedWall=false;
bool get_tof_diagnostic_snapshot(TofSensor sensor,TofDiagnosticSnapshot &s){
 if(!mixedWall)return ideal_tof_snapshot(sensor,s);
 s={};s.sequence=hostTime/30;s.sampled_ms=s.sequence*30;
 s.stored_object_count=1;s.selected_raw_distance_mm=parking_module::expectedOuterWallRange(sensor,hostPose)-46;
 s.objects[0].distance_mm=int(lroundf(s.selected_raw_distance_mm));
 return true;
}
int main(){int references=0,total=0;float minWall=1e9,minPillar=1e9,maxXError=0,maxYError=0,maxHeading=0;
 // Approximate564 before the logged correction. Physical origin of its raw
 // return is unrecorded; this injects a possible stable mixed return explicitly.
 for(int dir:{-1,1})for(float x:{450.f,480.f,510.f}){
  ::routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  using namespace parking_module;
  final_parking_reset();parking_module::routeTurnSign=dir;outerSensor=dir<0?TOF_LEFT:TOF_RIGHT;
  state=FP_APPROACH_LANE;hostPose={x,-1090,dir<0?175.f:5.f};truePose=hostPose;
  hostTime=1000;mixedWall=true;const float y=hostPose.y_mm;
  updateOuterWallReference(true);hostTime+=40;bool accepted=updateOuterWallReference(true);
  if(accepted||hostPose.y_mm!=y){std::cout<<"MIXED REFERENCE MOVED POSE "<<dir<<" "<<x<<"\n";return 9;}
  if(state!=FP_APPROACH_LANE||final_parking_aborted())return 10;++references;
  // A clear approach far from either uncertain rail retains early correction.
  hostPose={-450,-1090,dir<0?180.f:0.f};truePose=hostPose;initialWallFrames=0;hostTime+=40;
  updateOuterWallReference(true);hostTime+=40;
  if(!updateOuterWallReference(true)||fabsf(hostPose.y_mm+1136)>1)return 11;++references;
 }
 mixedWall=false;
 int dir,colors[24];float sx,sy,sh;
 while(std::cin>>dir>>sx>>sy>>sh){
  for(int &c:colors)std::cin>>c;
  for(float dx:{-10.f,0.f,10.f})for(float dy:{-10.f,0.f,10.f})
  for(float dh:{-2.f,0.f,2.f})for(float coast:{0.f,10.f,25.f}){
   ::routeTurnSign=dir;resetState();baseline();
   for(int i=0;i<24;++i){seats[i].confirmed=seats[i].injected=colors[i]!=0;seats[i].red=colors[i]==2;}
   learned();joinSameColourStraightEnds(livePath,true);roundKnownCornerPairs(livePath,true);
   if(!buildOptimizedPath())return 12;
   using namespace parking_module;
   final_parking_reset();
   const float h=(sh+dh)*PI/180;
   hostPose=truePose=PositionEstimate{sx+dx+coast*cosf(h),sy+dy+coast*sinf(h),sh+dh};
   hostTime=0;encoder=0;hostGyroHealthy=true;framePeriod=30;stop(true);
   robot_logger.lines.str("");robot_logger.lines.clear();robot_logger.writes=0;
   bool captured=false;int entrySegments=0;float timeScan=0;
   for(int step=0;step<7000&&!final_parking_complete()&&!final_parking_aborted();++step){
    auto previous=state;final_parking_update(dir);
    if(previous!=FP_SCAN_DRIVE&&state==FP_SCAN_DRIVE)timeScan=hostTime;
    if(previous==FP_CAPTURE_BRAKE&&state==FP_SEGMENT_SETTLE){
     captured=true;maxXError=fmaxf(maxXError,fabsf(hostPose.x_mm-captureTargetX));
     maxYError=fmaxf(maxYError,fabsf(hostPose.y_mm-OBSTACLE_FINAL_PARKING_SCAN_Y_MM));
     maxHeading=fmaxf(maxHeading,fabsf(parking_module::wrap180(hostPose.heading_deg-baseHeadingDeg())));
    }
    if(previous==FP_SEGMENT_SETTLE&&state==FP_SEGMENT_DRIVE)++entrySegments;
    if(dc_state==DC_ENABLED){
     if(servo_disabled)return 13;
     float remaining=fabsf(command*.02f),sign=command<0?-1.f:1.f;
     while(remaining>.0001f){
      const float ds=sign*fminf(1,remaining);remaining-=fabsf(ds);encoder+=ds;
      advanceParkingPose(hostPose,steeringCommand,ds);advanceParkingPose(truePose,steeringCommand,ds);
      if(!parkingEntryPoseSafe(truePose,steeringCommand))return 14;
      for(int i=0;i<24;++i)if(i==0||seats[i].confirmed){
       ObstacleClearanceSample sample{};calculateClearanceAtPose(seats[i],truePose.x_mm,truePose.y_mm,truePose.heading_deg,sample);
       minWall=fminf(minWall,sample.wallMm);if(seats[i].confirmed)minPillar=fminf(minPillar,sample.pillarMm);
      }
     }
    }hostTime+=20;
   }
   const auto saved=hostPose;hostPose=truePose;bool contained=finalFootprintContained();hostPose=saved;
   if(!final_parking_complete()||!captured||!contained||entrySegments!=7||!timeScan||robot_logger.writes!=1){
    std::cout<<"ARRIVAL FAIL "<<dir<<" "<<dx<<" "<<dy<<" "<<dh<<" coast "<<coast<<"\n"<<robot_logger.lines.str();return 15;
   }++total;
  }
 }
 std::cout<<"WALL REFERENCES "<<references<<" PASS; ARRIVALS "<<total<<" PARKED; minimum wall/pillar mm "
   <<minWall<<"/"<<minPillar<<" max capture x/y/heading error "<<maxXError<<"/"<<maxYError<<"/"<<maxHeading<<"\n";
}
'''
    dest=ROOT/'local_workspace/parking-arrival-564-567'
    dest.mkdir(parents=True,exist_ok=True)
    (dest/'input.txt').write_text('\n'.join(inputs)+'\n')
    for mode in ('current','old_early_reference'):
        code=source
        if mode=='old_early_reference':
            needle='initial && state==FP_APPROACH_LANE && !earlyWallReferenceClearOfMarkers(pose)'
            assert code.count(needle)==1
            code=code.replace(needle,'false')
        cpp,exe=dest/f'{mode}.cpp',dest/f'{mode}.exe';cpp.write_text(code)
        subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),
                        '-I',str(ROOT/'local_workspace/later-laps'),str(cpp),'-o',str(exe)],check=True)
        result=subprocess.run([str(exe)],input='\n'.join(inputs)+'\n',text=True,capture_output=True,timeout=240)
        (dest/f'{mode}.txt').write_text(result.stdout+result.stderr)
        print(result.stdout,end='')
        assert result.returncode==(0 if mode=='current' else 9),result.returncode
        if mode!='current':print('Negative control PASS: old early registration accepts the mixed return')


if __name__=='__main__':main()
