"""Production registered-capture selection, full entry and appended-state health.

Uses synthetic registered poses around the CW564 failure and the longer CCW
capture. Independent legacy selection is a negative/efficiency control. CAD
plant and ideal ToF do not model physical slip, actuator lag or camera images.
Run check_later_laps.py first; generated files remain in local_workspace.
"""
import shutil
import subprocess

from check_final_parking_sequence import ROOT, fixture_source


def main():
    fixture = fixture_source()
    needle = 'if (captureAlignPending && captureDirectPreflight(get_position_struct()))'
    assert fixture.count(needle) == 1
    fixture = fixture.replace('namespace parking_module {',
                              'bool legacyCapture=false;\nnamespace parking_module {')
    fixture = fixture.replace(needle, 'if (!legacyCapture && captureAlignPending && captureDirectPreflight(get_position_struct()))')
    driver = r'''
struct Result {bool parked=false, direct=false, unsafe=false;float travel=0;unsigned long time=0;};
Result runCapture(int dir,float x,float dy,float dh,bool legacy){
 ::routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
 parking_module::final_parking_reset();parking_module::routeTurnSign=dir;
 using namespace parking_module;
 outerSensor=dir>0?TOF_RIGHT:TOF_LEFT;scanPhase=5;
 measuredFixedInsideX=480;measuredMovingInsideX=232.5f;
 hostPose=truePose=PositionEstimate{x,OBSTACLE_FINAL_PARKING_SCAN_Y_MM+dy,
     dir>0?dh:180.f-dh};
 latestWallPose=hostPose;latestWallRange=expectedOuterWallRange(outerSensor,hostPose);
 hostTime=1000;encoder=0;hostGyroHealthy=true;framePeriod=30;legacyCapture=legacy;
 stop(true);state=FP_SCAN_BRAKE;stateStartMs=0;
 robot_logger.lines.str("");robot_logger.lines.clear();robot_logger.writes=0;
 Result r;
 for(int step=0;step<3500&&!final_parking_complete()&&!final_parking_aborted();++step){
  final_parking_update(dir);
  if(dc_state==DC_ENABLED){
   r.unsafe|=servo_disabled;
   float remaining=fabsf(command*.02f),sign=command<0?-1.f:1.f;
   while(remaining>.0001f){
    const float ds=sign*fminf(1,remaining);remaining-=fabsf(ds);encoder+=ds;r.travel+=fabsf(ds);
    advanceParkingPose(hostPose,steeringCommand,ds);advanceParkingPose(truePose,steeringCommand,ds);
    r.unsafe|=!parkingEntryPoseSafe(truePose,steeringCommand);
   }
  }hostTime+=20;
 }
 r.parked=final_parking_complete();r.time=hostTime-1000;
 r.direct=robot_logger.lines.str().find("direct alignment swept preflight PASS")!=std::string::npos;
 if(r.parked){const auto saved=hostPose;hostPose=truePose;r.unsafe|=!finalFootprintContained();hostPose=saved;}
 return r;
}
int main(){
 int total=0,parked=0,rejected=0,direct=0,rescued=0,improved=0;float bestSaved=0,secondsSaved=0;
 for(int dir:{-1,1})for(float x:{160.f,200.f,226.6f})
 for(float dy:{-15.f,-10.f,10.f,15.f})for(float dh:{-2.f,0.f,2.f}){
  auto old=runCapture(dir,x,dy,dh,true),now=runCapture(dir,x,dy,dh,false);++total;
  if(now.unsafe||(!now.parked && (old.parked || now.travel>old.travel+1))){
   std::cout<<"CAPTURE FAIL "<<dir<<" "<<x<<" "<<dy<<" "<<dh<<" unsafe "<<now.unsafe<<"\n"
       <<robot_logger.lines.str();return 1;}
  parked+=now.parked;rejected+=!now.parked;
  direct+=now.direct;rescued+=now.parked&&!old.parked;
  if(old.parked){
   if(now.travel>old.travel+1 || now.time>old.time+20){std::cout<<"CAPTURE REGRESSION\n";return 2;}
   if(old.travel-now.travel>1)++improved;
   bestSaved=fmaxf(bestSaved,old.travel-now.travel);
   secondsSaved=fmaxf(secondsSaved,(float(old.time)-now.time)/1000);
  }
 }
 if(!direct||!improved){std::cout<<"NO DIRECT BENEFIT\n";return 3;}
 std::cout<<"CAPTURE "<<total<<" parked "<<parked<<" retained_rejections "<<rejected
     <<" direct "<<direct<<" shorter "<<improved
     <<" legacy_rejected "<<rescued<<" max_saved_mm/s "<<bestSaved<<"/"<<secondsSaved<<"\n";
 // The recorded short CW564 approach must retain its necessary realignment.
 auto shortCw=runCapture(-1,226.6f,10.6f,-2.f,false);
 if(!shortCw.parked||shortCw.direct||shortCw.unsafe)return 4;
 int health=0;
 for(int dir:{-1,1})for(auto phase:{parking_module::FP_CAPTURE_ALIGN_DRIVE,parking_module::FP_CAPTURE_ALIGN_BRAKE}){
  ::routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  parking_module::final_parking_reset();parking_module::routeTurnSign=dir;
  using namespace parking_module;
  hostPose=truePose=PositionEstimate{250,-1205.4f,dir>0?0.f:180.f};
  captureTargetX=209.74f;state=phase;stateStartMs=1000;hostTime=1100;encoder=0;
  hostGyroHealthy=false;set_speed(45);final_parking_update(dir);++health;
  if(state!=phase||dc_state!=DC_DISABLED||!sensorHoldPending)return 5;
  hostTime+=600;hostGyroHealthy=true;final_parking_sensor_resume();
  if(stateStartMs!=1600)return 6;
  final_parking_update(dir);++health;
  if(final_parking_aborted() || (phase==FP_CAPTURE_ALIGN_DRIVE && (dc_state!=DC_ENABLED||servo_disabled)))return 7;
  hostPose.x_mm=NAN;final_parking_update(dir);++health;
  if(!final_parking_aborted()||dc_state!=DC_DISABLED)return 8;
 }
 std::cout<<"CAPTURE HEALTH "<<health<<" PASS; short CW retains alignment\n";
}
'''
    dest = ROOT/'local_workspace/parking-direct-capture'
    dest.mkdir(parents=True, exist_ok=True)
    for mode in ('current', 'missing_health_guard'):
        source = fixture
        if mode == 'missing_health_guard':
            needle = ' ||\n         state == FP_CAPTURE_ALIGN_DRIVE || state == FP_CAPTURE_ALIGN_BRAKE'
            assert source.count(needle) == 1
            source = source.replace(needle, '')
        cpp, exe = dest/f'{mode}.cpp', dest/f'{mode}.exe'
        cpp.write_text(source+driver)
        subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'include'),
                        '-I', str(ROOT/'local_workspace/later-laps'), str(cpp), '-o', str(exe)], check=True)
        result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=120)
        (dest/f'{mode}.txt').write_text(result.stdout+result.stderr)
        if mode == 'current':
            print(result.stdout, end='')
            assert result.returncode == 0, result.returncode
        else:
            assert result.returncode == 5, result.returncode
            print('Negative control PASS: omitted capture health gate issues motion with unhealthy gyro')


if __name__ == '__main__':
    main()
