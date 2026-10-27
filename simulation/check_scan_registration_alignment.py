"""Full parking replay with log-510-sized wall registration and X drift."""
import shutil
import subprocess
from check_cw_start_planner import ROOT
from check_final_parking_sequence import fixture_source


def main():
    f=fixture_source()+r'''
int main(){int total=0,bad=0;
 for(int dir:{-1,1})for(float bias:{-25.4f,25.4f}){
  routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  learned();buildOptimizedPath();parking_module::final_parking_reset();
  parking_module::routeTurnSign=dir;parking_module::outerSensor=dir>0?TOF_RIGHT:TOF_LEFT;
  parking_module::state=parking_module::FP_APPROACH_BRAKE;
  hostPose={797.4f,-1207.f,dir>0?-.5f:179.5f};truePose=hostPose;
  truePose.x_mm-=150;truePose.y_mm+=bias;
  hostTime=400;encoder=0;stop(true);hostGyroHealthy=true;bool paused=false,collision=false,scanStarted=false;
  robot_logger.lines.str("");robot_logger.lines.clear();robot_logger.writes=0;
  for(int step=0;step<20000&&!parking_module::final_parking_complete()&&!parking_module::final_parking_aborted();++step){
   if(!paused&&parking_module::state==parking_module::FP_SCAN_ALIGN_DRIVE){
    auto saved=parking_module::state;auto start=parking_module::scanAlignStartMs;
    hostGyroHealthy=false;parking_module::final_parking_update(dir);
    if(dc_state==DC_ENABLED||parking_module::state!=saved)++bad;
    hostTime+=1600;hostGyroHealthy=true;parking_module::final_parking_sensor_resume();
    if(parking_module::scanAlignStartMs!=start+1600)++bad;paused=true;
   }
   auto before=parking_module::state;parking_module::final_parking_update(dir);
   if(before!=parking_module::FP_SCAN_DRIVE&&parking_module::state==parking_module::FP_SCAN_DRIVE){
    scanStarted=true;if(!parking_module::linePoseReady(hostPose,OBSTACLE_FINAL_PARKING_SCAN_Y_MM))++bad;
   }
   if(dc_state==DC_ENABLED){
    if(!parking_module::parkingEntryPoseSafe(truePose,steeringCommand))collision=true;
    float ds=command*.02f;encoder+=ds;
    parking_module::advanceParkingPose(hostPose,steeringCommand,ds);
    parking_module::advanceParkingPose(truePose,steeringCommand,ds);
   }
   hostTime+=20;
  }
  auto saved=hostPose;hostPose=truePose;bool contained=parking_module::finalFootprintContained();hostPose=saved;
  bool ok=paused&&scanStarted&&contained&&!collision&&parking_module::final_parking_complete();
  if(!ok){++bad;std::cout<<"FAIL "<<dir<<" "<<bias<<" collision="<<collision<<"\n"<<robot_logger.lines.str();}
  ++total;
 }
 std::cout<<"REGISTER ALIGN "<<total<<" failures "<<bad<<"\n";return bad?1:0;
}
'''
    dest=ROOT/'local_workspace/fix_508_511';dest.mkdir(parents=True,exist_ok=True)
    cpp,exe=dest/'scan_registration.cpp',dest/'scan_registration.exe';cpp.write_text(f)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),
        '-I',str(ROOT/'simulation/host'),str(cpp),'-o',str(exe)],check=True)
    r=subprocess.run([str(exe)],capture_output=True,text=True)
    (dest/'scan_registration_output.txt').write_text(r.stdout+r.stderr)
    print('\n'.join(l for l in r.stdout.splitlines() if l.startswith(('FAIL','REGISTER'))))
    raise SystemExit(r.returncode)


if __name__=='__main__':main()
