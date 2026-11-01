"""603 clipped-depth regression, exact wall shortcut and real entry-only mode.

No original pixels, actuator latency or independently measured chassis poses.
Artifacts stay below local_workspace; production functions are compiled afresh.
"""
import shutil
import subprocess
import sys

from check_final_parking_sequence import ROOT, fixture_source as parking_fixture
from check_far_obstacle_acquisition import fixture_source as acquisition_fixture


def run(name, source, expected=0):
    dest=ROOT/'local_workspace/returned-603'/name
    dest.mkdir(parents=True,exist_ok=True)
    cpp,exe=dest/'check.cpp',dest/'check.exe'
    cpp.write_text(source)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2',
        '-I',str(ROOT/'simulation/host'),'-I',str(ROOT/'include'),
        '-I',str(ROOT/'local_workspace/later-laps'),str(cpp),'-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],capture_output=True,text=True,timeout=240)
    (dest/'output.txt').write_text(result.stdout+result.stderr)
    print(result.stdout,end='')
    if result.returncode!=expected: raise SystemExit(result.returncode or 1)


def main():
    acquisition=acquisition_fixture()
    negative='--without-clipped-depth-gate' in sys.argv
    if negative:
        acquisition=acquisition.replace('if (blob->minX<=2 || blob->maxX>=317)','if (false)')
    run('clipped-negative' if negative else 'clipped',acquisition+r'''
int main(){int checks=0;
 for(int cx:{32,34,36}){
  for(auto &s:seats){s={};s.x=s.y=9000;}
  for(int st=0;st<3;++st)for(int side=0;side<2;++side){
   auto &s=seats[12+2*st+side];s.x=-500+500*st;s.y=side?1100:900;
   s.headingDeg=0;s.pathDistanceMm=775;
  }
  hostPose={-708,958,36};clockMs+=1000;
  Blob b{};b.found=true;b.color=ColorType::GREEN;b.centerX=cx;
  b.minX=0;b.maxX=2*cx;b.minY=90;b.maxY=144;b.area=1000;
  // Matches the170mm clipped-depth placeholder, not a calibrated range.
  if(obstacle_estimate_camera_range_mm(&b)>200)return 1;
  bool assigned=false;
  for(int i=0;i<3;++i){clockMs+=80;auto r=obstacle_path_observe(&b);
   assigned|=r.seatId!=-1;++checks;}
  if(assigned||seats[13].confirmed){
   std::cout<<"603 CLIPPED FALSE END confirmed="<<seats[13].confirmed<<"\n";return 2;
  }
  // A subsequent uncropped ordinary middle view remains acquirable.
  b.minX=154;b.maxX=174;b.centerX=164;b.maxY=140;b.minY=80;
  float range=obstacle_estimate_camera_range_mm(&b);
  hostPose={-range-OBSTACLE_CAMERA_LOCAL_X_MM,1100,0};
  clockMs+=1000;obstacle_path_observe(&b);clockMs+=80;
  auto r=obstacle_path_observe(&b);if(!r.confirmed||r.seatId!=15){
   std::cout<<"MIDDLE FAIL status/seat/confirmed="<<int(r.status)<<"/"<<int(r.seatId)<<"/"<<r.confirmed<<"\n";return 3;
  }++checks;
 }
 std::cout<<"603 CLIPPED DEPTH "<<checks<<" PASS\n";
}
''',expected=2 if negative else 0)
    if negative:
        print('PASS negative control: clipped placeholder confirms wrong end13')
        return
    run('parking',parking_fixture()+r'''
using namespace parking_module;
bool referenceConnector(const PositionEstimate &p){
 ObstacleClearanceSample walls;
 if(!obstacle_path_sample_pose_clearance(0,p.x_mm,p.y_mm,p.heading_deg,walls)||
    !walls.valid||walls.wallMm<5)return false;
 float h=p.heading_deg*PI/180.f,dx=OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM*cosf(h),
       dy=OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM*sinf(h),l=dx*dx+dy*dy;
 for(int i=0;i<24;++i){ObstacleSeatInfo s;if(!obstacle_path_get_seat(i,s)||!s.confirmed)continue;
  float t=::clampFloat(((s.xMm-p.x_mm)*dx+(s.yMm-p.y_mm)*dy)/fmaxf(l,1e-6f),0,1);
  float d=hypotf(s.xMm-p.x_mm-t*dx,s.yMm-p.y_mm-t*dy)-
     OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM-OBSTACLE_PILLAR_MOVEMENT_RADIUS_MM;
  if(!isfinite(d)||d<10)return false;
 }return true;
}
bool referenceEntry(const PositionEstimate &p,int steering,float margin){
 if(!referenceConnector(p))return false;
 using namespace parking_start_footprint;
 Quad bodies[6];robotQuads(p.x_mm,p.y_mm,p.heading_deg,(steering>0)-(steering<0),margin,bodies,
  OBSTACLE_FINAL_PARKING_STRAIGHT_WIDTH_MM);
 Quad fixed=rectangle(490,-1400,10+margin,100+margin,0);
 Quad moving=rectangle(movingInsideFaceX()-10,-1400,10+margin,100+margin,0);
 for(int offset=0;offset<(steering && margin>=3?2:1);++offset)for(auto b:bodies){
  if(offset)for(auto &pt:b.p){pt.x+=OBSTACLE_FINAL_PARKING_STEER_FRONT_RESERVE_MM*cosf(p.heading_deg*PI/180.f);
   pt.y+=OBSTACLE_FINAL_PARKING_STEER_FRONT_RESERVE_MM*sinf(p.heading_deg*PI/180.f);}
  for(const auto &pt:b.p)if(pt.y<=OBSTACLE_SOUTH_OUTER_WALL_Y_MM+margin)return false;
  if(overlaps(b,fixed)||overlaps(b,moving))return false;
 }return true;
}
int main(){int checks=0,parked=0;
 ::routeTurnSign=-1;resetState();baseline();
 for(int map:{0,1}){
  for(auto &s:seats)s.confirmed=map;
  for(float x:{-1510.f,-1420.f,-1000.f,-500.f,0.f,209.74f,326.2f,775.f,1420.f,1510.f})
  for(float y:{-1510.f,-1430.f,-1300.f,-1257.2f,-1205.4f,-900.f,-580.f,-500.f,0.f})
  for(float h:{-180.f,-135.f,-90.f,-45.f,0.f,45.f,90.f,135.f}){
   PositionEstimate p{x,y,h};
   if(connectorClearanceSafe(p)!=referenceConnector(p)){
    std::cout<<"WALL DIFFERENCE "<<x<<"/"<<y<<"/"<<h<<"\n";return 4;
   }++checks;
  }
 }
 std::cout<<"PARK WALL EQUIVALENCE "<<checks<<" PASS\n";
 int polygons=0;
 for(auto &s:seats)s.confirmed=false;
 for(float x=160;x<=520;x+=30)for(float y=-1450;y<=-1150;y+=25)
 for(float h=-180;h<180;h+=15)for(int steering:{-50,0,50}){
  PositionEstimate p{x,y,h};
  if(parkingEntryPoseSafe(p,steering,3)!=referenceEntry(p,steering,3))return 9;
  ++polygons;
 }
 std::cout<<"PARK RAIL EQUIVALENCE "<<polygons<<" PASS\n";
 for(int dir:{-1,1})for(float scale:{1.f,1.15f}){
  ::routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=false;
  hostTime=1000;encoder=0;hostGyroHealthy=true;
  robot_logger.lines.str("");robot_logger.lines.clear();robot_logger.writes=0;
  final_parking_start_entry_practice(dir);truePose=hostPose;
  if(state!=FP_CAPTURE_BRAKE||dc_state==DC_ENABLED||
     fabsf(hostPose.y_mm+1205.4f)>.01f||fabsf(measuredGapMm-247.5f)>.01f)return 5;
  bool contact=false;
  for(int tick=0;tick<6000&&!final_parking_complete()&&!final_parking_aborted();++tick){
   final_parking_update(dir);
   if(dc_state==DC_ENABLED){
    if(servo_disabled)return 6;
    float ds=command*.02f;encoder+=ds;
    advanceEntryModel(hostPose,steeringCommand,ds,scale);truePose=hostPose;
    if(!parkingEntryPoseSafe(truePose,steeringCommand,0))contact=true;
   }hostTime+=20;
  }
  if(!final_parking_complete()||!finalFootprintContained()||contact||steeringCommand!=0||robot_logger.writes!=1){
   std::cout<<"ENTRY PRACTICE FAIL "<<dir<<"/"<<scale<<"\n"<<robot_logger.lines.str();return 7;
  }++parked;
 }
 std::cout<<"ENTRY PRACTICE "<<parked<<" PARKED\n";
 // Replay the rounded603 post-brake anchor with its measured radius.
 for(int dir:{-1,1}){
  ::routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=false;
  final_parking_reset();parking_module::routeTurnSign=dir;scanPhase=5;measuredGapMm=247.7f;
  localToField(153.8f,242.8f,57.2f,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
  entryRadiusScale=1.213f;entryRadiusMeasured=true;
  if(!planParkingEntry(hostPose)){std::cout<<"603 REPLAN FAIL "<<dir<<"\n";return 8;}
 }
 std::cout<<"603 REGISTERED REPLAN 2 PASS\n";
}
''')


if __name__=='__main__':
    main()
