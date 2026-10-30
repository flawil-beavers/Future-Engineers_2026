"""Archived CCW return-RED geometry and independently sensed parking drift."""
import math
from check_connector_servo_resume import ROOT, block
from check_cw_556_558 import run
from check_far_obstacle_acquisition import fixture_source
from check_final_parking_sequence import fixture_source as parking_fixture


def main():
    rows=[]
    for n in (567,569):
        for line in (ROOT/f'simulation/evidence/parking_exit_diagnostics/20261027_log_{n}_ccw.txt').read_text().splitlines():
            if '[RED SEAT]' not in line or 'nearest/error=1/' not in line:continue
            f=dict(t.split('=',1) for t in line.split() if '=' in t)
            x,y,h=map(float,f['pose'].split(','))
            if not -500<y<100:continue
            b,r=map(float,f['bearing/range'].split('/'))
            cx=round(164.4-248.9*math.tan(math.radians(b)))
            foot=int(f['image_x/foot'].split('/')[1])
            rows.append((x,y,h,cx,foot))
    path=(ROOT/'src/obstacle_path.cpp').read_text()
    source=fixture_source().replace(
        'bool mappedParkingSeatFootMatches(uint8_t,const PositionEstimate &,int,bool){return true;}',
        block(path,path.index('float wrap180('))+'\n'+
        block(path,path.index('void seatCameraGeometry('))+'\n'+
        block(path,path.index('bool mappedParkingSeatFootMatches(')))
    code=source+r'''
int main(){float x,y,h;int cx,foot,total=0;
 while(std::cin>>x>>y>>h>>cx>>foot){
  for(auto &s:seats){s={};s.x=s.y=9000;}
  routeTurnSign=1;parkingSectionInnerSeatsOnly=true;
  for(int st=0;st<3;++st){int i=st*2;seats[i].x=seats[i+1].x=-500+500*st;
   seats[i].y=-1100;seats[i+1].y=-900;seats[i].headingDeg=seats[i+1].headingDeg=0;
   seats[i].pathDistanceMm=seats[i+1].pathDistanceMm=775+500*st;}
  seats[22].x=-1100;seats[22].y=-500;seats[22].confirmed=true;seats[22].red=false;
  hostPose={x,y,h};clockMs+=1000;
  Blob b{};b.found=true;b.color=ColorType::RED;b.centerX=cx;
  b.minX=cx-10;b.maxX=cx+10;b.minY=80;b.maxY=foot;b.area=650;
  auto a=obstacle_path_observe(&b);clockMs+=76;auto c=obstacle_path_observe(&b);
  if(!c.confirmed||c.seatId!=1){std::cout<<"RETURN RED FAIL "<<x<<" "<<y<<" "<<a.status<<"\n";return 1;}
  // Initial parking views must keep their strict foot/snap protection.
  seats[1].confirmed=false;seats[1].redVotes=0;parkingEntryActive=true;clockMs+=1000;
  obstacle_path_observe(&b);clockMs+=76;
  if(obstacle_path_observe(&b).confirmed && a.snapErrorMm>140.0f)return 2;
  parkingEntryActive=false;++total;
 }
 std::cout<<"CCW RETURN RED "<<total<<" archived geometries PASS; initial-view guard retained\n";
}
'''
    data='\n'.join(' '.join(map(str,r)) for r in rows)+'\n'
    run('567_569_return_red',code,data)
    run('567_569_without_return_bearing',code.replace(
        'const int returnSeat=resolveCcwReturnRedByBearing(result);',
        'const int returnSeat=-1;'),data,expected=1)

    code=parking_fixture()+r'''
int main(){int total=0;
 for(int dir:{-1,1})for(float dy:{-5.f,0.f,5.f})for(float dh:{-1.f,0.f,1.f}){
  routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  parking_module::final_parking_reset();parking_module::routeTurnSign=dir;
  parking_module::outerSensor=dir<0?TOF_LEFT:TOF_RIGHT;
  truePose={1051.9f,-1104.7f+dy,dir>0?359.4f+dh:180.6f-dh};
  hostPose=truePose;hostPose.y_mm-=102;
  hostTime=1000;encoder=0;stop(false);
  parking_module::updateOuterWallReference(true);hostTime+=40;
  if(!parking_module::updateOuterWallReference(true)){
   std::cout<<"567 WALL REGISTRATION FAIL "<<dir<<" "<<dy<<" "<<dh<<"\n";return 3;}
  if(fabsf(hostPose.y_mm-truePose.y_mm)>1)return 4;
  parking_module::scanAlignStartMs=hostTime;
  parking_module::scanAlignSign=-1;parking_module::state=parking_module::FP_SCAN_ALIGN_DRIVE;
  bool ready=false;
  for(int step=0;step<1000;++step){
   parking_module::final_parking_update(dir);
   if(parking_module::final_parking_aborted()){
    std::cout<<"567 ALIGN ABORT "<<dir<<" "<<dy<<" "<<dh<<"\n"<<robot_logger.lines.str();return 5;}
   if(parking_module::state==parking_module::FP_SCAN_SETTLE){ready=true;break;}
   if(dc_state==DC_ENABLED){
    if(!parking_module::parkingEntryPoseSafe(truePose,steeringCommand))return 6;
    float ds=command*.02f;encoder+=ds;
    parking_module::advanceParkingPose(hostPose,steeringCommand,ds);
    parking_module::advanceParkingPose(truePose,steeringCommand,ds);
   }hostTime+=20;
  }
  if(!ready)return 7;++total;
 }
 std::cout<<"567 WALL DRIFT "<<total<<" mirrored/perturbed registrations and safe scan-line recoveries PASS\n";
}
'''
    run('567_wall_drift',code)
    run('567_old_wall_limit',code.replace(
        'initial && pose.x_mm>=900.0f ? 120.0f :',
        'initial && pose.x_mm>=900.0f ? 80.0f :'),expected=3)


if __name__=='__main__':main()
