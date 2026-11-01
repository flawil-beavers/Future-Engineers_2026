"""Recorded bearing/pose and start-control regressions; no original pixels.

Builds production acquisition and actual CCW refinement functions with hardware
stubs. Reconstructed blobs validate association, not physical colour detection.
"""
import re
import shutil
import subprocess

from check_cw_start_planner import ROOT, fixture_source as geometry_fixture
from check_far_obstacle_acquisition import fixture_source as acquisition_fixture
from check_connector_servo_resume import block


def run(name, source, data=''):
    dest=ROOT/'local_workspace'/'returned-594-601'/name
    dest.mkdir(parents=True,exist_ok=True)
    cpp,exe=dest/'check.cpp',dest/'check.exe'
    cpp.write_text(source)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'simulation/host'),
                    '-I',str(ROOT/'include'),str(cpp),'-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],input=data,capture_output=True,text=True,timeout=180)
    (dest/'output.txt').write_text(result.stdout+result.stderr)
    print(result.stdout,end='')
    if result.returncode: raise SystemExit(result.returncode)


def main():
    path=(ROOT/'src/obstacle_path.cpp').read_text()
    rows=[]
    for number in (600,601):
        file=next((ROOT/'simulation/evidence/parking_exit_diagnostics').glob(f'*_log_{number}_*.txt'))
        for line in file.read_text().splitlines():
            if '[GREEN SEAT PRIMARY]' not in line or 'accepted=12' not in line: continue
            f=dict(t.split('=',1) for t in line.split() if '=' in t)
            rows.append(f['pose'].replace(',',' ')+' '+f['image_x/foot'].replace('/',' '))
    run('association',acquisition_fixture()+r'''
int main(){float x,y,h;int cx,foot,total=0;
 while(std::cin>>x>>y>>h>>cx>>foot){
  for(auto &s:seats){s={};s.x=s.y=9000;}
  for(int st=0;st<3;++st)for(int side=0;side<2;++side){
   auto &s=seats[12+2*st+side];s.x=500-500*st;s.y=side==0?1100:900;
   s.headingDeg=180;s.pathDistanceMm=775+500*st;
  }
  hostPose={x,y,h};clockMs+=1000;
  Blob b{};b.found=true;b.color=ColorType::GREEN;b.centerX=cx;
  b.minX=cx-10;b.maxX=cx+10;b.minY=80;b.maxY=foot;b.area=600;
  obstacle_path_observe(&b);clockMs+=80;obstacle_path_observe(&b);
  if(seats[12].confirmed){std::cout<<"FALSE END "<<x<<"/"<<y<<"/"<<h<<"\n";return 1;}++total;
 }
 std::cout<<"RECORDED FALSE END "<<total<<" PASS\n";
}
''','\n'.join(rows)+'\n')
    selection=acquisition_fixture()+r'''
struct TestVision {VisionResult result;const VisionResult &getResult(){return result;}} vision;
const Blob *selected=nullptr;const Blob *getLargestValidObstacle(){return selected;}
'''+block(path,path.index('const Blob *getDiscoveryObstacle()'))+r'''
int main(){
 for(auto &s:seats){s={};s.x=s.y=9000;}
 hostPose={0,0,0};auto &r=vision.result.redPillar;auto &g=vision.result.greenPillar;
 r.found=g.found=true;r.color=ColorType::RED;g.color=ColorType::GREEN;
 r.centerX=g.centerX=164;r.minX=g.minX=154;r.maxX=g.maxX=174;
 r.minY=g.minY=70;r.maxY=170;g.maxY=104;r.area=g.area=600;selected=&r;
 seats[6].x=125+obstacle_estimate_camera_range_mm(&r);seats[6].y=0;seats[6].confirmed=true;
 if(getDiscoveryObstacle()!=&g)return 1;
 seats[6].confirmed=false;if(getDiscoveryObstacle()!=&r)return 2;
 seats[6].confirmed=true;g.found=false;if(getDiscoveryObstacle()!=&r)return 3;
 selected=nullptr;if(getDiscoveryObstacle()!=nullptr)return 4;
 std::cout<<"KNOWN RED / NEXT GREEN SELECTION 4 PASS\n";
}
'''
    run('both_colours',selection)
    code=geometry_fixture().split('int main()')[0].replace('anchor.heading_deg=180;','anchor.heading_deg=0;')
    code+=r'''
uint8_t parkingCcwRefinePhase=0;bool parkingCcwRefineUsed=false,parkingCcwViewAlternate=false,parkingCcwRefineRecovery=true;
uint32_t parkingCcwRefineMs=0,parkingEntryObserveStartMs=0,clockMs=1000;
float parkingCcwRefineEncoder=0,parkingCcwRefineTravel=0,encoder=0;
int8_t parkingCcwRefineDirection=1,parkingCcwRefineSteering=-50;
bool parkingEntryPathFailed=false,parkingEntryTestHold=false,servo_disabled=true;
int command=0,steeringCommand=0;
uint32_t millis(){return clockMs;}float get_distance(){return encoder;}
void stop(bool){command=0;servo_disabled=true;}
void set_speed(int c){command=c;}void set_steering(int s){steeringCommand=s;}
void steer(int s){if(!servo_disabled)steeringCommand=s;}
'''
    for signature in ('void seatCameraGeometry(', 'bool seatComfortablyVisible(',
                      'bool preflightCcwFrontView(', 'float ccwFrontViewExtensionMm(',
                      'float cwScoutViewExtensionMm(', 'bool updateCcwFrontViewRefinement('):
        code+=block(path,path.rindex(signature))+'\n'
    code+=r'''
enum ParkingEntryScoutPhase {PARKING_ENTRY_SCOUT_STEER_SETTLE,PARKING_ENTRY_SCOUT_REVERSE,
 PARKING_ENTRY_SCOUT_OBSERVE,PARKING_ENTRY_SCOUT_RETURN_BRAKE,PARKING_ENTRY_SCOUT_FORWARD_RETURN,
 PARKING_ENTRY_SCOUT_COMPLETE_BRAKE};
ParkingEntryScoutPhase parkingEntryScoutPhase=PARKING_ENTRY_SCOUT_OBSERVE;
bool parkingEntryScouting=true,parkingEntryPrimaryRetryUsed=false;
int parkingEntryScoutStation=0,discoveryScanSide=0,newScoutViews=0;
float parkingEntryScoutStartEncoderDistance=0,parkingEntryScoutReturnTravelMm=0,parkingEntryScoutTargetMm=160;
uint32_t parkingEntryScoutPhaseStartMs=0;PositionEstimate parkingEntryScoutReturnPose,scoutPose;
PositionEstimate get_position_struct(){return scoutPose;}
struct Blob{};struct Observation{};using ObstacleObservationResult=Observation;
Observation lastDiscoveryObservation;
const Blob *getDiscoveryObstacle(){return nullptr;}const Blob *getLargestObstacle(){return nullptr;}
Observation obstacle_path_observe(const Blob *){return {};}
void processGreenSeatCandidates(const PositionEstimate &,const Observation &){}
void updateDiscoveryCoverage(const Observation &,const PositionEstimate &,const Blob *){}
bool stationResolved(uint8_t){return false;}
void restoreParkingEntryPrimaryScanTarget(){}
void armParkingEntryConnectorFromPose(const PositionEstimate &){}
void startParkingEntryPrimaryRetry(){}
void startParkingEntryScout(const PositionEstimate &){++newScoutViews;}
'''+block(path,path.index('void updateParkingEntryScout('))+'\n'
    code+=r'''
int main(){int checks=0;
 routeTurnSign=1;parkingCwShortStart=false;parkingCcwShortStart=true;parkingEntryTargetStation=2;baseline();
 //598: green confirmed while the low-speed correction is still travelling.
 for(bool resolved:{false,true}){
  parkingCcwRefinePhase=2;parkingCcwRefineTravel=25;parkingCcwRefineEncoder=0;
  parkingCcwRefineDirection=1;parkingCcwRefineSteering=-50;parkingCcwRefineMs=1000;
  parkingEntryTestHold=false;encoder=15;clockMs=4000;
  if(!updateCcwFrontViewRefinement(PositionEstimate{281.6f,-1230.3f,43.33f},resolved)||parkingEntryTestHold)return 1;
  if(resolved ? (command!=0||parkingCcwRefinePhase!=3):(command!=60||parkingCcwRefinePhase!=2))return 2;
  ++checks;
 }
 //599: after the first timed observation, a real new checked view is possible.
 for(auto p:{PositionEstimate{261.6f,-1213.7f,44.1f},PositionEstimate{286.9f,-1224.9f,47.3f}}){
  float length=ccwFrontViewExtensionMm(p);
  if(length<=0||!preflightCcwFrontView(p,length,parkingCcwRefineDirection,parkingCcwRefineSteering))return 3;
  ++checks;
 }
 //595: -16deg was formerly considered good enough although GREEN was unknown.
 routeTurnSign=-1;parkingCwShortStart=true;parkingCcwShortStart=false;
 // Rebuild the actual mirrored start geometry rather than reflecting bearings.
 for(int st=0;st<3;++st)for(int side=0;side<2;++side){
  seats[2*st+side].x=500-500*st;seats[2*st+side].y=side==0?-900:-1100;
 }
 auto p=PositionEstimate{361,-1320,83.3f};
 float length=cwScoutViewExtensionMm(p,0,131.4f);
 if(length<=0||!preflightCwStartArc(p,length))return 4;
 ++checks;
 //595: exhausted observation returns along its checked arc and retries.
 scoutPose=p;parkingEntryScoutReturnPose=p;encoder=-160;clockMs=10000;
 parkingEntryTestHold=false;parkingEntryScoutPhase=PARKING_ENTRY_SCOUT_OBSERVE;
 updateParkingEntryScout(false);
 if(parkingEntryTestHold||!parkingEntryScouting||parkingEntryScoutPhase!=PARKING_ENTRY_SCOUT_RETURN_BRAKE)return 5;
 clockMs+=500;updateParkingEntryScout(false);
 if(parkingEntryScoutPhase!=PARKING_ENTRY_SCOUT_FORWARD_RETURN)return 6;
 encoder+=160;updateParkingEntryScout(false);clockMs+=500;updateParkingEntryScout(false);
 if(newScoutViews!=1||parkingEntryTestHold)return 7;
 checks+=3;
 std::cout<<"RECORDED START CONTROL "<<checks<<" PASS\n";
}
'''
    run('start_control',code)


if __name__=='__main__': main()
