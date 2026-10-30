"""Logged CCW view anchors and production CAD recovery/refinement.

Ideal encoder/CAD motion is a model, not a replay of unavailable run images.
"""
import shutil
import subprocess
from check_cw_start_planner import ROOT, fixture_source
from check_connector_servo_resume import block

DEST = ROOT / 'local_workspace/ccw-572-575'


def main():
    path = (ROOT / 'src/obstacle_path.cpp').read_text()
    code = fixture_source().split('int main()')[0]
    code = code.replace('anchor.heading_deg=180;', 'anchor.heading_deg=0;')
    code += r'''
uint32_t hostTime=0,parkingEntryObserveStartMs=0,parkingCcwRefineMs=0;
uint8_t parkingCcwRefinePhase=0;
bool parkingCcwRefineUsed=false,parkingEntryPathFailed=false,parkingEntryTestHold=false;
float parkingCcwRefineEncoder=0,parkingCcwRefineTravel=0,encoder=0;
bool servo_disabled=true;int command=0,steeringCommand=0;
uint32_t millis(){return hostTime;}float get_distance(){return encoder;}
void stop(bool){command=0;servo_disabled=true;}
void set_steering(int s){steeringCommand=s;}void steer(int s){steeringCommand=s;}
void set_speed(int s){command=s;}
bool cornerViewSeatMayBeOccupied(uint8_t seat){return !seats[seat^1].confirmed;}
'''
    for sig in ('void seatCameraGeometry(', 'bool seatComfortablyVisible(',
                'bool preflightCcwFrontView(', 'float ccwFrontViewExtensionMm(', 'bool updateCcwFrontViewRefinement(',
                'PositionEstimate cornerViewArcPose(', 'bool cornerViewArcSafe('):
        code += block(path, path.rindex(sig)) + '\n'
    code += r'''
int main(){
 routeTurnSign=1;parkingCwShortStart=false;parkingCcwShortStart=true;
 parkingCcwFirstEdgeStart=true;parkingEntryTargetStation=2;baseline();
 int views=0,states=0;float minimum=100,maximum=0;
 for(auto anchor:{PositionEstimate{253.1f,-1227.2f,37.2f},
                  PositionEstimate{185.5f,-1220.1f,34.3f},
                  PositionEstimate{189.1f,-1215.5f,31.5f}})
 for(float dx:{-5.f,0.f,5.f})for(float dy:{-5.f,0.f,5.f})for(float dh:{-2.f,0.f,2.f}){
  PositionEstimate pose=anchor;pose.x_mm+=dx;pose.y_mm+=dy;pose.heading_deg+=dh;
  float extra=ccwFrontViewExtensionMm(pose);
  if(extra<=0||extra>35||!preflightCcwFrontView(pose,extra)){
   std::cout<<"VIEW FAIL "<<pose.x_mm<<" "<<pose.y_mm<<" "<<pose.heading_deg<<" extra "<<extra<<"\n";
   for(float e:{5.f,10.f,15.f,20.f,25.f,30.f,35.f}){
    float k=-1.f/OBSTACLE_PARKING_ENTRY_SCAN_RADIUS_MM,h=pose.heading_deg*PI/180,n=h-k*e;
    auto p=pose;p.x_mm+=(sinf(n)-sinf(h))/k;p.y_mm+=(cosf(h)-cosf(n))/k;p.heading_deg=n*180/PI;
    float b,r;seatCameraGeometry(5,p,b,r);std::cout<<e<<" bearing/range "<<b<<"/"<<r<<" safe "<<preflightCwStartArc(pose,e)<<" footprint "<<parkingStartFootprintSafe(p.x_mm,p.y_mm,p.heading_deg,1,5)<<" opposite "<<parkingStartFootprintSafe(p.x_mm,p.y_mm,p.heading_deg,-1,5)<<" visible "<<seatComfortablyVisible(5,p)<<"\n";
   }
   for(float e:{5.f,10.f,15.f,20.f,25.f,30.f,35.f}){
    float k=pursuitCurvatureForServo(-50),h=pose.heading_deg*PI/180,n=h+k*e;
    auto p=pose;p.x_mm+=(sinf(n)-sinf(h))/k;p.y_mm+=(cosf(h)-cosf(n))/k;p.heading_deg=n*180/PI;
    float b,r;seatCameraGeometry(5,p,b,r);std::cout<<"FORWARD "<<e<<" bearing/range "<<b<<"/"<<r<<" footprint "<<parkingStartFootprintSafe(p.x_mm,p.y_mm,p.heading_deg,-1,5)<<" opposite "<<parkingStartFootprintSafe(p.x_mm,p.y_mm,p.heading_deg,1,5)<<"\n";
   }return 1;}
  minimum=fminf(minimum,extra);maximum=fmaxf(maximum,extra);++views;
  parkingCcwRefineUsed=false;parkingCcwRefinePhase=0;parkingEntryTestHold=false;
  parkingEntryObserveStartMs=0;hostTime=OBSTACLE_PARKING_ENTRY_OBSERVE_MS;encoder=0;stop(false);
  bool completed=false;
  for(int tick=0;tick<400;++tick){
   bool active=updateCcwFrontViewRefinement(pose,false);
   if(parkingEntryTestHold)return 2;
   if(!active&&parkingCcwRefineUsed){completed=true;break;}
   if(command){
    if(servo_disabled||steeringCommand!=-50||command<=0)return 3;
    float ds=command*.02f;encoder+=ds;
    // This uses the independently shared CAD mapping consumed by normal pursuit.
    const float k=pursuitCurvatureForServo(steeringCommand);
    float h=pose.heading_deg*PI/180,next=h+k*ds;
    pose.x_mm+=(sinf(next)-sinf(h))/k;pose.y_mm+=(cosf(h)-cosf(next))/k;
    pose.heading_deg=next*180/PI;
    if(!parkingStartFootprintSafe(pose.x_mm,pose.y_mm,pose.heading_deg,-1,5))return 4;
   }hostTime+=20;
  }
  float bearing,range;seatCameraGeometry(5,pose,bearing,range);
  if(!completed||fabsf(bearing)>13||!seatComfortablyVisible(5,pose)||command)return 5;
  if(updateCcwFrontViewRefinement(pose,false))return 6; // bounded once
  for(bool red:{false,true}){
   memcpy(livePath,baselinePath,sizeof(PathPoint)*pathLength);
   seats[5].confirmed=true;seats[5].injected=true;seats[5].red=red;
   displaceForSeat(livePath,5,OBSTACLE_LAP1_CLEARANCE_MM,!red);
   uint16_t merge=0;
   if(!buildParkingEntryConnector(pose,livePath,merge)){
    std::cout<<"REFINED JOIN FAIL "<<pose.x_mm<<" "<<pose.y_mm<<" "<<pose.heading_deg<<" red "<<red<<"\n";return 16;
   }
  }
  seats[5].confirmed=seats[5].injected=false;
  ++states;
 }
 parkingCcwRefineUsed=false;parkingCcwRefinePhase=0;
 if(updateCcwFrontViewRefinement(PositionEstimate{253.1f,-1227.2f,37.2f},true))return 7;
 hostTime=parkingEntryObserveStartMs+100;
 if(updateCcwFrontViewRefinement(PositionEstimate{253.1f,-1227.2f,37.2f},false)||parkingCcwRefineUsed)return 14;
 if(preflightCcwFrontView(PositionEstimate{400,-1450,0},25)||
    preflightCcwFrontView(PositionEstimate{253.1f,-1227.2f,37.2f},36))return 15;
 parkingCcwShortStart=false;
 if(ccwFrontViewExtensionMm(PositionEstimate{253.1f,-1227.2f,37.2f})!=0)return 8;
 std::cout<<"CCW "<<views<<" perturbed view plans and "<<states<<" state sequences PASS; extra "<<minimum<<".."<<maximum<<"mm; resolved/one-shot/direction guards PASS\n";
 //572: 76.7mm after extra_origin. The gyro had turned11.0deg, not the
 //16.0deg predicted from servo-as-wheel with the physical100mm wheelbase.
 PositionEstimate origin{423.1f,1005.3f,171.0f};
 auto predicted=cornerViewArcPose(origin,-20,-76.7f);
 const float loggedHeading=160.0f;
 float current=fabsf(wrap180(loggedHeading-predicted.heading_deg));
 float old=fabsf(wrap180(loggedHeading-(171.f-tanf(20.f*PI/180)/OBSTACLE_WHEELBASE_MM*76.7f*180/PI)));
 // Archived poses/encoder are rounded; the old error lands at the5deg gate.
 if(current>=2||old<4.9f)return 9;
 std::cout<<"572 recorded-heading comparison CAD error "<<current<<"deg versus old "<<old<<"deg PASS\n";
 seats[5].confirmed=seats[6].confirmed=seats[11].confirmed=true;
 seats[12].confirmed=seats[13].confirmed=true; // independently CLEAR station6
 int arcs=0;
 for(float dx:{-5.f,0.f,5.f})for(float dy:{-5.f,0.f,5.f})for(float dh:{-2.f,0.f,2.f}){
  auto p=origin;p.x_mm+=dx;p.y_mm+=dy;p.heading_deg+=dh;
  if(!cornerViewArcSafe(p,-20,-110)){std::cout<<"ARC UNSAFE "<<dx<<" "<<dy<<" "<<dh<<"\n";return 10;}
  auto end=cornerViewArcPose(p,-20,-90);
  if(!seatComfortablyVisible(14,end))return 11;
  if(!cornerViewArcSafe(end,-20,110))return 12;
  auto back=cornerViewArcPose(end,-20,90);
  if(hypotf(p.x_mm-back.x_mm,p.y_mm-back.y_mm)>.01f||fabsf(wrap180(p.heading_deg-back.heading_deg))>.01f)return 13;
  ++arcs;
 }
 std::cout<<"572 "<<arcs<<" perturbed CAD sweeps, view and retrace PASS\n";
}
'''
    DEST.mkdir(parents=True, exist_ok=True)
    cpp, exe = DEST/'check.cpp', DEST/'check.exe'
    cpp.write_text(code)
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'include'),
                    '-I', str(ROOT/'local_workspace/later-laps'), str(cpp), '-o', str(exe)], check=True)
    r = subprocess.run([str(exe)], capture_output=True, text=True, timeout=120)
    (DEST/'check.txt').write_text(r.stdout+r.stderr)
    print(r.stdout, end='')
    assert r.returncode == 0, r.returncode


if __name__ == '__main__':
    main()
