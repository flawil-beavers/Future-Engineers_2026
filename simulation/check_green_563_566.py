"""Source-backed regression for archived CW563--566; no hardware access.
Reconstructed blobs and ideal camera/plant models are not original-frame replay.
"""
import math
from check_connector_servo_resume import ROOT, block
from check_far_obstacle_acquisition import fixture_source
from check_later_laps import later_lap_fixture
from check_cw_556_558 import run
from check_final_parking_sequence import fixture_source as parking_fixture


def main():
    rows=[]
    for n in (563,566):
        for line in (ROOT/f'simulation/evidence/parking_exit_diagnostics/20261027_log_{n}_cw.txt').read_text().splitlines():
            if '[GREEN SEAT PRIMARY]' not in line:continue
            f=dict(t.split('=',1) for t in line.split() if '=' in t)
            x,y,h=map(float,f['pose'].split(','))
            if not -400<x<-200 or y>-650:continue
            b,r=map(float,f['bearing/range'].split('/'))
            cx=round(164.4-248.9*math.tan(math.radians(b)))
            foot=int(f['image_x/foot'].split('/')[1])
            rows.append((x,y,h,cx,foot))
    source=fixture_source()
    code=source+r'''
int main(){float x,y,h;int cx,foot,total=0;
 while(std::cin>>x>>y>>h>>cx>>foot){
  for(auto &s:seats){s={};s.x=s.y=9000;}
  seats[6].x=-900;seats[7].x=-1100;seats[6].y=seats[7].y=-500;
  seats[6].headingDeg=seats[7].headingDeg=90;
  seats[6].pathDistanceMm=seats[7].pathDistanceMm=775;
  hostPose={x,y,h};clockMs+=1000;
  Blob b{};b.found=true;b.color=ColorType::GREEN;b.centerX=cx;
  b.minX=cx-10;b.maxX=cx+10;b.minY=80;b.maxY=foot;b.area=650;
  obstacle_path_observe(&b);clockMs+=76;auto r=obstacle_path_observe(&b);
  if(seats[6].confirmed){std::cout<<"WRONG INNER ROW "<<x<<"\n";return 1;}
  ++total;
 }
 std::cout<<"EARLY DISPUTED ROWS "<<total<<" PASS; no wrong seat6 confirmation\n";
}
'''
    data='\n'.join(' '.join(map(str,r)) for r in rows)+'\n'
    run('563_566_rows',code,data)
    run('563_566_without_deferral',code.replace('if (bearingSeat == -2 &&',
        'if (false &&'),data,expected=1)

    path=(ROOT/'src/obstacle_path.cpp').read_text()
    geo=later_lap_fixture().split('int main()')[0]
    geo=geo.replace('struct Discovery {bool observedClear=false;};',
        'struct Discovery {bool observedClear=false;bool seatObservedClear[2]={};};')
    geo+=r'''
#include "obstacle_blob_validation.h"
Blob edgeRaw{};
const Blob *getLargestObstacle(){return &edgeRaw;}
bool obstacle_blob_valid_for_acquisition(const Blob *b){return obstacle_blob_shape_valid(b);}
float obstacle_camera_bearing_deg(const Blob *b){return atanf((164.4f-b->centerX)/248.9f)*180/PI;}
struct Observation {bool productionValid=false;ColorType color=ColorType::NONE;
 int seatId=-1;float rangeMm=0,bearingDeg=0;}lastDiscoveryObservation;
int discoveryScanStation=-1,discoveryScanSide=-1;
using DiscoveryStation=Discovery;
bool extremeAdjacentReleasePending=false;int lastConfirmedSeatIndex=-1;
bool earlyMiddleViewActive[OBSTACLE_SEAT_COUNT]={};
float lastDiscoveryTargetNudgeDeg=0;unsigned long lastDiscoveryNudgeUpdateMs=0,clockMs=1000;
unsigned long millis(){return clockMs;}
bool discoveryAimCommandSafe(const PositionEstimate &,int);
'''
    for sig in ('bool earlyMiddleViewEligible(', 'bool earlyMiddleViewPathSafe(',
                'void rebuildLivePath()', 'bool completingExtremeAdjacentPair(',
                'bool cornerViewSeatMayBeOccupied(', 'bool cornerViewSweepSafe(',
                'bool discoveryAimCommandSafe(', 'void applyDiscoveryTargetNudge('):
        geo+=block(path,path.rindex(sig))+'\n'
    code=geo+r'''
int main(){int count=0;float minGap=1e9,worstConfirm=1e9;
 struct Seed{int dir;float x,y,h,scale,bias;};
 const Seed seeds[]={{-1,912.4f,268.4f,288.84f,.75f,0},
                     {1,-984.f,-32.f,273.8f,.85f,-3.2f}};
 for(const auto &seed:seeds)for(float dx:{-10.f,0.f,10.f})
 for(float dy:{-10.f,0.f,10.f})for(float dh:{-1.f,0.f,1.f}){
  routeTurnSign=seed.dir;resetState();baseline();
  for(auto &s:seats)s.confirmed=s.injected=false;
  for(auto &d:discoveryStations){d.observedClear=true;d.seatObservedClear[0]=d.seatObservedClear[1]=false;}
  discoveryStations[11].observedClear=false;
  seats[19].confirmed=seats[19].injected=seats[19].red=true;rebuildLivePath();
  PositionEstimate p{seed.x+dx,seed.y+dy,seed.h+dh};
  progressIndex=nearestPathIndex(livePath,p.x_mm,p.y_mm,0,pathLength);
  lastDiscoveryTargetNudgeDeg=0;lastDiscoveryNudgeUpdateMs=clockMs;
  edgeRaw={};lastDiscoveryObservation={};int votes=0;bool passed=false;
  unsigned long nextFrameMs=clockMs;
  for(int step=0;step<900;++step){
   progressIndex=nearestPathIndex(livePath,p.x_mm,p.y_mm,progressIndex,OBSTACLE_PATH_PROGRESS_WINDOW);
   float h=p.heading_deg*PI/180,cx=p.x_mm+125*cosf(h),cy=p.y_mm+125*sinf(h);
   float range=hypotf(seats[22].x-cx,seats[22].y-cy);
   float bearing=wrap180(atan2f(seats[22].y-cy,seats[22].x-cx)*180/PI-p.heading_deg);
   if(clockMs>=nextFrameMs){
    nextFrameMs=clockMs+80;
    edgeRaw={};edgeRaw.found=true;edgeRaw.color=ColorType::GREEN;
    const float imageBearing=bearing+seed.bias;
    edgeRaw.centerX=int(lroundf(164.4f-248.9f*tanf(imageBearing*PI/180)));
    edgeRaw.minX=edgeRaw.centerX-28;edgeRaw.maxX=edgeRaw.centerX+28;
    edgeRaw.minY=80;edgeRaw.maxY=int(52+33600/(seed.scale*range*cosf(imageBearing*PI/180)));edgeRaw.area=1000;
    lastDiscoveryObservation={};
    if(obstacle_blob_valid_for_acquisition(&edgeRaw))
     lastDiscoveryObservation={true,ColorType::GREEN,-1,
      (edgeRaw.maxX>=317||edgeRaw.minX<=2)?190.f:seed.scale*range,imageBearing};
    // Ideal fresh camera components become geometrically snappable at550mm.
    const float projectedX=cx+seed.scale*range*cosf(h+imageBearing*PI/180);
    const float projectedY=cy+seed.scale*range*sinf(h+imageBearing*PI/180);
    if(!seats[22].confirmed && range<=550 &&
       hypotf(projectedX-seats[22].x,projectedY-seats[22].y)<=140 &&
       edgeRaw.maxX<317 && edgeRaw.minX>2 &&
       obstacle_blob_valid_for_acquisition(&edgeRaw)){
      if(++votes>=2){worstConfirm=fminf(worstConfirm,range);
       seats[22].confirmed=seats[22].injected=true;seats[22].red=false;rebuildLivePath();}
    }else votes=0;
   }
   for(int seat:{19,22}){ObstacleClearanceSample c{};
    calculateClearanceAtPose(seats[seat],p.x_mm,p.y_mm,p.heading_deg,c);
    minGap=fminf(minGap,fminf(c.wallMm,c.pillarMm));
    if(c.wallMm<30||c.pillarMm<30){std::cout<<"VIEW ROUTE COLLISION "<<seed.dir<<" "<<dx<<" "<<dy<<" "<<dh<<" seat "<<seat<<" step "<<step<<" pose "<<p.x_mm<<","<<p.y_mm<<","<<p.heading_deg<<" gap "<<c.pillarMm<<" known "<<seats[22].confirmed<<"\n";return 2;}}
   if(p.y_mm<-680){passed=true;break;}
   float look=firstLapLookahead(livePath[progressIndex].speedMmS);
   auto target=findLookahead(livePath,p,look);
   if(!seats[22].confirmed)applyDiscoveryTargetNudge(target,p);
   float tx=target.x-p.x_mm,ty=target.y-p.y_mm;
   float k=2*(-tx*sinf(h)+ty*cosf(h))/fmaxf(1,tx*tx+ty*ty);
   int servo=int(clampFloat(pursuitServoForCurvature(k),-42,42));
   k=abs(servo)<2?0:-1.f/Ackermann::getTurnRadius(servo);float next=h+2*k;
   if(fabsf(k)>1e-6){p.x_mm+=(sinf(next)-sinf(h))/k;p.y_mm+=(cosf(h)-cosf(next))/k;}
   else{p.x_mm+=2*cosf(h);p.y_mm+=2*sinf(h);}p.heading_deg=wrap180(next*180/PI);
   // Keep the production100mm/s unknown-station cap inside750mm.
   const float forward=cyclicDistanceForward(baselinePath[progressIndex].distanceMm,seats[22].pathDistanceMm);
   if(!seats[22].confirmed && forward<=OBSTACLE_DISCOVERY_HOLD_DISTANCE_MM){
    std::cout<<"VIEW TOO LATE FOR EXISTING HOLD "<<seed.dir<<"\n";return 8;}
   clockMs+=!seats[22].confirmed && forward<=OBSTACLE_DISCOVERY_SLOW_DISTANCE_MM ? 20:10;
  }
  if(!passed||!seats[22].confirmed){std::cout<<"NO EARLY VIEW\n";return 3;}++count;
 }
 std::cout<<"THIRD GREEN CW/CCW VIEW/ROUTE "<<count<<" PASS; min reserve "<<minGap<<" latest confirmation range "<<worstConfirm<<"\n";
}
'''
    run('564_view_routes',code)

    # Couple production projection/voting to the actual unknown first-corner
    # path. Earlier tests seeded GREEN as already known; this one must earn it.
    header=(ROOT/'include/obstacle_path.h').read_text()
    first=geo.replace('struct Observation {bool productionValid=false;ColorType color=ColorType::NONE;\n int seatId=-1;float rangeMm=0,bearingDeg=0;}lastDiscoveryObservation;',
        block(header,header.index('enum ObstacleObservationStatus'))+';\n'+
        block(header,header.index('struct ObstacleObservationResult'))+';\n'+
        'ObstacleObservationResult lastDiscoveryObservation;')
    camera=(ROOT/'src/obstacle_vision.cpp').read_text()
    first=first.replace('float obstacle_camera_bearing_deg(const Blob *b){return atanf((164.4f-b->centerX)/248.9f)*180/PI;}',
        block(camera,camera.index('float obstacle_camera_bearing_deg(')))
    first+=r'''
PositionEstimate hostPose;
PositionEstimate get_position_struct(){return hostPose;}
bool parkingSectionInnerSeatsOnly=false;
int injectionCount=0;
bool oppositeSeatConflictReported[OBSTACLE_SEAT_COUNT]={};
int stationIndexForSeat(uint8_t i){return i/2;}
bool mappedParkingSeatFootMatches(uint8_t,const PositionEstimate &,int){return true;}
void logRedSeatProjection(const ObstacleObservationResult &,const char *,bool=false){}
void finalizeConfirmedSeat(ObstacleObservationResult &r){
 r.status=OBSTACLE_OBSERVATION_CONFIRMED;seats[r.seatId].injected=true;++injectionCount;rebuildLivePath();}
'''
    for sig in ('float obstacle_estimate_camera_forward_mm(', 'float obstacle_estimate_camera_range_mm('):
        first+=block(camera,camera.index(sig))+'\n'
    for sig in ('bool traversingInjectedAvoidance(', 'int nearestSeatIndex(',
                'bool recordSeatVote(', 'void prepareConsecutiveVote(', 'void expirePendingVotes(',
                'bool distantSeatSideAmbiguous(', 'int resolveSeatRowByBearing(',
                'bool officialMiddleConflictsWithEnd(', 'int resolveCcwReturnRedByBearing(',
                'ObstacleObservationResult obstacle_path_observe('):
        first+=block(path,path.index(sig))+'\n'
    first+=r'''
int main(){int count=0;float minGap=1e9;
 for(float dx:{-5.f,0.f,5.f})for(float dy:{-5.f,0.f,5.f})for(float dh:{-1.f,0.f,1.f}){
  routeTurnSign=-1;resetState();baseline();
  for(auto &s:seats){s.confirmed=s.injected=false;s.redVotes=s.greenVotes=0;}
  for(auto &d:discoveryStations){d.observedClear=true;d.seatObservedClear[0]=d.seatObservedClear[1]=false;}
  discoveryStations[3].observedClear=false;
  seats[4].confirmed=seats[4].injected=seats[4].red=true;rebuildLivePath();
  hostPose={-239.f+dx,-882.f+dy,144.3f+dh};clockMs+=1000;
  lastDiscoveryTargetNudgeDeg=0;lastDiscoveryNudgeUpdateMs=clockMs;
  lastDiscoveryObservation={};edgeRaw={};unsigned long nextFrame=clockMs;
  progressIndex=nearestPathIndex(livePath,hostPose.x_mm,hostPose.y_mm,0,pathLength);
  bool passed=false;
  for(int step=0;step<1500;++step){
   auto &p=hostPose;float h=p.heading_deg*PI/180;
   progressIndex=nearestPathIndex(livePath,p.x_mm,p.y_mm,progressIndex,OBSTACLE_PATH_PROGRESS_WINDOW);
   if(clockMs>=nextFrame){
    nextFrame=clockMs+80;
    float cx=p.x_mm+125*cosf(h),cy=p.y_mm+125*sinf(h);
    float range=hypotf(seats[7].x-cx,seats[7].y-cy);
    float b=wrap180(atan2f(seats[7].y-cy,seats[7].x-cx)*180/PI-p.heading_deg)-2.4f;
    edgeRaw={};edgeRaw.found=true;edgeRaw.color=ColorType::GREEN;
    edgeRaw.centerX=int(lroundf(164.4f-248.9f*tanf(b*PI/180)));
    edgeRaw.minX=edgeRaw.centerX-10;edgeRaw.maxX=edgeRaw.centerX+10;
    edgeRaw.maxY=int(lroundf(52+33600/(.65f*range*cosf(b*PI/180))));
    edgeRaw.minY=80;edgeRaw.area=650;
    lastDiscoveryObservation=obstacle_path_observe(&edgeRaw);
    if(seats[6].confirmed){std::cout<<"FIRST VIEW WRONG ROW\n";return 9;}
   }
   for(int seat:{4,7}){ObstacleClearanceSample c{};
    calculateClearanceAtPose(seats[seat],p.x_mm,p.y_mm,p.heading_deg,c);
    minGap=fminf(minGap,fminf(c.wallMm,c.pillarMm));
    if(c.wallMm<30||c.pillarMm<30){std::cout<<"FIRST VIEW COLLISION "<<step<<" "<<c.pillarMm<<"\n";return 10;}}
   if(p.y_mm>-200){passed=true;break;}
   float forward=cyclicDistanceForward(baselinePath[progressIndex].distanceMm,seats[7].pathDistanceMm);
   if(!seats[7].confirmed && forward<=OBSTACLE_DISCOVERY_HOLD_DISTANCE_MM){
    std::cout<<"FIRST VIEW TOO LATE "<<dx<<" "<<dy<<" "<<dh<<" pose "<<p.x_mm<<","<<p.y_mm<<","<<p.heading_deg<<"\n";return 11;}
   float look=firstLapLookahead(livePath[progressIndex].speedMmS);
   if(nearCorner(baselinePath[progressIndex].distanceMm))look*=OBSTACLE_LOOKAHEAD_CORNER_SCALE;
   auto target=findLookahead(livePath,p,look);
   const bool traversing=traversingInjectedAvoidance(baselinePath[progressIndex].distanceMm);
   if(!traversing)applyDiscoveryTargetNudge(target,p);
   else{lastDiscoveryTargetNudgeDeg=0;lastDiscoveryNudgeUpdateMs=clockMs;}
   float tx=target.x-p.x_mm,ty=target.y-p.y_mm;
   float k=2*(-tx*sinf(h)+ty*cosf(h))/fmaxf(1,tx*tx+ty*ty);
   int servo=int(clampFloat(pursuitServoForCurvature(k),-42,42));
   k=abs(servo)<2?0:-1.f/Ackermann::getTurnRadius(servo);float next=h+2*k;
   if(fabsf(k)>1e-6){p.x_mm+=(sinf(next)-sinf(h))/k;p.y_mm+=(cosf(h)-cosf(next))/k;}
   else{p.x_mm+=2*cosf(h);p.y_mm+=2*sinf(h);}p.heading_deg=wrap180(next*180/PI);
   clockMs+=!seats[7].confirmed && forward<=OBSTACLE_DISCOVERY_SLOW_DISTANCE_MM ? 20:8;
  }
  if(!passed||!seats[7].confirmed)return 12;++count;
 }
 std::cout<<"FIRST GREEN UNKNOWN-TO-CONFIRMED "<<count<<" coupled production cases PASS; min reserve "<<minGap<<"\n";
}
'''
    run('563_first_unknown_route',first)

    parking=parking_fixture()+r'''
int main(){int count=0;
 for(int dir:{-1,1})for(float dy:{-5.f,0.f,5.f})for(float dh:{-1.f,0.f,1.f}){
  routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  parking_module::final_parking_reset();parking_module::routeTurnSign=dir;
  parking_module::outerSensor=dir<0?TOF_LEFT:TOF_RIGHT;
  parking_module::measuredGapMm=251.3f;parking_module::captureTargetX=209.74f;
  hostPose={226.6f,-1194.8f+dy,dir<0?182.f+dh:-2.f-dh};truePose=hostPose;
  encoder=0;hostTime=1000;stop(false);
  if(!parking_module::captureAlignmentPreflight(hostPose)){
   std::cout<<"ALIGN PREFLIGHT FAIL "<<dir<<" "<<dy<<" "<<dh<<"\n";return 4;}
  parking_module::captureAlignPending=true;parking_module::state=parking_module::FP_CAPTURE_SETTLE;
  parking_module::stateStartMs=hostTime-500;
  bool reached=false;
  for(int step=0;step<1800;++step){
   parking_module::final_parking_update(dir);
   if(parking_module::final_parking_aborted()){std::cout<<robot_logger.lines.str();return 5;}
   if(parking_module::state==parking_module::FP_SEGMENT_SETTLE){reached=true;break;}
   if(dc_state==DC_ENABLED){
    if(!parking_module::parkingEntryPoseSafe(truePose,steeringCommand))return 6;
    float ds=command*.02f;encoder+=ds;
    parking_module::advanceParkingPose(hostPose,steeringCommand,ds);
    parking_module::advanceParkingPose(truePose,steeringCommand,ds);
   }hostTime+=20;
  }
  if(!reached)return 7;++count;
 }
 std::cout<<"564 REGISTERED CAPTURE "<<count<<" perturbed/mirrored poses reach entry with original gates\n";
}
'''
    run('564_capture',parking)
    run('564_without_capture_alignment',parking.replace(
        'state = captureAlignPending ? FP_CAPTURE_ALIGN_DRIVE : FP_CAPTURE_DRIVE;',
        'state = FP_CAPTURE_DRIVE;'),expected=5)


if __name__=='__main__':main()
