"""Extract production map/route/lap functions; verify laps 2/3 without hardware.

Explicit layouts replace images. CAD Ackermann steering replaces the ideal bicycle plant.
ToF, servo delay and image recognition are tested separately or remain limitations.
Generated fixtures and results remain in local_workspace.
"""
import itertools
import json
import shutil
import subprocess

from check_cw_start_planner import ROOT, fixture_source
from check_connector_servo_resume import block


# Direction, green seats, red seats, time ceiling, capsule floor, start straight.
CORNER_LAYOUTS = (
    (1, (3, 15), (6, 10, 19), 14.85, 42, True),  # 5
    (1, (3, 15), (6, 10, 18), 15.65, 42, True),  # A
    (1, (3, 7, 14), (10, 20), 14.15, 50, True),  # B
    (1, (3, 15), (7, 10, 19), 14.15, 42, True),  # C
    (1, (1, 9, 17, 21), (5, 12), 15.40, 50, False),  # D
    (-1, (4, 7, 23), (10, 14), 14.70, 50, False),  # E
    (-1, (4,), (0, 9, 15, 21), 13.40, 50, False),  # F
    (-1, (6, 19), (11, 14, 22), 14.90, 50, False),  # G
    (-1, (9, 12), (2, 16, 19, 23), 13.95, 50, True),  # J
    (-1, (4, 17, 19, 22), (0, 11, 12), 15.90, 50, False),  # K
    (1, (3, 13, 17), (6, 11, 20), 14.65, 50, True),  # M
    (1, (8, 16, 23), (12, 19), 14.70, 50, False),  # N
    (-1, (7, 11, 14, 22), (2, 18), 15.60, 50, False),  # 3
    (-1, (0, 15, 22), (6, 11, 19), 14.05, 50, False),  # 7
    (1, (9, 15, 18, 23), (), 12.60, 50, False),  # 12
    (1, (7, 10, 14, 23), (5, 18), 14.70, 50, False),  # 13
)


def later_lap_fixture(source=None):
    if source is None:
        source = (ROOT/'src/obstacle_path.cpp').read_text()
    fixture = fixture_source().split('int main()')[0]
    fixture = fixture.replace(
        'bool targetsOuterExtreme(const CandidateSeat &seat) {return seat.red ? seat.lateralMm<0 : seat.lateralMm>0;}',
        'bool targetsOuterExtreme(const CandidateSeat &seat);')
    fixture = fixture.replace('uint16_t pathLength=0;', 'uint16_t pathLength=0,progressIndex=0;')
    extra = r'''
#include "obstacle_section_layout.h"
constexpr int COURSE_SECTION_COUNT=4,COURSE_STATIONS_PER_SECTION=3;
constexpr int OBSTACLE_SECTION_LAYOUT_OFFICIAL=0,OBSTACLE_SECTION_LAYOUT_CHECK_ALL=1;
int sectionLayoutMode=0;
bool parkingEntryActive=false,parkingEntryObserving=false,parkingEntryScouting=false;
bool parkingEntryJoining=false,parkingEntryTestHold=false,runtimeTestMode=false;
bool optimizedBuilt=false,lapBoundaryPending=false,lapCountingArmed=false,laterLapPlanRejected=false,lapBoundaryHoldLogged=false,lapFinishPending=false;
bool lapHandoffActive=false,lap1CommandValid=false;
float lap1LookaheadMm=0,lap1SpeedMmS=0,lapHandoffEncoderMm=0,lapHandoffTravelMm=0;
float simulatedEncoderMm=0;
float simulatedLap1SpeedMmS=260;
float get_distance(){return simulatedEncoderMm;}
uint8_t completedLaps=0,runtimeLapTarget=3;bool finished=false;
PathPoint optimizedPath[OBSTACLE_MAX_PATH_WAYPOINTS];
PathPoint parkingApproachPath[OBSTACLE_MAX_PATH_WAYPOINTS];
float laterLapSpeedLimits[OBSTACLE_MAX_PATH_WAYPOINTS]={};
void joinSameColourStraightEnds(PathPoint *,bool,bool mixedColours=false);
void roundKnownCornerPairs(PathPoint *, bool injectedOnly = false,bool mixedColours=false);
uint32_t micros(){return 0;} // unlimited offline planning; budget faults tested separately
bool plannedClearanceSnapshotValid[OBSTACLE_SEAT_COUNT]={};
ObstacleClearanceSample plannedClearanceAtInjection[OBSTACLE_SEAT_COUNT];
bool obstacle_path_get_planned_clearance(uint8_t,ObstacleClearanceSample&){return false;}
'''
    fixture += extra
    fixture += '\n#include "ackermann_kinematics.h"\n'
    signatures = (
        'bool parkingReturnMotionSafe(', 'float recordedLapSpeed(',
        'uint8_t sectionInferredEmpty(', 'bool stationDirectlyResolved(', 'bool stationResolved(', 'bool allStationsResolved(',
        'bool withinCornerGate(', 'bool nearCorner(', 'bool isExtremeAdjacentPair(',
        'bool targetsOuterExtreme(', 'bool hasConfirmedExtremeAdjacentPair(',
        'bool isSecondExtremeAdjacentSeat(', 'bool upcomingAdjacentStationUnresolved(',
        'float validatedClearanceForSeat(', 'float optimizedClearanceForSeat(',
        'bool optimizedUsesOuterPlateau(', 'bool laterLapMapValid()',
        'int ccwDiscoveryTallRedExitSeat(',
        'void preserveLaterLapSeam(', 'bool laterLapRouteSafe(',
        'void joinSameColourStraightEnds(PathPoint *route, bool injectedOnly, bool mixedColours)\n{',
        'void roundKnownCornerPairs(PathPoint *route, bool injectedOnly, bool mixedColours)\n{',
        'void carryKnownInnerLaneToMiddle(', 'float laterLapBending(', 'void smoothKnownLaterLapRoute(',
        'float laterLapCurvatureSpeed(', 'float laterLapTrackingSpeed(', 'void planLaterLapSpeeds(',
        *(['PathPoint laterLapLookaheadTarget('] if 'PathPoint laterLapLookaheadTarget(' in source else []),
        'bool laterLapTrackedRouteSafe(', 'float laterLapTargetTime(',
        *(['void connectKnownLaterLapRoute('] if 'void connectKnownLaterLapRoute(' in source else []),
        'void shortenKnownInnerRoute(',
        *(['void addLaterLapStartMiddleReserve('] if 'void addLaterLapStartMiddleReserve(' in source else []),
        *(['int solitaryMiddleSeat(',
           *(['bool isCwSolitaryOuterRed('] if 'bool isCwSolitaryOuterRed(' in source else []),
           'float smoothLaneBlend(', 'void refineCcwSolitaryPillars(']
          if 'void refineCcwSolitaryPillars(' in source else []),
        *(['void refineCwSolitaryRed('] if 'void refineCwSolitaryRed(' in source else []),
        *(['bool connectLaterLapPasses(',
           *(['bool tryLaterLapPassConnection('] if 'bool tryLaterLapPassConnection(' in source else []),
           'void refineLaterLapPassConnections(']
          if 'void refineLaterLapPassConnections(' in source else []),
        *(['void refineCcwParallelStart('] if 'void refineCcwParallelStart(' in source else []),
        *(['void refineLaterLapLongCorners('] if 'void refineLaterLapLongCorners(' in source else []),
        *(['void refineLaterLapCornerGeometry('] if 'void refineLaterLapCornerGeometry(' in source else []),
        *(['uint16_t laterLapWaypointAt(', 'bool tryLaterLapStraightPass(',
           'void refineLaterLapStartStraight(', 'void refineLaterLapOpenStart(',
           *(['void refineLaterLapEarlyStartExit(', 'void refineLaterLapInnerMiddleCorners(']
             if 'void refineLaterLapEarlyStartExit(' in source else []),
           'void refineLaterLapStartExit(', 'void refineCcwFirstSeatCorner(', 'void refineCcwGreenRedStraight(']
          if 'void refineLaterLapStartStraight(' in source else []),
        'bool buildOptimizedPath()', 'bool completePendingLap()', 'bool finalCornerVehicleClear(',
        'bool recordParkingFinish(', 'void updateProgress(',
        'PathPoint findLookahead(', 'float lapHandoffWeight()',
        'PathPoint lapHandoffTarget(', 'float lapHandoffSpeed(')
    fixture += '\n'.join(block(source,source.index(s)) for s in signatures)
    fixture += r'''
void resetState(){
 lapHandoffActive=lap1CommandValid=false;
 lap1LookaheadMm=lap1SpeedMmS=lapHandoffEncoderMm=lapHandoffTravelMm=simulatedEncoderMm=0;
 completedLaps=0;finished=false;optimizedBuilt=false;lapBoundaryPending=false;lapCountingArmed=false;
 laterLapPlanRejected=false;lapBoundaryHoldLogged=false;lapFinishPending=false;runtimeTestMode=false;
 runtimeLapTarget=3;progressIndex=0;sectionLayoutMode=0;
 for(auto &d:discoveryStations)d.observedClear=true;
}
void learned(){
 memcpy(livePath,baselinePath,sizeof(PathPoint)*pathLength);
 for(int i=0;i<OBSTACLE_SEAT_COUNT;++i)if(seats[i].confirmed){
  const float clearance=validatedClearanceForSeat(i);
  displaceForSeat(livePath,i,clearance,(routeTurnSign>0&&i==5&&!seats[i].red)||
    fabsf(clearance-OBSTACLE_OUTER_SAFE_CLEARANCE_MM)<.1f);
 }
}
float discoveryCornerMinimum(bool rounded){
 routeTurnSign=1;resetState();baseline();
 for(auto &seat:seats){seat.confirmed=false;seat.injected=false;}
 for(int i:{9,12,17,18}){seats[i].confirmed=seats[i].injected=true;
  seats[i].red=(i==9||i==12);}
 // Replay the old 260 mm discovery geometry from log 463. The current
 // compact lap-1 geometry is exercised separately by the full map matrix.
 memcpy(livePath,baselinePath,sizeof(PathPoint)*pathLength);
 for(int i:{9,12,17,18})displaceForSeat(livePath,i,OBSTACLE_LAP1_CLEARANCE_MM,false);
 if(rounded)roundKnownCornerPairs(livePath,true);
 // Last measured pose before seat18 confirmation in log463. Confirmation
 // itself was not pose-logged, so this is a nearby geometric replay only.
 float x=-519.4f,y=730.8f,h=194.49f*PI/180,minimum=1e9f;
 progressIndex=nearestPathIndex(baselinePath,x,y,0,pathLength);
 for(int step=0;step<1200;++step){
  progressIndex=nearestPathIndex(livePath,x,y,progressIndex,OBSTACLE_PATH_PROGRESS_WINDOW);
  PositionEstimate pose;pose.x_mm=x;pose.y_mm=y;pose.heading_deg=h*180/PI;
  ObstacleClearanceSample sample{};calculateClearanceAtPose(seats[18],x,y,pose.heading_deg,sample);
  minimum=fminf(minimum,sample.pillarMm);if(sample.wallMm<=0)return -1;
  if(y<300)return minimum;
  float look=firstLapLookahead(livePath[progressIndex].speedMmS);
  if(nearCorner(baselinePath[progressIndex].distanceMm))look*=OBSTACLE_LOOKAHEAD_CORNER_SCALE;
  auto target=findLookahead(livePath,pose,look);float dx=target.x-x,dy=target.y-y;
  float k=2*(-dx*sinf(h)+dy*cosf(h))/fmaxf(1,dx*dx+dy*dy);
  int steering=static_cast<int>(clampFloat(pursuitServoForCurvature(k),-42,42));
  k=pursuitCurvatureForServo(steering);
  float next=h+k*2;
  if(fabsf(k)>1e-6){x+=(sinf(next)-sinf(h))/k;y+=(cosf(h)-cosf(next))/k;}
  else{x+=2*cosf(h);y+=2*sinf(h);}h=next;
 }
 return -1;
}
bool stateChecks(){
 const float oldCorner=discoveryCornerMinimum(false),newCorner=discoveryCornerMinimum(true);
 if(oldCorner<=0||newCorner<90||newCorner<oldCorner+25)return false;
 std::cout<<"DISCOVERY corner old/new modeled pillar_mm="<<oldCorner<<"/"<<newCorner<<"\n";
 // CCW discovery pair: only active injections may round a corner. Stored
 // behind-start observations must not reshape the initial departure.
 routeTurnSign=1;resetState();baseline();
 for(auto &seat:seats){seat.confirmed=false;seat.injected=false;}
 seats[17].confirmed=seats[17].injected=true;seats[17].red=false;
 learned();seats[18].confirmed=true;seats[18].red=false;
 PathPoint original[OBSTACLE_MAX_PATH_WAYPOINTS];
 memcpy(original,livePath,sizeof(livePath));roundKnownCornerPairs(livePath,true);
 if(memcmp(original,livePath,sizeof(PathPoint)*pathLength))return false;
 seats[18].injected=true;displaceForSeat(livePath,18,260,true);
 memcpy(original,livePath,sizeof(livePath));roundKnownCornerPairs(livePath,true);
 if(!memcmp(original,livePath,sizeof(PathPoint)*pathLength))return false;
 routeTurnSign=-1;
 // Logs 458/459: connector joins the final route indices before the initial
 // phase-zero crossing. An unexplored map must NOT hold this start as lap 1.
 resetState();baseline();learned();
 for(auto &d:discoveryStations)d.observedClear=false;
 progressIndex=pathLength-2;
 PositionEstimate seamPose;seamPose.x_mm=livePath[0].x;seamPose.y_mm=livePath[0].y;
 updateProgress(livePath,seamPose);
 if(progressIndex!=0||lapBoundaryPending||completedLaps||lapCountingArmed)return false;
 // Actual mid-course traversal arms counting; the next wrap must still hold
 // when the map is incomplete, preserving the existing safety gate.
 progressIndex=pathLength/2;
 seamPose.x_mm=livePath[progressIndex].x;seamPose.y_mm=livePath[progressIndex].y;
 updateProgress(livePath,seamPose);
 if(!lapCountingArmed)return false;
 progressIndex=pathLength-2;seamPose.x_mm=livePath[0].x;seamPose.y_mm=livePath[0].y;
 updateProgress(livePath,seamPose);
 if(!lapBoundaryPending||completedLaps||lapCountingArmed)return false;
 resetState();
 if(recordedLapSpeed(260)!=260)return false;
 completedLaps=1;if(recordedLapSpeed(260)!=520)return false;
 completedLaps=2;if(recordedLapSpeed(180)!=360)return false;
 lapFinishPending=true;if(recordedLapSpeed(180)!=180)return false;
 lapFinishPending=false;runtimeTestMode=true;if(recordedLapSpeed(260)!=260)return false;
 resetState();baseline();learned();lapBoundaryPending=true;
 discoveryStations[7].observedClear=false;
 if(completePendingLap()||completedLaps||!lapBoundaryPending)return false;
 discoveryStations[7].observedClear=true;
 if(!completePendingLap()||completedLaps!=1||!optimizedBuilt)return false;
 if(completePendingLap()||completedLaps!=1)return false;
 lapBoundaryPending=true;if(!completePendingLap()||completedLaps!=2||finished)return false;
 lapBoundaryPending=true;if(!completePendingLap()||completedLaps!=3||finished||!lapFinishPending)return false;
 PositionEstimate finishPose;finishPose.x_mm=livePath[3].x;finishPose.y_mm=livePath[3].y;
 updateProgress(livePath,finishPose);
 if(!finished||lapFinishPending||completedLaps!=3)return false;
 // Final handover must use the complete vehicle and only the third lap.
 for(int dir:{-1,1}){
  routeTurnSign=dir;resetState();baseline();learned();
  if(!buildOptimizedPath())return false;
  int index=-1;
  for(int i=0;i<pathLength;++i)if(baselinePath[i].distanceMm>=corners[3].pathEndMm &&
    finalCornerVehicleClear(PositionEstimate{optimizedPath[i].x,optimizedPath[i].y,connectorRouteHeading(optimizedPath,i)})){index=i;break;}
  if(index<0)return false;
  PositionEstimate p{optimizedPath[index].x,optimizedPath[index].y,connectorRouteHeading(optimizedPath,index)};
  PositionEstimate cornerPose{dir<0?350.f:-350.f,-1000.f,dir<0?180.f:0.f};
  if(finalCornerVehicleClear(cornerPose))return false;
  completedLaps=1;lapCountingArmed=true;progressIndex=index;
  updateProgress(optimizedPath,p);if(finished||completedLaps!=1)return false;
  completedLaps=2;runtimeTestMode=true;progressIndex=index;
  updateProgress(optimizedPath,p);if(finished||completedLaps!=2)return false;
  runtimeTestMode=false;progressIndex=index;
  updateProgress(optimizedPath,p);if(!finished||completedLaps!=3||lapFinishPending)return false;
 }
 resetState();baseline();learned();runtimeLapTarget=1;lapBoundaryPending=true;
 if(!completePendingLap()||completedLaps!=1||!finished||optimizedBuilt)return false;
 resetState();baseline();
 seats[6].confirmed=seats[7].confirmed=true;
 if(laterLapMapValid())return false;
 seats[7].confirmed=false;seats[8].confirmed=true;
 if(laterLapMapValid())return false; // Official middle plus an end.
 sectionLayoutMode=1;if(!laterLapMapValid())return false;
 seats[10].confirmed=true;if(!laterLapMapValid())return false; // O3 three stations.
 resetState();baseline();learned();runtimeTestMode=true;
 discoveryStations[7].observedClear=false;lapBoundaryPending=true;
 if(!completePendingLap()||completedLaps!=1||optimizedBuilt)return false;
 return true;
}
int failReason=0,failSeat=-1;float failX=0,failY=0,failH=0;
bool driveTwoLaps(float speedCap,float &minimumWall,float &minimumPillar,
                  float *cwRedPassMaximumY=nullptr){
 failReason=1;failSeat=-1;
 minimumWall=minimumPillar=1e9f;
 const auto start=livePath[0];float x=start.x,y=start.y;
 float heading=connectorRouteHeading(livePath,0)*PI/180;
 lap1CommandValid=true;lap1SpeedMmS=speedCap>0?fminf(speedCap,simulatedLap1SpeedMmS):simulatedLap1SpeedMmS;
 lap1LookaheadMm=firstLapLookahead(lap1SpeedMmS);
 if(nearCorner(baselinePath[0].distanceMm))lap1LookaheadMm*=OBSTACLE_LOOKAHEAD_CORNER_SCALE;
 lapBoundaryPending=true;if(!completePendingLap())return false;
 uint8_t witnessed[24]={},passCounts[24]={};
 float previousForward[24];for(auto &f:previousForward)f=NAN;
 // Outer avoidance corners lengthen the route beyond two baseline laps.
 for(int step=0;step<5000;++step){
  failX=x;failY=y;failH=heading*180/PI;
  const PathPoint *route=optimizedBuilt?optimizedPath:livePath;
  PositionEstimate pose;pose.x_mm=x;pose.y_mm=y;pose.heading_deg=heading*180/PI;
  updateProgress(route,pose);
  if(cwRedPassMaximumY && fabsf(x-seats[15].x)<420.0f &&
     y>500.0f && completedLaps>=1 && completedLaps<=2)
   cwRedPassMaximumY[completedLaps-1]=fmaxf(cwRedPassMaximumY[completedLaps-1],y);
  if(finished){
   for(int seat=0;seat<24;++seat)if(seats[seat].confirmed&&passCounts[seat]<2){failReason=6;failSeat=seat;return false;}
   return completedLaps==3;
  }
  if(lapBoundaryPending||laterLapPlanRejected)return false;
  route=optimizedBuilt?optimizedPath:livePath;
  if(y < -1100 && (!parking_start_footprint::safe(x,y,heading*180/PI,-1,false)||
     !parking_start_footprint::safe(x,y,heading*180/PI,1,false))){
   failReason=5;return false;
  }
  parking_start_footprint::Quad footprints[2][6];
  parking_start_footprint::robotQuads(x,y,heading*180/PI,-1,0,footprints[0]);
  parking_start_footprint::robotQuads(x,y,heading*180/PI,1,0,footprints[1]);
  for(int seat=0;seat<OBSTACLE_SEAT_COUNT;++seat){
   if(seat && !seats[seat].confirmed)continue;
   ObstacleClearanceSample sample{};
   if(!calculateClearanceAtPose(seats[seat],x,y,heading*180/PI,sample))return false;
   minimumWall=fminf(minimumWall,sample.wallMm);
   if(seats[seat].confirmed)minimumPillar=fminf(minimumPillar,sample.pillarMm);
   if(sample.wallMm<=0||(seats[seat].confirmed&&sample.pillarMm<=0)){
    failReason=2;failSeat=seat;return false;
   }
   if(seats[seat].confirmed&&witnessed[seat]!=completedLaps){
    const float phase=cyclicDistanceForward(seats[seat].pathDistanceMm,
                                            baselinePath[progressIndex].distanceMm);
    {
     const float ref=seats[seat].headingDeg*PI/180;
     float minForward=1e9f,minSide=1e9f,maxSide=-1e9f;
     for(const auto &bodies:footprints){
      for(const auto &body:bodies)for(const auto &p:body.p){
      const float px=p.x-seats[seat].x;
      const float py=p.y-seats[seat].y;
      minForward=fminf(minForward,px*cosf(ref)+py*sinf(ref));
      const float side=-px*sinf(ref)+py*cosf(ref);
      minSide=fminf(minSide,side);maxSide=fmaxf(maxSide,side);
      }
     }
     if(phase<1000 && std::isfinite(previousForward[seat]) &&
        previousForward[seat]<=0 && minForward>0){
      if(seats[seat].red?maxSide>=-25:minSide<=25){failReason=3;failSeat=seat;return false;}
      witnessed[seat]=completedLaps;
      ++passCounts[seat];
     }
     previousForward[seat]=minForward;
    }
   }
  }
  float lookahead=OBSTACLE_LATER_LAP_LOOKAHEAD_MM;
  if(nearCorner(baselinePath[progressIndex].distanceMm))lookahead*=OBSTACLE_LOOKAHEAD_CORNER_SCALE;
  const float handoffWeight=lapHandoffWeight();
  const auto target=lapHandoffTarget(route,pose,lookahead,handoffWeight);
  const float dx=target.x-x,dy=target.y-y;
  const float forward=dx*cosf(heading)+dy*sinf(heading);
  const float lateral=-dx*sinf(heading)+dy*cosf(heading);
  const float requestedCurvature=2*lateral/fmaxf(1,dx*dx+dy*dy);
  const float requiredSteering=pursuitServoForCurvature(requestedCurvature);
  if(!std::isfinite(requiredSteering)){failReason=4;return false;}
  const float steering=static_cast<int>(clampFloat(requiredSteering,-OBSTACLE_MAX_PURSUIT_STEERING_DEG,
                                 OBSTACLE_MAX_PURSUIT_STEERING_DEG));
  if(!parkingReturnMotionSafe(pose,static_cast<int>(steering))){failReason=8;return false;}
  const float curvature=fabsf(steering)<2?0:-1/Ackermann::getTurnRadius(steering);
  // Worst-case 20 ms travel at the new maximum, including 10% speed error.
  // Constant step stresses every turn at this resolution even when speed falls.
  const float ds=OBSTACLE_PATH_MAX_SPEED*OBSTACLE_LATER_LAP_SPEED_FACTOR*
      OBSTACLE_LATER_LAP_SPEED_TOLERANCE*.020f;
  simulatedEncoderMm+=ds;
  const float next=heading+curvature*ds;
  if(fabsf(curvature)>1e-6){x+=(sinf(next)-sinf(heading))/curvature;
   y+=(cosf(heading)-cosf(next))/curvature;}
  else{x+=ds*cosf(heading);y+=ds*sinf(heading);}
  heading=next;
 }
 return false;
}
int main(){
 if(!stateChecks()){std::cout<<"STATE FAIL\n";return 2;}
 std::cout<<"STATE PASS\n";
 routeTurnSign=1;resetState();baseline();
 for(auto &seat:seats)seat.confirmed=seat.injected=false;
 for(int i:{1,5,9,14,19,22}){
  seats[i].confirmed=seats[i].injected=true;
  seats[i].red=i!=1&&i!=9;
 }
 learned();
 if(!buildOptimizedPath())return 4;
 const auto green=nearestPathIndex(baselinePath,seats[9].x,seats[9].y,0,pathLength);
 const auto red=nearestPathIndex(baselinePath,seats[14].x,seats[14].y,0,pathLength);
 const auto corner=nearestPathIndex(baselinePath,848.35f,858.68f,0,pathLength);
 if(optimizedPath[green].x>700.0f || optimizedPath[corner].x>600.0f ||
    optimizedPath[red].y<1270.0f || optimizedPath[red].y>1290.0f)return 4;
 std::cout<<"LOG28 curve and RED plateau PASS\n";
 routeTurnSign=-1;resetState();baseline();
 for(auto &seat:seats)seat.confirmed=seat.injected=false;
 for(int i:{0,4,7,10,15,20}){
  seats[i].confirmed=seats[i].injected=true;
  seats[i].red=i!=4&&i!=20;
 }
 learned();
 if(!buildOptimizedPath())return 5;
 const auto singleRed=nearestPathIndex(baselinePath,seats[15].x,seats[15].y,0,pathLength);
 if(fabsf(optimizedPath[singleRed].y-755.0f)>1.0f)return 5;
 for(int offset=-7;offset<=7;++offset)
  if(fabsf(optimizedPath[singleRed+offset].y-optimizedPath[singleRed].y)>2.0f)return 5;
 resetState();learned();float passWall,passPillar,trackY[2]={};
 if(!driveTwoLaps(0,passWall,passPillar,trackY))return 5;
 if(trackY[0]<700.0f || trackY[0]>805.0f || trackY[1]<700.0f || trackY[1]>805.0f)return 5;
 std::cout<<"LOG29/37 straight inward RED pass, tracked peak y laps 2/3 "
          <<trackY[0]<<"/"<<trackY[1]<<" mm PASS\n";
 int direction,id=0,colors[24];float cap;
 while(std::cin>>direction>>cap){
  for(int &c:colors)if(!(std::cin>>c))return 3;
  routeTurnSign=direction;resetState();baseline();
  for(int i=0;i<24;++i){seats[i].confirmed=colors[i]!=0;
   seats[i].red=colors[i]==2;seats[i].injected=colors[i]!=0;}
  learned();joinSameColourStraightEnds(livePath,true);float wall,pillar;
  const bool pass=driveTwoLaps(cap,wall,pillar);
  std::cout<<"CASE "<<id++<<" "<<pass<<" "<<wall<<" "<<pillar<<" "
    <<failReason<<" "<<failSeat<<" "<<failX<<" "<<failY<<" "<<failH<<"\n";
 }
}
'''
    if 'void refineLaterLapPassConnections(' in source:
        fixture = fixture.replace(' int direction,id=0,colors[24];float cap;', r'''
 // Two outer GREEN end signs, followed by an outer-row middle RED in CCW.
 // The third section supplies the next pass beyond that long connection.
 routeTurnSign=1;resetState();baseline();
 for(auto &seat:seats)seat.confirmed=seat.injected=false;
 for(int i:{6,10,14,22}){
  seats[i].confirmed=seats[i].injected=true;
  seats[i].red=i>=14;
 }
 learned();joinSameColourStraightEnds(livePath,true);
 float length,curvature,totalTurn;
 if(!buildOptimizedPath())return 6;
 laterLapBending(optimizedPath,length,curvature,&totalTurn);
 const float pace=laterLapTargetTime(optimizedPath,true);
 if(totalTurn*180/PI>510.0f || pace>15.7f)return 6;
 resetState();learned();joinSameColourStraightEnds(livePath,true);
 if(!driveTwoLaps(0,passWall,passPillar) || passWall<50.0f || passPillar<40.0f)return 6;
 std::cout<<"CCW GREEN pair to RED turn/time/wall/pillar "
          <<totalTurn*180/PI<<"/"<<pace<<"/"<<passWall<<"/"<<passPillar<<" PASS\n";
 int direction,id=0,colors[24];float cap;
''')
    if 'void refineLaterLapLongCorners(' in source:
        fixture = fixture.replace(' int direction,id=0,colors[24];float cap;', r'''
 // The layouts from the annotated route comparisons. Each ceiling is
 // below the preceding planner's time on that layout.
 const struct {
  int direction;
  std::initializer_list<int> greens, reds;
  float maximumTime;
  float minimumPillar=50.0f;
  bool parallelStart=false;
 } longCorners[] = {
  {-1,{4,20},{0,7,10,15},13.70f},
  { 1,{6,10},{14,22},14.75f},
  {-1,{7,23},{0,10,14},16.10f},
  { 1,{1,9},{5,14,20},15.55f}
 };
 for(const auto &layout:longCorners){
  routeTurnSign=layout.direction;resetState();baseline();
  for(auto &seat:seats)seat.confirmed=seat.injected=false;
  for(int i:layout.greens){seats[i].confirmed=seats[i].injected=true;seats[i].red=false;}
  for(int i:layout.reds){seats[i].confirmed=seats[i].injected=true;seats[i].red=true;}
  learned();joinSameColourStraightEnds(livePath,true);
  if(!buildOptimizedPath())return 7;
  const float seconds=laterLapTargetTime(optimizedPath,true);
  if(seconds>layout.maximumTime)return 7;
  if(layout.parallelStart){
   const float lane=optimizedPath[0].y;
   for(uint16_t i=0;i<pathLength;++i){
    const float phase=baselinePath[i].distanceMm;
    if((phase<=150.0f || phase>=loopLengthMm-150.0f) &&
       fabsf(optimizedPath[i].y-lane)>0.5f)return 7;
   }
  }
  resetState();learned();joinSameColourStraightEnds(livePath,true);
  if(!driveTwoLaps(0,passWall,passPillar) || passWall<50.0f ||
     passPillar<layout.minimumPillar)return 7;
  std::cout<<"LONG CORNER layout/time/wall/pillar "<<layout.direction<<"/"
           <<seconds<<"/"<<passWall<<"/"<<passPillar<<" PASS\n";
 }
 int direction,id=0,colors[24];float cap;
''')
    if '20261121_lap23_lane_continuity' in source or 'void refineCcwParallelStart(' in source:
        fixture = fixture.replace(
            '{ 1,{1,9},{5,14,20},15.55f}',
            '''{ 1,{1,9},{5,14,20},15.55f},
  {-1,{2,7,11,22},{14},15.40f},
  {-1,{},{0,4,9,15,21},12.80f},
  { 1,{3,15},{6,10,19},18.05f,40.0f},
  { 1,{12,20},{11,17},14.00f}''')
    if 'void refineCcwParallelStart(' in source:
        fixture = fixture.replace('{ 1,{3,15},{6,10,19},18.05f,40.0f}',
                                  '{ 1,{3,15},{6,10,19},15.30f,45.0f,true}')
    if 'void refineLaterLapCornerGeometry(' in source:
        # The long front capsule overstates clearance needs on these bends.
        # The measured wheel/body audit checks the same layouts at 50 mm.
        layouts = []
        corner_layouts = CORNER_LAYOUTS
        if 'void refineLaterLapEarlyStartExit(' not in source:
            corner_layouts = CORNER_LAYOUTS[:12] if 'void refineLaterLapStartStraight(' in source else CORNER_LAYOUTS[:8]
        for direction, greens, reds, time, gap, parallel in corner_layouts:
            green_seats = ','.join(map(str, greens))
            red_seats = ','.join(map(str, reds))
            layouts.append(f'{{{direction},{{{green_seats}}},{{{red_seats}}},'
                           f'{time:.2f}f,{gap}.0f,{str(parallel).lower()}}}')
        fixture = fixture.replace(
            '{ 1,{3,15},{6,10,19},15.30f,45.0f,true}',
            ',\n  '.join(layouts))
    # baseline() must select the supplied travel direction at runtime.
    fixture = fixture.replace('anchor.heading_deg=180;', 'anchor.heading_deg=routeTurnSign>0?0:180;')
    return fixture


def main():
    fixture = later_lap_fixture()
    destination = ROOT/'local_workspace/later-laps'
    destination.mkdir(parents=True,exist_ok=True)
    (destination/'Arduino.h').write_text('#pragma once\n#define M_PI 3.14159265358979323846\n#include <cstddef>\n#include <cmath>\ntemplate<class T> T constrain(T v,T lo,T hi){return v<lo?lo:(v>hi?hi:v); }\n')
    cpp=destination/'check.cpp';cpp.write_text(fixture)
    compiler=shutil.which('g++') or shutil.which('clang++')
    if not compiler:raise RuntimeError('Host C++ compiler on PATH required')
    exe=destination/'check.exe'
    subprocess.run([compiler,'-std=c++17','-O2','-I',str(ROOT/'include'),'-I',str(destination),str(cpp),'-o',str(exe)],check=True)
    # All 28 distinct normal-section layouts: 12 singletons plus 16 end pairs.
    section=[]
    for station,side,color in itertools.product(range(3),range(2),(1,2)):
        colors=[0]*6;colors[2*station+side]=color;section.append(colors)
    for first,last,c1,c2 in itertools.product(range(2),range(2),(1,2),(1,2)):
        colors=[0]*6;colors[first]=c1;colors[4+last]=c2;section.append(colors)
    cases=[]
    for direction,cap in itertools.product((-1,1),(0,140)):
        inner=0 if direction<0 else 1
        start=[[0]*6]
        for station,color in itertools.product(range(3),(1,2)):
            colors=[0]*6;colors[2*station+inner]=color;start.append(colors)
        for c1,c2 in itertools.product((1,2),repeat=2):
            colors=[0]*6;colors[inner]=c1;colors[4+inner]=c2;start.append(colors)
        for layout in start:
            for normal in section:
                # Repeat the normal layout on remaining straights to exercise
                # same/different colour approaches through every corner.
                cases.append(dict(direction=direction,speed_cap=cap,colors=layout+normal*3))
    # All ordered neighbouring layouts, alternating through the other three
    # straights, expose different colours and placement sides at each corner.
    for direction,a,b in itertools.product((-1,1),section,section):
        cases.append(dict(direction=direction,speed_cap=0,colors=[0]*6+a+b+a))
    # Exact heterogeneous physical A map, including the missed S2 GREEN in466.
    physical=[0]*24
    for seat,color in ((2,1),(6,2),(11,1),(12,1),(17,2),(20,2)):physical[seat]=color
    for direction in (-1,1):
        cases.append(dict(direction=direction,speed_cap=0,colors=physical.copy() if direction<0 else [physical[i^1] for i in range(24)]))
    # Same physical A field traversed CCW: reverse section order, station
    # order and local left/right; the start middle remains middle-inner.
    reverse=[0]*24
    for i,c in enumerate(physical):
        section,station,side=i//6,(i%6)//2,i%2
        reverse[((4-section)%4)*6+(2-station)*2+(side^1)]=c
    cases.append(dict(direction=1,speed_cap=0,colors=reverse))
    # Exact layoutB observed in477/478; reverse canonical seats for CCW.
    physical_b=[0]*24
    for seat,color in ((0,1),(4,2),(7,1),(10,2),(15,1),(19,2),(22,1)):
        physical_b[seat]=color
    reverse_b=[0]*24
    for i,c in enumerate(physical_b):
        section,station,side=i//6,(i%6)//2,i%2
        reverse_b[((4-section)%4)*6+(2-station)*2+(side^1)]=c
    for direction,layout in ((-1,physical_b),(1,reverse_b)):
        cases.append(dict(direction=direction,speed_cap=0,colors=layout))
    # C: start far-inner RED; two outer GREEN ends in the next straight.
    physical_c=[0]*24
    for seat,color in ((4,2),(7,1),(11,1),(17,2),(22,1)):
        physical_c[seat]=color
    reverse_c=[0]*24
    for i,c in enumerate(physical_c):
        section,station,side=i//6,(i%6)//2,i%2
        reverse_c[((4-section)%4)*6+(2-station)*2+(side^1)]=c
    for direction,layout in ((-1,physical_c),(1,reverse_c)):
        cases.append(dict(direction=direction,speed_cap=0,colors=layout))
    # Log 7 stores visible 50 mm GREEN at seat 10 as effective RED.
    # Both recorded laps must use that same passing direction.
    log7_cw, log7_ccw = [0]*24, [0]*24
    for seat, color in ((0,2),(4,1),(7,2),(10,2),(14,2),(21,1)):
        log7_cw[seat] = color
        section, station, side = seat//6, (seat%6)//2, seat%2
        log7_ccw[((4-section)%4)*6+(2-station)*2+(side^1)] = color
    for direction, layout in ((-1,log7_cw),(1,log7_ccw)):
        cases.append(dict(direction=direction,speed_cap=0,colors=layout))
    # Log 28: a single CCW GREEN followed by a single RED in the next section.
    log28 = [0]*24
    for seat, color in ((1,1),(5,2),(9,1),(14,2),(19,2),(22,2)):
        log28[seat] = color
    cases.append(dict(direction=1,speed_cap=0,colors=log28))
    log29 = [0]*24
    for seat, color in ((0,2),(4,1),(7,2),(10,2),(15,2),(20,1)):
        log29[seat] = color
    cases.append(dict(direction=-1,speed_cap=0,colors=log29))
    inputs='\n'.join(' '.join(map(str,[c['direction'],c['speed_cap'],*c['colors']])) for c in cases)+'\n'
    run=subprocess.run([str(exe)],input=inputs,text=True,capture_output=True,check=True)
    (destination/'output.txt').write_text(run.stdout)
    for line in run.stdout.splitlines():
        if line.startswith(('DISCOVERY ','LOG28 ','LOG29/37 ','CCW GREEN pair ','LONG CORNER ')):print(line)
        if line.startswith('CASE '):
            _,number,passed,wall,pillar,reason,seat,x,y,h=line.split();cases[int(number)].update(
                passed=bool(int(passed)),minimum_wall_mm=float(wall),minimum_pillar_mm=float(pillar),
                failure_reason=0 if int(passed) else int(reason),
                failure_seat=int(seat),pose=[float(x),float(y),float(h)])
    # A collision-free ideal replay with only a few mm reserve is not an
    # acceptable optimization. Preserve a30mm modeled pillar-reserve floor.
    for c in cases:
        if c['passed'] and c['minimum_pillar_mm'] < 30:
            c['passed']=False
            c['failure_reason']=7
    failed=[c for c in cases if not c['passed']]
    report=dict(total=len(cases),failed=len(failed),cases=cases,
                limitations=['Known explicit layouts, calibrated CAD plant, no images, ToF feedback or servo delay',
                             'Repeated sections and all ordered neighbouring layouts, not exhaustive full-field layouts'])
    (destination/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print('STATE PASS; LATER LAPS',len(cases),'FAILED',len(failed))
    for case in failed[:12]:print('FAIL',case)
    if failed:raise SystemExit(1)


if __name__=='__main__':main()
