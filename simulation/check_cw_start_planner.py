"""Compile the actual CW scan, route, clearance and connector C++ functions.

No robot/firmware upload. Extracts production code on each invocation; the
fixture replaces only logging/hardware and supplies explicit known layouts.
Includes all ten official layouts, clear diagnostic and archived exit poses.
"""
import json
import shutil
import subprocess
from pathlib import Path

from check_connector_servo_resume import block
from review_cw_start_matrix import review

ROOT = Path(__file__).resolve().parents[1]


def fixture_source():
    source = (ROOT / 'src/obstacle_path.cpp').read_text()
    header = (ROOT / 'include/obstacle_path.h').read_text()
    def extract(signature):
        return block(source, source.index(signature))
    functions = [extract(s) for s in (
        'float clampFloat(', 'float adaptiveLookahead(', 'float wrap180(',
        'float distanceSquared(', 'void transformFromRunFrame(',
        'void appendLocalPoint(', 'void appendStraight(', 'void appendCorner(',
        'PathPoint interpolateBaseline(', 'void initializeSeats(',
        'float cyclicDistanceForward(', 'void recomputeSpeedProfile(',
        'void smoothRange(', 'uint16_t nearestPathIndex(',
        'float targetLateralForSeat(', 'void displaceForSeat(',
        'bool parkingShortStart()', 'bool parkingStartFootprintSafe(',
        'float parkingEntryScanArcMm()', 'float parkingEntryScoutArcMm()',
        'void appendParkingEntryPoint(', 'void buildParkingEntryPath(',
        'bool preflightCwStartArc(', 'float connectorRouteHeading(',
        'PathPoint connectorLookaheadFrom(\n    uint8_t index, float lookaheadMm, const PathPoint *route,\n    uint16_t mergeIndex, bool continueIntoRoute,\n    uint16_t *targetIndex)\n{',
        'bool connectorJoinReached(', 'bool connectorRolloutFeasible(', 'bool tryParkingEntryConnectorMerge(',
        'bool buildParkingEntryConnector(', 'bool parkingConnectorMergeUnchanged(',
        'bool retainParkingConnectorForFarGreen(')]
    # Only PathPoint and CandidateSeat; terminate at the next declaration.
    structs = block(source, source.index('struct PathPoint'))+';\n'+block(source, source.index('struct CandidateSeat'))+';\n'
    clearance = source[source.index('struct WallSegment'):source.index('bool isExtremeAdjacentPair(')]
    types = header[header.index('enum ObstacleWallFeature'):header.index('/** Section layout assumption')]
    fixture = r'''
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
constexpr int A0=0,A1=1,A2=2;
constexpr float PI=3.14159265358979323846f;
#include "run_telemetry.h"
bool telemetryRouteDirty=false;
#include "config.h"
#include "parking_start_footprint.h"
@@TYPES@@
@@STRUCTS@@
struct PositionEstimate {float x_mm=0,y_mm=0,heading_deg=0,confidence_mm=0;};
struct Corner {float pathStartMm=0,pathEndMm=0; bool recedesOnLeft=false,recedesOnRight=false;};
struct Discovery {bool observedClear=false;};
struct NullLog {
 template<class T> void print(T) {} template<class T> void print(T,int) {}
 template<class T> void println(T) {} template<class T> void println(T,int) {}
}; NullLog Serial;
PathPoint baselinePath[OBSTACLE_MAX_PATH_WAYPOINTS],smoothingBuffer[OBSTACLE_MAX_PATH_WAYPOINTS];
PathPoint livePath[OBSTACLE_MAX_PATH_WAYPOINTS];
CandidateSeat seats[OBSTACLE_SEAT_COUNT]; Corner corners[4]; Discovery discoveryStations[12];
uint16_t pathLength=0;
float loopLengthMm=0,firstCornerDistanceMm=500;
int routeTurnSign=-1,parkingEntryTargetStation=1;
bool parkingCwShortStart=true,parkingEntryConnectorRouteLookahead=false;
bool parkingCcwShortStart=false;
bool parkingCcwFirstEdgeStart=false;
PathPoint parkingEntryPath[OBSTACLE_PARKING_ENTRY_MAX_WAYPOINTS];
uint8_t parkingEntryLength=0,parkingEntryProgress=0;
PathPoint parkingEntryConnector[OBSTACLE_PARKING_ENTRY_CONNECTOR_MAX_WAYPOINTS];
uint8_t parkingEntryConnectorLength=0;
float parkingEntryConnectorLookaheadMm=0;
unsigned parkingEntryConnectorTraceCount=0,parkingEntryConnectorTraceMs=0;
uint16_t parkingEntryConnectorMergeIndex=0;
bool parkingEntryConnectorActive=false,parkingEntryFarGreenFollowup=false;
int parkingEntryConnectorChangedSeat=-1;
bool targetsOuterExtreme(const CandidateSeat &seat) {return seat.red ? seat.lateralMm<0 : seat.lateralMm>0;}
constexpr int COURSE_SEATS_PER_STATION=2;
float clampFloat(float,float,float);
bool calculateClearanceAtPose(const CandidateSeat&,float,float,float,ObstacleClearanceSample&);
PathPoint connectorLookaheadFrom(uint8_t,float,const PathPoint*,uint16_t,bool,uint16_t* = nullptr);
@@CLEARANCE@@
@@FUNCTIONS@@
void baseline() {
 pathLength=0; PositionEstimate anchor;anchor.y_mm=-1000;anchor.heading_deg=180;
 float x=0,y=0,h=0,d=0;appendLocalPoint(x,y,0,d,anchor);
 appendStraight(500,x,y,h,d,anchor);
 for(int c=0;c<4;++c){appendCorner(c,x,y,h,d,anchor);appendStraight(c==3?500:1000,x,y,h,d,anchor);}
 loopLengthMm=d;initializeSeats();
}
int main() {
 float x,y,h,returnX,returnY,returnH;int near,middle,far,total=0,failed=0;
 while(std::cin>>x>>y>>h>>near>>middle>>far>>returnX>>returnY>>returnH) {
  baseline();memcpy(livePath,baselinePath,sizeof(PathPoint)*pathLength);
  int colors[3]={near,middle,far};
  for(int s=0;s<3;++s) {
   discoveryStations[s].observedClear=colors[s]==0;
   if(colors[s]){seats[2*s].confirmed=true;seats[2*s].red=colors[s]==2;}
   // Right/near is confirmed in the map but initially stored, not injected.
   if(s && colors[s])displaceForSeat(livePath,2*s,
       s==2&&colors[s]==1?OBSTACLE_PARKING_CW_INNER_GREEN_CLEARANCE_MM:OBSTACLE_LAP1_CLEARANCE_MM,false);
  }
  PositionEstimate pose;pose.x_mm=x;pose.y_mm=y;pose.heading_deg=h;
  bool arc=preflightCwStartArc(pose,parkingEntryScanArcMm()+parkingEntryScoutArcMm());
  buildParkingEntryPath(pose);
  auto at=parkingEntryPath[parkingEntryLength-1];
  pose.x_mm=at.x+returnX;pose.y_mm=at.y+returnY;pose.heading_deg=at.headingDeg+returnH;
  uint16_t merge=0; bool join=buildParkingEntryConnector(pose,livePath,merge);
  ++total; if(!arc||!join){++failed;std::cout<<"FAIL "<<x<<","<<y<<","<<h<<" layout="<<near<<middle<<far<<" arc="<<arc<<" join="<<join<<"\n";}
  if(far && returnX==0 && returnY==0 && returnH==0) {
   // A left/end pillar often becomes visible only while already joining.
   // Build its actual unresolved prefix, then test retained joins at three
   // positions after that pillar is confirmed and injected into the route.
   memcpy(livePath,baselinePath,sizeof(PathPoint)*pathLength);
   seats[4].confirmed=false;seats[4].injected=false;
   bool prefix=buildParkingEntryConnector(pose,livePath,merge);
   if(!prefix){++failed;std::cout<<"FAIL late prefix\n";}
   else {
    parkingEntryConnectorMergeIndex=merge;parkingEntryConnectorActive=true;
    const uint8_t prefixLength=parkingEntryConnectorLength;
    const float prefixLookahead=parkingEntryConnectorLookaheadMm;
    PathPoint savedPrefix[OBSTACLE_PARKING_ENTRY_CONNECTOR_MAX_WAYPOINTS];
    memcpy(savedPrefix,parkingEntryConnector,sizeof(savedPrefix));
    seats[4].confirmed=true;seats[4].injected=true;
    displaceForSeat(livePath,4,far==1?OBSTACLE_PARKING_CW_INNER_GREEN_CLEARANCE_MM:OBSTACLE_LAP1_CLEARANCE_MM,false);
    for(int part=0;part<3;++part) {
     memcpy(parkingEntryConnector,savedPrefix,sizeof(savedPrefix));
     parkingEntryConnectorLength=prefixLength;parkingEntryConnectorLookaheadMm=prefixLookahead;
     parkingEntryConnectorMergeIndex=merge;
     auto point=savedPrefix[part*prefixLength/3];
     PositionEstimate advanced;advanced.x_mm=point.x;advanced.y_mm=point.y;advanced.heading_deg=point.headingDeg;
     parkingEntryConnectorChangedSeat=4;
     ++total;
     uint16_t revisedMerge=0;
     if(!retainParkingConnectorForFarGreen(advanced,livePath) &&
        !buildParkingEntryConnector(advanced,livePath,revisedMerge)) {
      ++failed;std::cout<<"FAIL late detection layout="<<near<<middle<<far<<" part="<<part<<"\n";
     }
    }
   }
   parkingEntryConnectorActive=false;
  }
 }
 std::cout<<"TOTAL "<<total<<" FAILED "<<failed<<"\n";
 return failed?1:0;
}
'''.replace('@@TYPES@@', types).replace('@@STRUCTS@@', structs).replace('@@CLEARANCE@@', clearance).replace('@@FUNCTIONS@@', '\n'.join(functions))
    return fixture


def main():
    fixture = fixture_source()
    # Execute the production settled-pose gate as well as the actual swept
    # scanner/connector. Log 460 had a valid seed but brake-settled error 2.1.
    motion_source = (ROOT/'src/obstacle.cpp').read_text()
    gate = block(motion_source, motion_source.index('static bool cwShortStartPoseUsable('))
    fixture = fixture.replace('int main() {', gate + r'''
int main() {
 PositionEstimate settled;settled.x_mm=240.9f;settled.y_mm=-1220.6f;settled.heading_deg=182.2f;
 if(!cwShortStartPoseUsable(true,2.1f,settled) ||
    !cwShortStartPoseUsable(true,5.0f,settled) ||
    cwShortStartPoseUsable(true,5.01f,settled) ||
    cwShortStartPoseUsable(false,2.1f,settled) ||
    cwShortStartPoseUsable(true,NAN,settled))return 2;
 settled.heading_deg=NAN;if(cwShortStartPoseUsable(true,2.1f,settled))return 2;
''')
    fixture = fixture.replace('#include <cmath>', '#include <cmath>\nusing std::isfinite;')
    destination = ROOT / 'local_workspace/cw-start-planner'
    destination.mkdir(parents=True, exist_ok=True)
    cpp = destination / 'check.cpp'
    cpp.write_text(fixture)
    compiler = shutil.which('g++') or shutil.which('clang++')
    if not compiler:
        raise RuntimeError('A host C++ compiler on PATH is required')
    executable = destination / 'check.exe'
    subprocess.run([compiler, '-std=c++17', '-O2', '-I', str(ROOT/'include'), str(cpp), '-o', str(executable)], check=True)
    layouts = [(1,0,0),(2,0,0),(0,1,0),(0,2,0),(0,0,1),(0,0,2),
               (1,0,1),(1,0,2),(2,0,1),(2,0,2),(0,0,0)]
    poses = [review(n)['proposed_short_approach']['logged_prelocalization_pose']
             for n in range(426,446) if n != 429]
    cases = [(*pose,*layout,0,0,0) for pose in poses for layout in layouts]
    cases += [(243+dx,-1214+dy,180+dh,*layout,0,0,0)
              for dx in (-5,0,5) for dy in (-5,0,5) for dh in (-1,0,1)
              for layout in layouts]
    cases += [(*pose,*layout,dx,dy,dh) for pose in poses
              for layout in layouts for dx in (-5,0,5)
              for dy in (-5,0,5) for dh in (-1,0,1)]
    # Actual log-460 exit, every official start layout, including measured
    # position/heading variation through the new settled-heading allowance.
    cases += [(240.9+dx,-1220.6+dy,182.2+dh,*layout,0,0,0)
              for dx in (-5,0,5) for dy in (-5,0,5) for dh in (-.8,0,.8)
              for layout in layouts]
    # Current measured failed/successful exits and 5-degree candidate boundary.
    cases += [(238.8+dx,-1222.0+dy,183.23+dh,*layout,0,0,0)
              for dx in (-5,0,5) for dy in (-5,0,5) for dh in (-1,0,1)
              for layout in layouts]
    cases += [(240,-1220,180+dh,*layout,0,0,0)
              for dh in (-5,5) for layout in layouts]
    inputs = '\n'.join(' '.join(map(str,c)) for c in cases)+'\n'
    result = subprocess.run([str(executable)], input=inputs, text=True, capture_output=True)
    (destination/'result.txt').write_text(result.stdout+result.stderr)
    (destination/'cases.json').write_text(json.dumps(cases,indent=2)+'\n')
    print(result.stdout, end='')
    result.check_returncode()


if __name__ == '__main__':
    main()
