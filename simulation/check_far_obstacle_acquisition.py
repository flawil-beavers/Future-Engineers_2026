"""Production blob/projection/vote path beyond the 600mm clear-view window.

Synthetic geometry, not a measured camera range guarantee. Route/map mutation
is stubbed after confirmation; acquisition and association are production code.
"""
import shutil
import subprocess

from check_connector_servo_resume import block
from check_cw_start_planner import ROOT


def fixture_source(surprise=False):
    path = (ROOT/'src/obstacle_path.cpp').read_text()
    camera = (ROOT/'src/obstacle_vision.cpp').read_text()
    code = ('#define OBSTACLE_SURPRISE_RULE_ENABLED '+('1' if surprise else '0')+'\n')+r'''
#include <cstdint>
#include <cmath>
#include <iostream>
constexpr int A0=0,A1=1,A2=2;
constexpr float PI=3.14159265358979323846f;
#include "obstacle_path.h"
#include "position_estimator.h"
#include "obstacle_blob_validation.h"
using std::isfinite;
struct NullLog {template<class T>void print(T){} template<class T>void print(T,int){}
 template<class T>void println(T){} template<class T>void println(T,int){}} Serial;
''' + block(path, path.index('struct CandidateSeat')) + r''';
CandidateSeat seats[OBSTACLE_SEAT_COUNT];
bool parkingSectionInnerSeatsOnly=false,runtimeTestMode=false;
bool parkingEntryActive=false,parkingEntryObserving=false,parkingEntryScouting=false,
 parkingEntryConnectorActive=false;
constexpr int COURSE_SEATS_PER_STATION=2;
bool oppositeSeatConflictReported[OBSTACLE_SEAT_COUNT]={};
int8_t surpriseSeatIndex=-1;
int sectionLayoutMode=0,completedLaps=0,injectionCount=0,progressIndex=0,routeTurnSign=-1;
struct Point {float distanceMm=0;}baselinePath[1];
float loopLengthMm=10000;unsigned long clockMs=1000;
unsigned long millis(){return clockMs;}
PositionEstimate hostPose;
PositionEstimate get_position_struct(){return hostPose;}
float distanceSquared(float x,float y,float a,float b){return (x-a)*(x-a)+(y-b)*(y-b);}
int stationIndexForSeat(uint8_t seat){return seat/2;}
bool obstacle_blob_valid_for_acquisition(const Blob *b){return obstacle_blob_shape_valid(b);}
bool mappedParkingSeatFootMatches(uint8_t,const PositionEstimate &,int,bool){return true;}
void logRedSeatProjection(const ObstacleObservationResult &,const char *,bool=false){}
void finalizeConfirmedSeat(ObstacleObservationResult &r){
 r.status=OBSTACLE_OBSERVATION_CONFIRMED;seats[r.seatId].injected=true;++injectionCount;
}
'''
    for signature in ('float obstacle_camera_bearing_deg(',
                      'float obstacle_estimate_camera_forward_mm(',
                      'float obstacle_estimate_camera_range_mm('):
        code += block(camera, camera.index(signature)) + '\n'
    if surprise:
        code+='struct DiscoveryStation{bool observedClear=false;}discoveryStations[OBSTACLE_SEAT_COUNT/2];\n'
        code+='uint8_t sectionInferredEmpty(uint8_t){return 0;}\n'
        code+=path[path.index('enum SurpriseOrder :'):path.index('SurpriseOrder surpriseOrderForSeat(')]
        code+=block(path,path.index('SurpriseOrder surpriseOrderForSeat('))+'\n'
        code+=block(path,path.index('void rememberModifiedSign('))+'\n'
    for signature in ('int nearestSeatIndex(', 'bool recordSeatVote(',
                      'bool recordObservedSeatVote(',
                      'void prepareConsecutiveVote(', 'void expirePendingVotes(',
                      'float cyclicDistanceForward(', 'bool distantSeatSideAmbiguous(',
                      'int resolveSeatRowByBearing(', 'bool officialMiddleConflictsWithEnd(',
                      'int resolveCcwReturnRedByBearing(',
                      'ObstacleObservationResult obstacle_path_observe('):
        code += block(path, path.index(signature)) + '\n'
    return code


def main():
    code = fixture_source()
    code += r'''
void reset(float range, int row){
 for(auto &s:seats){s={};s.x=9000;s.y=9000;}
 seats[6].x=seats[7].x=800;seats[6].y=0;seats[7].y=200;
 seats[6].headingDeg=seats[7].headingDeg=0;
 seats[6].pathDistanceMm=seats[7].pathDistanceMm=775;
 hostPose={800-range-OBSTACLE_CAMERA_LOCAL_X_MM,float(row*200),0};
 completedLaps=0;injectionCount=0;clockMs+=1000;
}
Blob pillar(ColorType color,int foot){
 Blob b{};b.found=true;b.color=color;b.minX=154;b.maxX=174;
 b.minY=70;b.maxY=foot;b.centerX=164;b.centerY=(70+foot)/2;b.area=600;
 return b;
}
int main(){int checks=0;
 for(auto color:{ColorType::RED,ColorType::GREEN})for(int row:{0,1}){
  // foot104 gives 646mm forward, beyond the 600mm clear/absence window.
  for(int foot:{104,100}){
   Blob b=pillar(color,foot);float range=obstacle_estimate_camera_range_mm(&b);
   if(range<=OBSTACLE_DISCOVERY_VIEW_MAX_MM)return 1;
   reset(range,row);auto first=obstacle_path_observe(&b);++checks;
   if(first.status!=OBSTACLE_OBSERVATION_VOTE||first.seatId!=6+row||first.confirmed)return 2;
   clockMs+=80;auto second=obstacle_path_observe(&b);++checks;
   if(second.status!=OBSTACLE_OBSERVATION_CONFIRMED||!second.confirmed||
      !second.injected||second.seatId!=6+row||injectionCount!=1)return 3;
   // An existing map is immutable after lap1.
   completedLaps=1;obstacle_path_observe(&b);++checks;if(injectionCount!=1)return 4;
  }
  Blob b=pillar(color,104);float range=obstacle_estimate_camera_range_mm(&b);
  // A far pillar aligned with its seat retains the calibrated depth tolerance.
  // Only simultaneous angular AND depth disagreement rejects the new GREEN
  // end aliases; an80mm depth discrepancy alone must not force close scanning.
  reset(range,row);hostPose.x_mm-=80;
  obstacle_path_observe(&b);clockMs+=80;auto noisy=obstacle_path_observe(&b);++checks;
  if(!noisy.confirmed||noisy.seatId!=6+row)return 7;
  reset(range,row);hostPose.y_mm=100; // rail midpoint is ambiguous at any range.
  for(int i=0;i<3;++i){clockMs+=80;auto r=obstacle_path_observe(&b);++checks;
   if(r.confirmed||r.seatId!=-1||injectionCount)return 5;}
  reset(range,row);seats[6].pathDistanceMm=seats[7].pathDistanceMm=3000;
  for(int i=0;i<3;++i){clockMs+=80;auto r=obstacle_path_observe(&b);++checks;
   if(r.confirmed||r.seatId!=-1||injectionCount)return 6;}
 }
 std::cout<<"FAR ACQUISITION "<<checks<<" PASS: both colours/rows,646/700mm,"
   " votes,confirmation,injection dispatch,ambiguity,relevance,map freeze\n";
}
'''
    dest = ROOT/'local_workspace/far-acquisition'
    dest.mkdir(parents=True, exist_ok=True)
    # Use the existing host Arduino declarations; no hardware libraries needed.
    cpp, exe = dest/'check.cpp', dest/'check.exe'
    cpp.write_text(code)
    build = subprocess.run([shutil.which('g++'), '-std=c++17', '-O2',
                    '-I', str(ROOT/'simulation/host'), '-I', str(ROOT/'include'),
                    str(cpp), '-o', str(exe)], capture_output=True, text=True)
    (dest/'build.txt').write_text(build.stdout+build.stderr)
    if build.returncode:
        print('\n'.join(build.stderr.splitlines()[:35]))
        raise SystemExit(build.returncode)
    run = subprocess.run([str(exe)], capture_output=True, text=True)
    (dest/'output.txt').write_text(run.stdout+run.stderr)
    print(run.stdout, end='')
    raise SystemExit(run.returncode)


if __name__ == '__main__':
    main()
