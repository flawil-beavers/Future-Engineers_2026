"""Source-backed check for the non-terminal lap-1 discovery search.

The fixture executes the production viewpoint selector. It does not drive
hardware and writes only generated files below local_workspace/.
"""
from pathlib import Path
import shutil
import subprocess

from check_connector_servo_resume import block
from check_lap1_approach_view import fixture_source
from check_returned_603_and_parking_practice import run


ROOT = Path(__file__).resolve().parents[1]


def check_log52(source, selector):
    fixture = fixture_source()
    fixture += r'''
uint8_t cornerViewStation=3,cornerViewExtraTriedMask=0;
bool cornerViewForwardTried=false;
uint16_t cornerViewSearchCycle=0;
uint32_t cornerViewLastSearchMs=0,cornerViewPhaseMs=0;
float cornerViewSearchStartEncoder=1538.5f,cornerViewExtraDistanceMm=0;
int cornerViewExtraDirection=-1,cornerViewExtraSteering=0,cornerViewPhase=0;
constexpr int CORNER_VIEW_EXTRA_SETTLE=1;
void holdCornerViewSteering(int){}
'''
    for signature in ('bool seatComfortablyVisible(', 'PositionEstimate cornerViewArcPose(',
                      'bool cornerViewArcSafe('):
        fixture += block(source, source.rindex(signature)) + '\n'
    run('52-cw-forward-view', fixture + selector + r'''
int main(){
 routeTurnSign=-1;resetState();baseline();
 for(auto &seat:seats)seat.confirmed=seat.injected=false;
 for(auto &station:discoveryStations)station={};
 for(int station=0;station<3;++station)discoveryStations[station].observedClear=true;
 seats[1].confirmed=seats[1].red=true;
 discoveryStations[3].seatObservedClear[0]=true;
 const PositionEstimate stopped{-712.2f,-972.6f,123.75f};
 simulatedEncoderMm=1307.f;
 float bearing,range;
 seatCameraGeometry(7,stopped,bearing,range);
 if(fabsf(range-487.1f)>2||range<=OBSTACLE_DISCOVERY_CLEAR_MAX_MM)return 1;
 if(!beginCornerViewExtra(stopped)||cornerViewExtraDirection!=1||!cornerViewForwardTried){
  std::cout<<"LOG52 unexpected direction "<<cornerViewExtraDirection<<" distance "<<cornerViewExtraDistanceMm<<"\n";
  for(int steering:{0,-20,20})for(float travel:{-130.f,-100.f,-70.f,70.f,100.f,130.f}){
   auto p=cornerViewArcPose(stopped,steering,travel);
   seatCameraGeometry(7,p,bearing,range);
   std::cout<<steering<<"/"<<travel<<" "<<range<<"/"<<bearing<<" safe="<<
      cornerViewArcSafe(stopped,steering,travel+copysignf(20.f,travel))<<"\n";
  }
  return 2;
 }
 auto view=cornerViewArcPose(stopped,cornerViewExtraSteering,cornerViewExtraDistanceMm);
 seatCameraGeometry(7,view,bearing,range);
 if(range>435||fabsf(bearing)>18||!seatComfortablyVisible(7,view))return 3;
 if(!cornerViewArcSafe(stopped,cornerViewExtraSteering,cornerViewExtraDistanceMm+20))return 4;
 std::cout<<"LOG52 forward mm="<<cornerViewExtraDistanceMm<<" steering="<<cornerViewExtraSteering
          <<" view range/bearing="<<range<<"/"<<bearing<<"\n";
 clockMs+=500;cornerViewExtraTriedMask=6;
 if(beginCornerViewExtra(stopped))return 5;
 // With no legal rear candidates and the body already too close to the
 // wall, forward search must also keep the motor held.
 clockMs+=500;cornerViewForwardTried=false;
 if(beginCornerViewExtra(PositionEstimate{-712.2f,-1480.f,123.75f}))return 6;
 std::cout<<"LOG52 checked closer viewpoint, repeat and wall rejected\n";
}
''')


def main():
    source = (ROOT / "src/obstacle_path.cpp").read_text()
    selector = block(source, source.index("bool beginCornerViewExtra("))
    observe_start = source.index("    case CORNER_VIEW_OBSERVE:")
    observe_end = source.index("    case CORNER_VIEW_RETURN:", observe_start)
    observe = source[observe_start:observe_end]
    required = (
        "if (beginCornerViewExtra(pose)) return;",
        "Unresolved; observing and retrying safe viewpoints",
        "cornerViewCanResume = stationDirectlyResolved(cornerViewStation)",
    )
    if any(text not in observe for text in required):
        raise SystemExit("production unresolved branch is not an active retry")
    if "returned but station unresolved" in source:
        raise SystemExit("unresolved evidence still dispatches a terminal lock")

    fixture = r'''
#include <cstdint>
#include <cmath>
#include <iostream>
#include <limits>
#include <algorithm>
constexpr float PI=3.14159265358979323846f;
constexpr int COURSE_SEATS_PER_STATION=2;
constexpr float OBSTACLE_DISCOVERY_VIEW_MIN_MM=230,OBSTACLE_DISCOVERY_CLEAR_MAX_MM=450;
struct PositionEstimate{float x_mm=0,y_mm=0,heading_deg=0;};
struct DiscoveryStation{bool seatObservedClear[2]={};};
DiscoveryStation discoveryStations[12];
struct CandidateSeat{};CandidateSeat seats[24];
enum Phase{CORNER_VIEW_IDLE,CORNER_VIEW_EXTRA_SETTLE};
Phase cornerViewPhase=CORNER_VIEW_IDLE;
uint8_t cornerViewStation=6,cornerViewExtraTriedMask=0;
uint16_t cornerViewSearchCycle=0;
uint32_t cornerViewPhaseMs=0,nowMs=100;
uint32_t cornerViewLastSearchMs=0;
float cornerViewExtraDistanceMm=0,cornerViewSearchStartEncoder=0,encoder=0;
int cornerViewExtraSteering=0,heldSteering=0,cornerViewExtraDirection=-1;
bool cornerViewForwardTried=false;
bool safeNegative=true,safePositive=true,resolved=false;
int visibilityMode=0;
struct NullLog{template<class T>void print(T){} template<class T>void print(T,int){}
 template<class T>void println(T){}}Serial;
uint32_t millis(){return nowMs;}
float get_distance(){return encoder;}
bool stationDirectlyResolved(uint8_t){return resolved;}
PositionEstimate cornerViewArcPose(const PositionEstimate &p,int steering,float d){
 PositionEstimate q=p;q.x_mm+=d;q.heading_deg+=float(steering);return q;}
bool cornerViewArcSafe(const PositionEstimate &,int steering,float){
 return steering==0 ? false:steering<0?safeNegative:safePositive;}
bool seatComfortablyVisible(uint8_t seat,const PositionEstimate &view){
 if(visibilityMode==0)return true;
 if(visibilityMode==1)return view.heading_deg>0 && (seat%2)==1;
 return false;}
void seatCameraGeometry(uint8_t seat,const PositionEstimate &view,float &bearing,float &range){
 bearing=(seat%2?8.0f:-8.0f)-view.heading_deg*0.1f;range=255-view.x_mm;}
void holdCornerViewSteering(int steering){heldSteering=steering;}
''' + selector + r'''
int main(){
 PositionEstimate pose{};
 // Prefer a meaningful retreat. A failed attempt preserves its bit so
 // the next selection does not retry the same movement from the same pose.
 nowMs+=250;
 if(!beginCornerViewExtra(pose)||heldSteering!=-20||cornerViewExtraDistanceMm<110)return 1;
 cornerViewPhase=CORNER_VIEW_IDLE;nowMs+=250;
 if(!beginCornerViewExtra(pose)||heldSteering!=20)return 2;
 cornerViewPhase=CORNER_VIEW_IDLE;nowMs+=250;
 if(beginCornerViewExtra(pose))return 3;
 // Preserve clear evidence for one side and choose the only view that can see
 // the remaining seat.
 discoveryStations[6].seatObservedClear[0]=true;
 cornerViewExtraTriedMask=0;visibilityMode=1;nowMs+=250;
 if(!beginCornerViewExtra(pose)||cornerViewExtraSteering!=20)return 4;
 // An unavailable movement is never commanded; a later loop can retry after
 // fresh perception or a changed safe-preflight result.
 cornerViewExtraTriedMask=0;visibilityMode=0;
 safeNegative=safePositive=false;heldSteering=99;nowMs+=250;
 if(beginCornerViewExtra(pose)||heldSteering!=99)return 5;
 encoder=-340;safeNegative=safePositive=true;nowMs+=250;
 if(beginCornerViewExtra(pose))return 6;
 std::cout<<"ACTIVE LAP1 SEARCH distinct swept viewpoints, bounded total retreat, preserved clear evidence PASS\n";
}
'''
    out = ROOT / "local_workspace" / "active-lap1-search"
    out.mkdir(parents=True, exist_ok=True)
    cpp = out / "check.cpp"
    exe = out / "check.exe"
    cpp.write_text(fixture)
    subprocess.run([
        shutil.which("g++"), "-std=c++17", "-O2", str(cpp), "-o", str(exe)
    ], check=True)
    result = subprocess.run([str(exe)], text=True, capture_output=True)
    (out / "output.txt").write_text(result.stdout + result.stderr)
    print(result.stdout, end="")
    if result.returncode:
        raise SystemExit(result.returncode)
    check_log52(source, selector)


if __name__ == "__main__":
    main()
