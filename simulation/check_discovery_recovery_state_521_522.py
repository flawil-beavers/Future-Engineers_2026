"""Exercise actual speed/UNKNOWN/recovery dispatch blocks with fake clock/motor."""
from pathlib import Path
import shutil,subprocess
from check_connector_servo_resume import block
ROOT=Path(__file__).resolve().parents[1]
s=(ROOT/'src/obstacle_path.cpp').read_text()
start=s.index('    const char *discoveryTraceReason = nullptr;',s.index('void obstacle_path_update('))
end=s.index('    // Bounded onboard diagnostics:',start)
speed=s[start:end]
start=s.index('    if (discoveryTraceReason != nullptr &&\n        strcmp(discoveryTraceReason, "hold_expired")',end)
dispatch=block(s,start)
pace_helpers='\n'.join(block(s,s.index(signature)) for signature in (
    'float clampFloat(', 'float cappedPathSpeed(', 'float wrap180(',
    'bool traversingInjectedAvoidance(', 'float laterLapCurvatureSpeed(',
    'float laterLapTrackingSpeed(', 'float lap1ClearRouteSpeed(',
    'float parkingConnectorDepartureSpeed('))
code=r'''
#include <cstdint>
#include <cstring>
#include <cmath>
#include <iostream>
constexpr int A0=0,A1=1,A2=2;
constexpr float PI=3.14159265358979323846f;
#include "config.h"
#include "ackermann_kinematics.h"
#include "drive_profile.h"
using std::isfinite;
bool lap1CommandValid=false;float lookahead=100,lap1LookaheadMm=0,lap1SpeedMmS=0;
constexpr int COURSE_STATIONS_PER_SECTION=3,CORNER_VIEW_IDLE=0,CORNER_VIEW_SETTLE=1;
struct NullLog{template<class T>void print(T){} template<class T>void print(T,int){}
template<class T>void println(T){} template<class T>void println(T,int){}} Serial;
bool runtimeTestMode=false,lapFinishPending=false,parkingEntryConnectorActive=false,
 parkingEntryFarGreenFollowup=false,parkingEntryJoining=false;
bool shortStart=false,startStationResolved=true,lapCountingArmed=false;
int parkingEntryTargetStation=2,parkingEntryConnectorLength=0;
bool parkingShortStart(){return shortStart;}
bool stationResolved(int){return startStationResolved;}
int completedLaps=0;float curvature=0,steering=0,commandedSpeed=100;
bool discoveryHolding=false,discoveryBlocked=false,cornerViewAttempted[12]={};
struct Seat{bool confirmed=false,red=false,injected=false;float pathDistanceMm=0;}seats[24];
struct PositionEstimate{float x_mm=0,y_mm=0,heading_deg=0;}pose;
struct PathPoint{float x=0,y=0,distanceMm=0;} baselinePath[1];
int progressIndex=0;PathPoint *path=baselinePath;
struct Corner{float pathStartMm=500;} corners[1];
uint16_t pathLength=1;
uint16_t nearestPathIndex(const PathPoint*,float,float,uint16_t,uint16_t){return 0;}
int discoveryScanStation=-1;
float loopLengthMm=7000,routeHeading=0;
float connectorRouteHeading(const PathPoint *,int){return routeHeading;}
float cyclicDistanceForward(float from,float to){return fmodf(to-from+loopLengthMm,loopLengthMm);}
float parkingFinishSpeed(float s,const PositionEstimate&){return s;}
int routeTurnSign=1;float runtimeSpeedCapMmS=0;
int discoveryHoldStation=-1,discoveryBlockedStation=-1,cornerViewStation=-1,cornerViewPhase=0,cornerViewObservationTraceCount=0;
uint32_t nowMs=1000,discoveryHoldStartMs=0,cornerViewPhaseMs=0;
uint8_t cornerViewExtraTriedMask=0;uint16_t cornerViewSearchCycle=0;
uint32_t cornerViewLastNoSafeTraceMs=0,cornerViewLastSearchMs=0;
float cornerViewRequestedMinRange=0;
int selectedStation=-1;float selectedForward=335;int motorSpeed=-9;
uint32_t millis(){return nowMs;}
int nearestUpcomingUnresolvedStation(float &f){f=selectedForward;return selectedStation;}
int besideStation=-1;
int nearestUpcomingUnresolvedStation(float &f,uint16_t,float){
 if(besideStation>=0){f=10;return besideStation;}
 return nearestUpcomingUnresolvedStation(f);
}
template<class A,class B>float laterLapInwardTrackingSpeed(float s,const A&,B,int){return s;}
void set_speed(int s){motorSpeed=s;}
void holdCornerViewCentered(){motorSpeed=0;}
int getLargestObstacle(){return 0;}
float cornerViewMinimumRange(int){return 300;}
struct Observation{bool productionValid=true;int seatId=-1;float rangeMm=300;int bottom=160;}lastDiscoveryObservation;
'''+pace_helpers+r'''
void update(){
'''+speed+dispatch+r'''
}
int main(){int n=0,bad=0;
 selectedStation=-1;update();++n;if(motorSpeed!=100||discoveryHolding)++bad;
 // Moving lap1: use the new cap only in the existing observation window.
 commandedSpeed=260;selectedStation=4;
 for(float distance:{341.f,500.f,750.f,751.f}){
  selectedForward=distance;update();++n;
  const int expected=distance<=750?static_cast<int>(OBSTACLE_DISCOVERY_SPEED_MM_S):260;
  if(motorSpeed!=expected||discoveryHolding)++bad;
 }
 commandedSpeed=90;selectedForward=500;update();++n;if(motorSpeed!=90)++bad;
 commandedSpeed=260;
 for(int lap:{1,2}){completedLaps=lap;update();++n;if(motorSpeed!=260)++bad;}
 completedLaps=0;runtimeTestMode=true;update();++n;if(motorSpeed!=260)++bad;
 runtimeTestMode=false;commandedSpeed=100;selectedForward=335;
 for(int station=0;station<12;++station){
  selectedStation=station;discoveryHolding=discoveryBlocked=false;
  discoveryHoldStation=discoveryBlockedStation=-1;cornerViewPhase=0;
  nowMs+=1000;update();++n;
  if(motorSpeed!=0||!discoveryHolding||cornerViewPhase!=0)++bad;
  nowMs+=OBSTACLE_DISCOVERY_HOLD_GRACE_MS;update();++n;
  if(motorSpeed!=0||cornerViewPhase!=CORNER_VIEW_SETTLE||cornerViewStation!=station||!cornerViewAttempted[station]||discoveryBlocked)++bad;
 }
 discoveryHolding=discoveryBlocked=false;cornerViewPhase=0;
 selectedStation=4;selectedForward=1200;commandedSpeed=260;
 const auto expect=[&](int expected){update();++n;if(motorSpeed!=expected)++bad;};
 seats[6].pathDistanceMm=150;seats[6].confirmed=true;
 expect(260); // Confirmation alone may still be deferred from the path.
 seats[6].injected=true;
 expect(320);
 selectedStation=-1;expect(320);
 selectedStation=4;selectedForward=1000;expect(290);
 selectedForward=900;expect(260);
 selectedForward=750;expect(120);
 selectedForward=1200;
 discoveryScanStation=4;expect(260);discoveryScanStation=-1;
 parkingEntryFarGreenFollowup=true;expect(OBSTACLE_PARKING_ENTRY_JOIN_SPEED_MM_S);
 parkingEntryFarGreenFollowup=false;
 pose.y_mm=40;expect(260);pose.y_mm=0;
 pose.heading_deg=18;expect(260);pose.heading_deg=INFINITY;expect(260);pose.heading_deg=0;
 runtimeSpeedCapMmS=280;expect(280);runtimeSpeedCapMmS=0;
 steering=42;update();++n;if(motorSpeed<=260||motorSpeed>=320)++bad;steering=0;
 for(int lap:{1,2}){completedLaps=lap;expect(260);}completedLaps=0;

 // Departure pace with the first place accounted for, in either direction.
 seats[6].injected=false;shortStart=true;parkingEntryConnectorLength=2;
 parkingEntryConnectorActive=true;pose.y_mm=-1150;
 for(int sign:{-1,1}){
  routeTurnSign=sign;
  selectedStation=-1;expect(140);
  besideStation=2;expect(80);besideStation=-1;
  selectedStation=4;selectedForward=850;expect(80);
  selectedStation=-1;pose.y_mm=-1200;expect(80);pose.y_mm=-1150;
  startStationResolved=false;expect(80);startStationResolved=true;
  steering=21;expect(80);steering=0;
 }
 parkingEntryConnectorActive=false;path[0].y=pose.y_mm;
 selectedStation=-1;expect(320);
 besideStation=2;expect(260);besideStation=-1;
 selectedStation=4;selectedForward=750;expect(120);
 selectedForward=1200;lapCountingArmed=true;expect(260);
 baselinePath[0].y=pose.y_mm=0;shortStart=false;seats[6].injected=true;
 // Model the actual motor ramp across quantized 50 mm route stations.
 // The extra pace is gone before aiming; no state needs resetting at a sign.
 for(float dt:{.010f,.020f,.050f}){
  float distance=1500,speed=260,accel=0,limit=0,peak=0;
  while(distance>OBSTACLE_LOOK_START_MM){
   selectedForward=roundf(distance/OBSTACLE_PATH_SAMPLE_MM)*OBSTACLE_PATH_SAMPLE_MM;
   update();distance-=DriveProfile::advance(speed,accel,limit,motorSpeed,dt,DEFAULT_ACCELERATION);
   peak=fmaxf(peak,speed);
  }
  ++n;if(peak<317||speed>261)++bad;
  std::cout<<"Confirmed pass ramp dt/peak/aim_mm_s "<<dt<<"/"<<peak<<"/"<<speed<<"\n";
 }
 std::cout<<"DISCOVERY STATE "<<n<<" failures "<<bad<<"\n";return bad?1:0;
}
'''
dest=ROOT/'local_workspace/fix_521_522';dest.mkdir(parents=True,exist_ok=True)
(dest/'states.cpp').write_text(code)
subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),
                '-I',str(ROOT/'simulation/host'),str(dest/'states.cpp'),'-o',str(dest/'states.exe')],check=True)
run=subprocess.run([str(dest/'states.exe')],text=True,capture_output=True)
(dest/'states_output.txt').write_text(run.stdout+run.stderr)
print(run.stdout,end='');raise SystemExit(run.returncode)
