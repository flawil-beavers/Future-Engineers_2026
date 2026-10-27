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
code=r'''
#include <cstdint>
#include <cstring>
#include <cmath>
#include <iostream>
constexpr int A0=0,A1=1,A2=2;
constexpr float PI=3.14159265358979323846f;
#include "config.h"
constexpr int COURSE_STATIONS_PER_SECTION=3,CORNER_VIEW_IDLE=0,CORNER_VIEW_SETTLE=1;
struct NullLog{template<class T>void print(T){} template<class T>void print(T,int){}
template<class T>void println(T){} template<class T>void println(T,int){}} Serial;
bool runtimeTestMode=false,lapFinishPending=false,parkingEntryConnectorActive=false,
 parkingEntryFarGreenFollowup=false,parkingEntryJoining=false;
int completedLaps=0;float curvature=0,steering=0,commandedSpeed=100;
bool discoveryHolding=false,discoveryBlocked=false,cornerViewAttempted[12]={},cornerViewExtraUsed=false;
int discoveryHoldStation=-1,discoveryBlockedStation=-1,cornerViewStation=-1,cornerViewPhase=0,cornerViewObservationTraceCount=0;
uint32_t nowMs=1000,discoveryHoldStartMs=0,cornerViewPhaseMs=0;
float cornerViewRequestedMinRange=0;
int selectedStation=-1;float selectedForward=335;int motorSpeed=-9;
uint32_t millis(){return nowMs;}
int nearestUpcomingUnresolvedStation(float &f){f=selectedForward;return selectedStation;}
float laterLapTrackingSpeed(float s,float,int){return s;}
void set_speed(int s){motorSpeed=s;}
void holdCornerViewCentered(){motorSpeed=0;}
int getLargestObstacle(){return 0;}
float cornerViewMinimumRange(int){return 300;}
struct Observation{bool productionValid=true;int seatId=-1;float rangeMm=300;int bottom=160;}lastDiscoveryObservation;
void update(){
'''+speed+dispatch+r'''
}
int main(){int n=0,bad=0;
 selectedStation=-1;update();++n;if(motorSpeed!=100||discoveryHolding)++bad;
 for(int station=0;station<12;++station){
  selectedStation=station;discoveryHolding=discoveryBlocked=false;
  discoveryHoldStation=discoveryBlockedStation=-1;cornerViewPhase=0;
  nowMs+=1000;update();++n;
  if(motorSpeed!=0||!discoveryHolding||cornerViewPhase!=0)++bad;
  nowMs+=OBSTACLE_DISCOVERY_HOLD_GRACE_MS;update();++n;
  if(motorSpeed!=0||cornerViewPhase!=CORNER_VIEW_SETTLE||cornerViewStation!=station||!cornerViewAttempted[station]||discoveryBlocked)++bad;
 }
 std::cout<<"DISCOVERY STATE "<<n<<" failures "<<bad<<"\n";return bad?1:0;
}
'''
dest=ROOT/'local_workspace/fix_521_522';dest.mkdir(parents=True,exist_ok=True)
(dest/'states.cpp').write_text(code)
subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),str(dest/'states.cpp'),'-o',str(dest/'states.exe')],check=True)
run=subprocess.run([str(dest/'states.exe')],text=True,capture_output=True)
(dest/'states_output.txt').write_text(run.stdout+run.stderr)
print(run.stdout,end='');raise SystemExit(run.returncode)
