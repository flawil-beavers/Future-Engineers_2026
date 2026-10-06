"""Actual C++ stored-seat activation and parking footprint cross-check."""
import random
import shutil
import subprocess
from pathlib import Path

from check_connector_servo_resume import block
from parking_exit_swept_search import Pose, collision, robot_polygons

ROOT = Path(__file__).resolve().parents[1]


def main():
    source = (ROOT/'src/obstacle_path.cpp').read_text()
    functions = '\n'.join(block(source,source.index(s)) for s in (
        'float cyclicDistanceForward(', 'bool storeCwStartSeat(',
        'void activateCwStoredSeat(', 'bool storeCcwStartSeat(',
        'void activateCcwStoredSeats(', 'bool parkingEntryPrerequisitesResolved()'))
    fixture = r'''
#include <cstdint>
#include <iostream>
#include "parking_start_footprint.h"
constexpr float OBSTACLE_PARKING_CW_STORED_SEAT_APPROACH_MM=800;
constexpr float OBSTACLE_PARKING_CCW_STORED_SEAT_APPROACH_MM=800;
bool parkingCwShortStart=true,parkingEntryConnectorActive=false;
bool parkingCcwShortStart=false;uint8_t parkingCcwStoredSeatMask=0;
int parkingEntryTargetStation=2;bool resolved[3]={};
bool stationResolved(uint8_t s){return resolved[s];}
int8_t parkingCwStoredSeat=-1;unsigned completedLaps=0,injections=0;
float loopLengthMm=7141.5927f;
struct Seat {float pathDistanceMm=0;bool red=false,injected=false;};Seat seats[24];
struct Log{template<class T> void print(T){} template<class T> void println(T){}}Serial;
void injectSeat(uint8_t s,bool delayed){if(!delayed)std::abort();seats[s].injected=true;++injections;}
@@FUNCTIONS@@
int main(){
 for(bool red : {false,true}){
  seats[0]={loopLengthMm-500,red,false};injections=0;completedLaps=0;
  if(!storeCwStartSeat(0)||seats[0].injected) return 1;
  for(float phase : {loopLengthMm-300,0.f,500.f,4000.f,5800.f})
   activateCwStoredSeat(phase);
  if(injections!=0||parkingCwStoredSeat!=0) return 2;
  parkingEntryConnectorActive=true;activateCwStoredSeat(6000);
  if(injections!=0) return 3;
  parkingEntryConnectorActive=false;activateCwStoredSeat(6000);
  if(injections!=1||!seats[0].injected||parkingCwStoredSeat!=-1) return 4;
  activateCwStoredSeat(6100);if(injections!=1||storeCwStartSeat(0))return 5;
 }
 parkingCwShortStart=false;seats[0].injected=false;
 if(storeCwStartSeat(0))return 6;
 parkingCwShortStart=true;completedLaps=1;if(storeCwStartSeat(0))return 7;
 completedLaps=0;if(storeCwStartSeat(2))return 8;
 parkingCwShortStart=false;parkingCcwShortStart=true;
 // All colour pairs, including two stored positions, preserve both records.
 for(bool backRed : {false,true})for(bool middleRed : {false,true}){
  seats[1]={loopLengthMm-500,backRed,false};seats[3]={0,middleRed,false};
  injections=0;parkingCcwStoredSeatMask=0;
  if(!storeCcwStartSeat(1)||!storeCcwStartSeat(3)||
     parkingCcwStoredSeatMask!=10||injections) return 9;
  if(!storeCcwStartSeat(1)||parkingCcwStoredSeatMask!=10)return 10;
  // Initial phase zero is not a normal-route CCW pose: the scan/connector
  // has its own progress and blocks activation until the forward handoff.
  for(float phase : {100.f,500.f,4000.f,loopLengthMm-1400})
   activateCcwStoredSeats(phase);
  if(injections||parkingCcwStoredSeatMask!=10)return 11;
  parkingEntryConnectorActive=true;activateCcwStoredSeats(loopLengthMm-1300);
  if(injections)return 12;
  parkingEntryConnectorActive=false;activateCcwStoredSeats(loopLengthMm-1300);
  if(injections!=1||parkingCcwStoredSeatMask!=8||!seats[1].injected)return 13;
  activateCcwStoredSeats(loopLengthMm-800);
  if(injections!=2||parkingCcwStoredSeatMask||!seats[3].injected)return 14;
  activateCcwStoredSeats(loopLengthMm-700);
  if(injections!=2||storeCcwStartSeat(1)||storeCcwStartSeat(3))return 15;
 }
 seats[1].injected=false;completedLaps=1;
 if(storeCcwStartSeat(1))return 16;
 completedLaps=0;if(storeCcwStartSeat(5))return 17;
 parkingCcwShortStart=false;if(storeCcwStartSeat(1))return 18;
 // Unknown behind observations cannot block a resolved official CCW front;
 // unresolved front still holds, legacy CCW/CW still require their scout.
 resolved[2]=true;
 if(parkingEntryPrerequisitesResolved())return 19;
 parkingCcwShortStart=true;
 if(!parkingEntryPrerequisitesResolved())return 20;
 resolved[2]=false;resolved[1]=true;
 if(parkingEntryPrerequisitesResolved())return 21;
 float x,y,h;int steer,behind,ahead;
 while(std::cin>>x>>y>>h>>steer>>behind>>ahead)
  std::cout<<parking_start_footprint::safe(x,y,h,steer,behind,0,ahead)<<"\n";
}
'''.replace('@@FUNCTIONS@@',functions)
    destination=ROOT/'local_workspace/cw-start-state'
    destination.mkdir(parents=True,exist_ok=True)
    cpp=destination/'check.cpp';cpp.write_text(fixture)
    compiler=shutil.which('g++') or shutil.which('clang++')
    if not compiler:
        raise RuntimeError('A host C++ compiler on PATH is required')
    exe=destination/'check.exe'
    subprocess.run([compiler,'-std=c++17','-O2','-I',str(ROOT/'include'),str(cpp),'-o',str(exe)],check=True)
    random.seed(20261006)
    cases=[(random.uniform(170,550),random.uniform(-1460,-1030),
            random.uniform(65,195),random.choice((-1,0,1)),random.choice((0,1)),0)
           for _ in range(1200)]
    cases += [(random.uniform(-100,550),random.uniform(-1460,-800),
               random.uniform(-60,60),random.choice((-1,0,1)),0,1)
              for _ in range(1200)]
    expected=[]
    for x,y,h,steer,behind,ahead in cases:
        local=Pose(480-x,y+1500,180-h)
        polys=robot_polygons(local,-steer)
        safe=not any(collision(local,-steer,gap) for gap in (242.5,252.5))
        if behind and max(480-px for poly in polys for px,_ in poly)>=500:
            safe=False
        if ahead and min(480-px for poly in polys for px,_ in poly)<=0:
            safe=False
        expected.append(int(safe))
    result=subprocess.run([str(exe)],input='\n'.join(' '.join(map(str,c)) for c in cases)+'\n',
                          text=True,capture_output=True,check=True)
    actual=list(map(int,result.stdout.split()))
    assert actual==expected, [(cases[i],a,b) for i,(a,b) in enumerate(zip(actual,expected)) if a!=b][:5]
    print('PASS: CW/CCW both-colour storage and release, two CCW behind seats, front-only prerequisites; 2400 C++/Python footprint cases agree')


if __name__=='__main__':
    main()
