"""Source-backed start-scout transition with a falsely confirmed middle seat.

Reproduces576's inference gap; hardware/clearance decisions stubbed here.
Full physical geometry remains covered by the start planners.
"""
from pathlib import Path
import shutil
import subprocess
from check_connector_servo_resume import ROOT, block


def main():
    path=(ROOT/'src/obstacle_path.cpp').read_text()
    code=r'''
#include <cstdint>
#include <iostream>
constexpr int OBSTACLE_SEAT_COUNT=24,COURSE_SEATS_PER_STATION=2,
 COURSE_STATIONS_PER_SECTION=3,OBSTACLE_PARKING_EXIT_STEERING=50,
 PARKING_ENTRY_SCOUT_STEER_SETTLE=0;
struct PositionEstimate {float x_mm=0,y_mm=0,heading_deg=0;};
struct Seat {float y=0;bool confirmed=false;};Seat seats[24];
struct Discovery {bool observedClear=false;};Discovery discoveryStations[12];
struct Log {template<class T>void print(T){}template<class T>void println(T){}} Serial;
bool parkingCcwShortStart=false,parkingEntryPrimaryRetryUsed=false,
 parkingEntryTestHold=false,parkingEntryScouting=false,servo_disabled=true;
int parkingEntryTargetStation=1,parkingEntryScoutStation=-1,parkingEntryScoutPhase=0,
 discoveryScanStation=-1,discoveryScanSide=-1,routeTurnSign=-1,joins=0,retries=0;
float parkingEntryScoutTargetMm=0,parkingEntryScoutStartEncoderDistance=0;
uint32_t parkingEntryScoutPhaseStartMs=0;PositionEstimate parkingEntryScoutReturnPose;
float get_distance(){return 42;}uint32_t millis(){return 1000;}
float parkingEntryScoutArcMm(){return 130;}
void set_steering(int){}void steer(int){}
void armParkingEntryConnectorFromPose(const PositionEstimate &){++joins;}
void startParkingEntryPrimaryRetry(){++retries;}
bool preflightParkingEntryScout(const PositionEstimate &,uint8_t){return true;}
uint8_t sectionInferredEmpty(uint8_t section){return section==0&&seats[2].confirmed?5:0;}
'''
    for sig in ('bool stationDirectlyResolved(', 'bool stationResolved(', 'void startParkingEntryScout('):
        code+=block(path,path.index(sig))+'\n'
    code+=r'''
int main(){
 seats[0].y=-900;seats[1].y=-1100;seats[2].confirmed=true;
 if(!stationResolved(0)||stationDirectlyResolved(0))return 1;
 startParkingEntryScout(PositionEstimate{308.1f,-1209.7f,148.5f});
 if(!parkingEntryScouting||joins||parkingEntryTestHold||discoveryScanStation!=0||discoveryScanSide!=0){
  std::cout<<"576 skipped scout reproduced joins="<<joins<<"\n";return 2;}
 // Independently observed clear or confirmed near pillar may still skip it.
 for(int evidence=0;evidence<3;++evidence){
  parkingEntryScouting=false;joins=0;
  discoveryStations[0].observedClear=evidence==0;
  seats[0].confirmed=evidence==1;seats[1].confirmed=evidence==2;
  startParkingEntryScout(PositionEstimate{});
  if(parkingEntryScouting||joins!=1)return 3;
 }
 // Missing primary evidence must arm a fresh observation, including a
 // defensive transition where no previous retry flag was set.
 for(bool ccw:{false,true}){
  for(auto &s:seats)s.confirmed=false;
  discoveryStations[0].observedClear=true;
  parkingCcwShortStart=ccw;parkingEntryPrimaryRetryUsed=false;
  parkingEntryTestHold=false;retries=joins=0;
  startParkingEntryScout(PositionEstimate{});
  if(parkingEntryTestHold||retries!=1||joins||!parkingEntryPrimaryRetryUsed)return 4;
 }
 std::cout<<"576 inferred-empty transition scouts; direct-clear/confirmed bypasses; missing primary retries PASS\n";
}
'''
    dest=ROOT/'local_workspace/cw-576';dest.mkdir(parents=True,exist_ok=True)
    for name,source,expected in (
        ('scout',code,0),
        ('old_inference_gap',code.replace(
            'if (stationDirectlyResolved(static_cast<uint8_t>(parkingEntryScoutStation)))',
            'if (stationResolved(static_cast<uint8_t>(parkingEntryScoutStation)))'),2)):
        cpp,exe=dest/(name+'.cpp'),dest/(name+'.exe');cpp.write_text(source)
        subprocess.run([shutil.which('g++'),'-std=c++17','-O2',str(cpp),'-o',str(exe)],check=True)
        result=subprocess.run([str(exe)],capture_output=True,text=True)
        (dest/(name+'.txt')).write_text(result.stdout+result.stderr)
        print(result.stdout,end='');assert result.returncode==expected
    print('576 removed-direct-evidence negative control reproduces skipped scout')


if __name__=='__main__':main()
