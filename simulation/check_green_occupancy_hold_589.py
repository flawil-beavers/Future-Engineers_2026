"""Production clear-evidence grace for the one-frame GREEN candidate in log589."""

import shutil
import subprocess

from check_connector_servo_resume import ROOT, block


def main():
    source = (ROOT / "src/obstacle_path.cpp").read_text()
    code = r'''
#include <cstdint>
#include <cmath>
#include <cstring>
#include <iostream>
constexpr int A0=0,A1=1,A2=2;
constexpr float PI=3.14159265358979323846f;
#include "config.h"
#include "course_map.h"
#include "obstacle_path.h"
#include "position_estimator.h"
using std::isfinite;
struct DiscoveryStation {
 uint8_t clearFrames[COURSE_SEATS_PER_STATION]={};
 bool seatObservedClear[COURSE_SEATS_PER_STATION]={};
 uint8_t lastClearEvidenceMask=0;bool observedClear=false;
};
DiscoveryStation discoveryStations[OBSTACLE_SEAT_COUNT/2];
struct CandidateSeat {float x=0,y=0;} seats[OBSTACLE_SEAT_COUNT];
bool greenSeatCandidateThisFrame[OBSTACLE_SEAT_COUNT]={};
uint32_t greenSeatCandidateLastSeenMs[OBSTACLE_SEAT_COUNT]={};
uint8_t completedLaps=0;bool parkingEntryActive=true,parkingEntryObserving=true;
int8_t parkingEntryTargetStation=2;uint32_t clockMs=1000;
uint32_t millis(){return clockMs;}
bool stationResolved(uint8_t){return false;}
bool seatComfortablyVisible(uint8_t seat,const PositionEstimate &){return seat==5;}
void seatCameraGeometry(uint8_t,const PositionEstimate &,float &bearing,float &range){bearing=0;range=309;}
bool rejectedGreenBehindSeat(const Blob *,float,float){return false;}
bool unmappedForegroundBlocksClear(const Blob *,const ObstacleObservationResult &,float,float){return false;}
bool rejectedBlobBlocksSeatClear(const Blob *,float){return false;}
bool observationAllowsClearAtGeometry(const ObstacleObservationResult &,float,float){return true;}
void logParkingStartClearImage(uint8_t,const PositionEstimate &){}
uint8_t sectionInferredEmpty(uint8_t){return 0;}
void course_map_record_clear_station(uint8_t,uint8_t,float,float,float,float){}
'''
    code += block(source, source.index("void updateDiscoveryCoverage("))
    code += r'''
int main(){
 PositionEstimate pose{};ObstacleObservationResult observation{};
 greenSeatCandidateLastSeenMs[5]=1000;
 // Log589 changed the seat to CLEAR after 192 ms. Five nominal 76 ms frames
 // must still be unable to clear it inside the production 600 ms hold.
 for(int i=0;i<5;++i){clockMs=1192+76*i;updateDiscoveryCoverage(observation,pose,nullptr);}
 if(discoveryStations[2].seatObservedClear[1]||discoveryStations[2].clearFrames[1])return 1;
 // The hold is finite: after expiry, five independent clear frames work normally.
 for(int i=0;i<5;++i){clockMs=1700+76*i;updateDiscoveryCoverage(observation,pose,nullptr);}
 if(!discoveryStations[2].seatObservedClear[1]||discoveryStations[2].clearFrames[1]<5)return 2;
 std::cout<<"LOG589 OCCUPANCY HOLD PASS: no 192ms false CLEAR; normal five-frame clear resumes after 600ms\n";
}
'''
    out = ROOT / "local_workspace" / "green-occupancy-hold-589"
    out.mkdir(parents=True, exist_ok=True)
    cpp, exe = out / "check.cpp", out / "check.exe"
    cpp.write_text(code)
    subprocess.run([
        shutil.which("g++"), "-std=c++17", "-O2",
        "-I", str(ROOT / "simulation" / "host"),
        "-I", str(ROOT / "include"), str(cpp), "-o", str(exe)
    ], check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
