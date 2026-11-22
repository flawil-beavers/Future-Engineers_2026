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
struct NullLog{template<class T>void print(T){} template<class T>void print(T,int){}
template<class T>void println(T){} template<class T>void println(T,int){}} Serial;
struct DiscoveryStation {
 uint8_t clearFrames[COURSE_SEATS_PER_STATION]={};
 bool seatObservedClear[COURSE_SEATS_PER_STATION]={};
 uint8_t lastClearEvidenceMask=0;bool observedClear=false;
};
DiscoveryStation discoveryStations[OBSTACLE_SEAT_COUNT/2];
struct CandidateSeat {
 float x=0,y=0,headingDeg=0;
 uint32_t lastVoteMs=0;
 bool confirmed=false;
} seats[OBSTACLE_SEAT_COUNT];
bool greenSeatCandidateThisFrame[OBSTACLE_SEAT_COUNT]={};
uint32_t greenSeatCandidateLastSeenMs[OBSTACLE_SEAT_COUNT]={};
uint8_t completedLaps=0;bool parkingEntryActive=true,parkingEntryObserving=true;
int8_t routeTurnSign=1;
bool parkingEntryScouting=false;
constexpr int CORNER_VIEW_IDLE=0;int cornerViewPhase=0,cornerViewStation=-1;
int8_t parkingEntryTargetStation=2;uint32_t clockMs=1000;
uint32_t millis(){return clockMs;}
bool stationResolved(uint8_t){return false;}
int visibleSeat=5;float visibleRange=309, visibleBearing=0.0f;
bool seatComfortablyVisible(uint8_t seat,const PositionEstimate &){return seat==visibleSeat;}
void seatCameraGeometry(uint8_t,const PositionEstimate &,float &bearing,float &range){bearing=visibleBearing;range=visibleRange;}
bool unmappedForegroundBlocksClear(const Blob *raw,const ObstacleObservationResult &,float,float){return raw!=nullptr;}
bool rejectedBlobBlocksSeatClear(const Blob *raw,float){return raw!=nullptr;}
bool observationAllowsClearAtGeometry(const ObstacleObservationResult &,float,float){return true;}
void logParkingStartClearImage(uint8_t,const PositionEstimate &){}
uint8_t sectionInferredEmpty(uint8_t){return 0;}
void course_map_record_clear_station(uint8_t,uint8_t,float,float,float,float){}
'''
    code += block(source, source.index("float wrap180("))
    code += block(source, source.index("bool rejectedGreenBehindSeat("))
    code += block(source, source.index("bool confirmedPillarOccludesSeat("))
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
 visibleSeat=9;visibleRange=422;visibleBearing=21.7f;parkingEntryActive=parkingEntryObserving=false;
 for(int i=0;i<10;++i){clockMs+=80;updateDiscoveryCoverage(observation,pose,nullptr);}
 if(discoveryStations[4].seatObservedClear[1]||discoveryStations[4].clearFrames[1])return 16;
 visibleRange=340;
 for(int i=0;i<10;++i){clockMs+=80;updateDiscoveryCoverage(observation,pose,nullptr);}
 if(!discoveryStations[4].seatObservedClear[1])return 17;
 discoveryStations[4]={};visibleBearing=0.0f;
 std::cout<<"LOG41 CCW first middle empty waits for the nearer view PASS\n";
 visibleSeat=6;visibleRange=600;parkingEntryActive=parkingEntryObserving=false;
 for(int i=0;i<10;++i){clockMs+=80;updateDiscoveryCoverage(observation,pose,nullptr);}
 if(discoveryStations[3].seatObservedClear[0])return 3;
 visibleRange=450;
 for(int i=0;i<10;++i){clockMs+=80;updateDiscoveryCoverage(observation,pose,nullptr);}
 if(!discoveryStations[3].seatObservedClear[0])return 4;
 visibleSeat=8;visibleRange=567;
 for(int i=0;i<10;++i){clockMs+=80;updateDiscoveryCoverage(observation,pose,nullptr);}
 if(discoveryStations[4].seatObservedClear[0])return 5;
 visibleSeat=9;visibleRange=503.6f;
 for(int i=0;i<10;++i){clockMs+=80;updateDiscoveryCoverage(observation,pose,nullptr);}
 if(discoveryStations[4].seatObservedClear[1])return 14;
 for(int seat:{8,9}){
  visibleSeat=seat;visibleRange=440;
  for(int i=0;i<4;++i){clockMs+=80;updateDiscoveryCoverage(observation,pose,nullptr);}
 }
 if(!discoveryStations[4].observedClear)return 15;
 std::cout<<"LOG20 EMPTY PASS: far middle views rejected; nearby views can clear\n";
 visibleSeat=18;visibleRange=279;parkingEntryActive=parkingEntryObserving=parkingEntryScouting=false;
 seats[18].headingDeg=270;pose.heading_deg=303.9;
 for(int i=0;i<10;++i){clockMs+=80;updateDiscoveryCoverage(observation,pose,nullptr);}
 if(discoveryStations[9].seatObservedClear[0])return 6;
 pose.heading_deg=280;
 for(int i=0;i<3;++i){clockMs+=80;updateDiscoveryCoverage(observation,pose,nullptr);}
 if(!discoveryStations[9].seatObservedClear[0])return 7;
 std::cout<<"LOG3 CLEAR PASS: oblique corner absence rejected; aligned clear view accepted\n";
 // An occluded seat remains unresolved even with consecutive no-blob frames.
 // Move the mapped foreground pillar off the ray, and test pending vote expiry.
 visibleSeat=8;visibleRange=430;pose.heading_deg=0;discoveryStations[4]={};
 seats[8].x=500;seats[6].x=250;seats[6].confirmed=true;
 for(int i=0;i<3;++i){clockMs+=80;updateDiscoveryCoverage(observation,pose,nullptr);}
 if(discoveryStations[4].clearFrames[0])return 8;
 seats[6].y=70;seats[8].lastVoteMs=clockMs;
 for(int i=0;i<3;++i){clockMs+=80;updateDiscoveryCoverage(observation,pose,nullptr);}
 if(discoveryStations[4].clearFrames[0])return 9;
 clockMs=seats[8].lastVoteMs+OBSTACLE_SEAT_VOTE_WINDOW_MS+1;
 for(int i=0;i<2;++i){clockMs+=80;updateDiscoveryCoverage(observation,pose,nullptr);}
 if(!discoveryStations[4].seatObservedClear[0])return 10;
 std::cout<<"PENDING VOTE AND MAPPED OCCLUSION PASS\n";
 // Log18 at the measured lookout. The wide blob crosses the ray, but
 // the ground foot belongs well behind the near 438.5 mm seat.
 visibleSeat=6;visibleRange=438.5f;pose.heading_deg=62.8f;seats[6].headingDeg=90.0f;
 discoveryStations[3]={};seats[6].confirmed=false;
 Blob background{};background.found=true;background.color=ColorType::GREEN;
 background.minX=188;background.maxX=242;background.minY=88;background.maxY=120;
 if(!rejectedGreenBehindSeat(&background,-7.24f,438.5f))return 11;
 for(int i=0;i<6;++i){clockMs+=80;updateDiscoveryCoverage(observation,pose,&background);}
 if(!discoveryStations[3].seatObservedClear[0])return 12;
 // Same colour and horizontal span with a near ground foot must veto EMPTY.
 discoveryStations[3]={};background.maxY=145;
 for(int i=0;i<6;++i){clockMs+=80;updateDiscoveryCoverage(observation,pose,&background);}
 if(discoveryStations[3].clearFrames[0])return 13;
 std::cout<<"LOG18 DEPTH PASS: distant green clears, near green still vetoes EMPTY\n";

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
