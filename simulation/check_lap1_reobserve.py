"""Execute actual lap1 recovery states across soft drift and hard faults.

Motion geometry is controlled here; existing swept-geometry checks remain
separate. This verifies dispatch, braking, fresh frames and safe-resume gates.
"""
import subprocess
import shutil
from check_connector_servo_resume import ROOT, block


def main():
    path=(ROOT/'src/obstacle_path.cpp').read_text()
    source=r'''
#include <cmath>
#include <cstdint>
#include <iostream>
#include <cassert>
#include <algorithm>
using std::isfinite;
constexpr float PI=3.14159265358979323846f;
constexpr int COURSE_SEATS_PER_STATION=2;
constexpr int OBSTACLE_SEAT_COUNT=24;
constexpr float OBSTACLE_DISCOVERY_VIEW_MIN_MM=220;
constexpr int OBSTACLE_CORNER_VIEW_REVERSE_SPEED_MM_S=60;
struct PositionEstimate{float x_mm=0,y_mm=0,heading_deg=0;};
struct DiscoveryStation{uint8_t clearFrames[2]={};bool seatObservedClear[2]={};};
DiscoveryStation discoveryStations[12];
''' + path[path.index('enum CornerViewPhase :'):path.index('PathPoint\nfindLookahead',path.index('enum CornerViewPhase :'))] + r'''
uint32_t now=1000;float encoder=0;PositionEstimate pose;
int speed=0,steering=0,collected=0,preflights=0,resumes=0;
bool locked=false,resolved=false,resumeSafe=false,arcSafe=false,commandSafe=true;
bool servo_disabled=true,discoveryHolding=true,discoveryBlocked=false;
int discoveryHoldStation=3,discoveryBlockedStation=-1,progressIndex=0;
uint32_t discoveryHoldStartMs=0,lastDiscoveryNudgeUpdateMs=0;
float lastDiscoveryTargetNudgeDeg=0;
struct Log{template<class T>void print(T){}template<class T>void println(T){}
 template<class T>void print(T,int){}template<class T>void println(T,int){}}Serial;
uint32_t millis(){return now;}uint32_t micros(){return now*1000;}
float get_distance(){return encoder;}PositionEstimate get_position_struct(){return pose;}
float wrap180(float a){while(a>180)a-=360;while(a<-180)a+=360;return a;}
void holdCornerViewCentered(){speed=0;steering=0;}
void holdCornerViewSteering(int s){speed=0;steering=s;}
void set_steering(int s){steering=s;}void set_speed(int s){speed=s;}
void lockCornerView(const char*){locked=true;speed=0;cornerViewPhase=CORNER_VIEW_LOCKED;}
void logCornerViewPose(const char*,const PositionEstimate&,float){}
bool stationDirectlyResolved(uint8_t){return resolved;}
bool discoveryResumeFromViewSafe(const PositionEstimate&,uint16_t&i){++resumes;i=8;return resumeSafe;}
void collectCornerViewFrame(const PositionEstimate&,bool fresh){if(fresh){++collected;++cornerViewStoppedFrames;}}
int getLargestObstacle(){return 0;}float cornerViewMinimumRange(int){return 220;}
bool obstacle_blob_valid_for_acquisition(int){return false;}
float cornerViewReverseDistance(const PositionEstimate&,uint8_t,float){return 0;}
bool cornerViewSweepSafe(const PositionEstimate&,float){return false;}
int cornerViewStraightSteering(const PositionEstimate&,int){return 0;}
bool cornerViewSteeredCommandSafe(const PositionEstimate&,int,int){++preflights;return commandSafe;}
PositionEstimate cornerViewArcPose(PositionEstimate p,int,float){return p;}
bool cornerViewArcSafe(const PositionEstimate&,int,float){++preflights;return arcSafe;}
bool beginCornerViewExtra(const PositionEstimate&){++preflights;return false;}
''' + block(path,path.index('void reobserveCornerView('))+'\n' + block(path,path.index('bool resumeDiscoveryFromView('))+'\n' + block(path,path.index('void updateCornerViewExtra('))+'\n' + block(path,path.index('void updateCornerView(bool'))+r'''
void reset(){
 pose={};encoder=0;cornerViewOrigin={};cornerViewOriginEncoder=0;
 cornerViewStation=3;cornerViewPhase=CORNER_VIEW_REVERSE;cornerViewPhaseMs=now;
 cornerViewDistanceMm=90;cornerViewMeasuredReverseMm=0;
 cornerViewStoppedFrames=0;locked=resolved=resumeSafe=arcSafe=false;
 commandSafe=true;speed=0;collected=preflights=resumes=0;
}
int main(){int cases=0;
 for(float yaw:{-11.f,-6.f,6.f,11.f})for(float cross:{-24.f,-16.f,0.f,16.f,24.f}){
  reset();pose={-30,cross,yaw};encoder=-30;
  updateCornerView(true);assert(!locked&&speed==0&&cornerViewPhase==CORNER_VIEW_REBASE_BRAKE);
  now+=299;updateCornerView(true);assert(cornerViewPhase==CORNER_VIEW_REBASE_BRAKE);
  now+=1;updateCornerView(true);assert(cornerViewPhase==CORNER_VIEW_OBSERVE&&!locked);
  // No safe movement: continue collecting independent frames, never certify
  // empty or return to driving merely because an observation timed out.
  now+=2000;for(int i=0;i<6;++i){now+=80;updateCornerView(true);}
  assert(collected==6&&preflights>0&&!locked&&speed==0&&cornerViewPhase==CORNER_VIEW_OBSERVE);
  resolved=true;resumeSafe=false;updateCornerView(true);
  assert(!locked&&cornerViewPhase==CORNER_VIEW_OBSERVE&&resumes>0);
  resumeSafe=true;updateCornerView(true);
  assert(!locked&&cornerViewPhase==CORNER_VIEW_IDLE&&progressIndex==8&&!discoveryBlocked);
  ++cases;
 }
 for(auto p:{PositionEstimate{0,26,0},PositionEstimate{0,0,13},PositionEstimate{NAN,0,0}}){
  reset();pose=p;updateCornerView(true);assert(locked&&speed==0);++cases;
 }
 reset();encoder=6;updateCornerView(true);assert(locked);++cases; // wrong direction
 reset();now+=6001;updateCornerView(true);assert(locked);++cases; // motion timeout
 reset();commandSafe=false;updateCornerView(true);
 assert(!locked&&cornerViewPhase==CORNER_VIEW_REBASE_BRAKE&&speed==0);++cases;
 reset();cornerViewPhase=CORNER_VIEW_EXTRA_SETTLE;cornerViewExtraSteering=20;
 now+=301;updateCornerView(true);
 assert(!locked&&cornerViewPhase==CORNER_VIEW_REBASE_BRAKE&&speed==0);++cases;
 reset();cornerViewPhase=CORNER_VIEW_EXTRA_RETURN_SETTLE;cornerViewExtraStartEncoder=0;
 encoder=-90;cornerViewExtraOrigin=pose;now+=301;updateCornerView(true);
 assert(!locked&&cornerViewPhase==CORNER_VIEW_REBASE_BRAKE&&speed==0);++cases;
 reset();cornerViewPhase=CORNER_VIEW_EXTRA_RETURN_SETTLE;cornerViewExtraStartEncoder=0;
 encoder=-115;cornerViewExtraOrigin=pose;now+=301;updateCornerView(true);
 assert(locked&&speed==0);++cases;
 std::cout<<"LAP1 REOBSERVE "<<cases<<" PASS: soft mismatch brakes/searches; unresolved frames continue; unsafe join/hard faults stay stopped\n";
}
'''
    out=ROOT/'local_workspace/lap1-reobserve';out.mkdir(parents=True,exist_ok=True)
    cpp,exe=out/'check.cpp',out/'check.exe';cpp.write_text(source)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2',str(cpp),'-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],text=True,capture_output=True)
    (out/'output.txt').write_text(result.stdout+result.stderr);print(result.stdout,end='')
    if result.returncode:print(result.stderr,end='')
    raise SystemExit(result.returncode)


if __name__=='__main__':main()
