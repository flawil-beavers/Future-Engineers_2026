"""Production start/exit commands and motor ramps at their stopping gates."""
from check_connector_servo_resume import block
from check_cw_start_planner import ROOT
from check_returned_603_and_parking_practice import run

obstacle = (ROOT/'src/obstacle.cpp').read_text()
path = (ROOT/'src/obstacle_path.cpp').read_text()
exit_speed = block(obstacle, obstacle.index('static int parkingExitSegmentSpeed('))
scan_speeds = '\n'.join(block(path, path.index(signature)) for signature in (
    'int parkingEntryScoutMotionSpeed(', 'int parkingEntryScanMotionSpeed('))
at = obstacle.index('    set_speed(segment.direction * parkingExitSegmentSpeed(')
command = obstacle[at:obstacle.index(';', at)+1]
run('exit-long-arc-speed', r'''
#include <cassert>
#include <cstdint>
#include <iostream>
constexpr int A0=0,A1=1,A2=2;
constexpr float PI=3.14159265358979323846f;
#include "drive_profile.h"
bool parkingCwShortStart=true,shortStart=true;
int routeTurnSign=-1;
bool parkingShortStart(){return shortStart;}
'''+exit_speed+scan_speeds+r'''
int commanded=0,oc_parking_exit_segment=0;
struct Segment {int direction;} segment;
void set_speed(int value){commanded=value;}
void update(float speedEnd,float stateDistance){
'''+command+r'''
}
struct Motion {float seconds,endSpeed,peak;};
template<class Target>Motion drive(float distance,float dt,int direction,Target command){
 float travel=0,speed=0,accel=0,limit=0,t=0,peak=0;
 while(travel<distance && t<6){
  const int target=direction*command(travel);
  travel+=direction*DriveProfile::advance(speed,accel,limit,target,dt,DEFAULT_ACCELERATION);
  peak=fmaxf(peak,fabsf(speed));t+=dt;
 }
 assert(travel>=distance);
 return {t,fabsf(speed),peak};
}
int main(){int exitChecks=0,scanChecks=0,scoutChecks=0;
 float exitGain=0,scanGain=0,scoutGain=0,maxScanEnd=0,maxScoutEnd=0;
 const float lengths[]={30,36,26,89,OBSTACLE_PARKING_EXIT_FINAL_ALIGN_MIN_MM};
 for(int dir:{-1,1})for(int i=0;i<5;++i)for(float dt:{.010f,.020f,.030f,.040f,.050f}){
  oc_parking_exit_segment=i;segment.direction=dir;
  const int terminal=i<3?80:100;
  const auto slow=drive(lengths[i],dt,dir,[&](float){return terminal;});
  const auto fast=drive(lengths[i],dt,dir,[&](float at){
   update(lengths[i],at);return dir*commanded;
  });
  assert(fast.endSpeed<=terminal+2 && fast.seconds<=slow.seconds+dt+.001f);
  exitGain=fmaxf(exitGain,slow.seconds-fast.seconds);++exitChecks;
 }
 for(int length=40;length<=84;++length)for(float dt:{.010f,.020f,.030f,.040f,.050f}){
  const auto slow=drive(length-3,dt,-1,[](float){return 60;});
  const auto fast=drive(length-3,dt,-1,[&](float at){return parkingEntryScanMotionSpeed(length-at);});
  assert(fast.endSpeed<=62 && fast.seconds<=slow.seconds+dt+.001f);
  scanGain=fmaxf(scanGain,slow.seconds-fast.seconds);maxScanEnd=fmaxf(maxScanEnd,fast.endSpeed);++scanChecks;
 }
 for(int length=115;length<=165;++length)for(float dt:{.010f,.020f,.030f,.040f,.050f})for(int dir:{-1,1}){
  const auto slow=drive(length,dt,dir,[&](float at){return DriveProfile::approachSpeed(length-at,100,60,32,180);});
  const auto fast=drive(length,dt,dir,[&](float at){return parkingEntryScoutMotionSpeed(length,at);});
  assert(fast.endSpeed<=62 && fast.seconds<=slow.seconds+dt+.001f);
  scoutGain=fmaxf(scoutGain,slow.seconds-fast.seconds);maxScoutEnd=fmaxf(maxScoutEnd,fast.endSpeed);++scoutChecks;
 }
 shortStart=false;assert(parkingEntryScanMotionSpeed(84)==60);
 routeTurnSign=1;assert(parkingEntryScoutMotionSpeed(130,0)==60);
 std::cout<<"START MOTION exit/scan/scout "<<exitChecks<<"/"<<scanChecks<<"/"<<scoutChecks
  <<" PASS; max savings s "<<exitGain<<"/"<<scanGain<<"/"<<scoutGain
  <<"; max terminal scan/scout "<<maxScanEnd<<"/"<<maxScoutEnd<<" mm/s\n";
}
''')

tracking = obstacle[obstacle.index('struct ParkingExitYawProgress'):
                    obstacle.index('// Steering is expressed relative')]
sweep = block(obstacle, obstacle.index('static bool parkingExitMotionSafe('))
update = obstacle[obstacle.index('    const float stateDistance =\n',
                                 obstacle.index('    const ParkingExitSegment &segment =\n',
                                                obstacle.index('static bool updateParkingExit()'))):
                  obstacle.index('\nstatic WallSide wallForColor')]
run('exit-slip-reserve', r'''
#include <cassert>
#include <cstdint>
#include <iostream>
constexpr int A0=0,A1=1,A2=2;
constexpr float PI=3.14159265358979323846f;
#include "drive_profile.h"
#include "parking_start_footprint.h"
struct PositionEstimate {float x_mm,y_mm,heading_deg;};
#include "ackermann_kinematics.h"
'''+exit_speed+sweep+tracking+r'''
enum {PARKING_EXIT_SEGMENT_DRIVE,PARKING_EXIT_SEGMENT_BRAKE};
int command=0,oc_parking_exit_state=0;
uint8_t oc_parking_exit_segment=4;
uint32_t hostTime=10000,oc_parking_exit_brake_start_ms=0;
float oc_parking_exit_state_distance=0,oc_parking_exit_start_heading=0;
float travel=240,gyroAngle=0;
bool servo_disabled=false,gyroHealthy=true;
PositionEstimate pose{};
struct Log {template<class T>void print(T){} template<class T>void print(T,int){}
 template<class T>void println(T){} template<class T>void println(T,int){}} Serial;
struct Segment {int direction;float distanceMm;} segment{1,175};
float distanceSince(float start){return travel-start;}
float get_angle(){return gyroAngle;}
uint32_t millis(){return hostTime;}
bool gyro_is_healthy(){return gyroHealthy;}
PositionEstimate get_position_struct(){return pose;}
void set_steering(int){} void steer(int){}
void set_speed(int speed){command=speed;}
void stop(bool){command=0;}
bool update(int segmentSteering){
'''+update+r'''
void setCase(int direction, float error, float x, float y){
 hostTime=10000;travel=240;gyroAngle=direction*error;
 const float base=direction<0?180.0f:0.0f;
 pose={x,y,base+gyroAngle};
 parkingExitYawProgress={70,error+0.8f,hostTime-100,0,false};
 oc_parking_exit_state=PARKING_EXIT_SEGMENT_DRIVE;
 gyroHealthy=true;
}
int main(){
 int checked=0;
 for(int direction:{-1,1}){
  // Tail slopes bracket the last moving slices of CW 642, 638 and 644.
  for(float error:{9.5f,11.5f,12.8f}){
   setCase(direction,error,direction<0?186.0f:527.0f,-1146);
   const float step=error<12.0f?0.78f:0.95f;
   while(fabsf(gyroAngle)>2.0f){
    update(50*direction);assert(command==60&&parkingExitYawProgress.reserveActive);
    const float h=pose.heading_deg*PI/180.0f;
    pose.x_mm+=3.0f*cosf(h);pose.y_mm+=3.0f*sinf(h);
    gyroAngle-=direction*step;pose.heading_deg-=direction*step;
    travel+=3;hostTime+=75;
   }
   assert(travel<285);update(50*direction);
   assert(command==0&&oc_parking_exit_state==PARKING_EXIT_SEGMENT_BRAKE);++checked;
  }
  for(int fault=0;fault<7;++fault){
   setCase(direction,10.0f,direction<0?186.0f:527.0f,-1146);
   if(fault==0){parkingExitYawProgress.bestErrorDeg=9.8f;parkingExitYawProgress.lastProgressMs=hostTime-700;}
   if(fault==1){parkingExitYawProgress.bestErrorDeg=8.5f;}
   if(fault==2){gyroAngle=direction*20.7f;parkingExitYawProgress.bestErrorDeg=21.5f;}
   if(fault==3){travel=285;}
   if(fault==4){pose.y_mm=-1495;}
   if(fault==5){
    parkingExitYawProgress.reserveActive=true;
    parkingExitYawProgress.reserveStartMs=hostTime-1600;
    travel=239.5f;
   }
   if(fault==6){gyroHealthy=false;}
   update(50*direction);
   assert(command==0&&oc_parking_exit_state==PARKING_EXIT_SEGMENT_BRAKE);++checked;
  }
 }
 std::cout<<"EXIT SLIP RESERVE "<<checked<<" checks PASS\n";
}
''')
