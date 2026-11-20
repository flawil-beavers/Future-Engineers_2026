"""Measured 98/35/25/37mm parking envelope and mirrored normal entry.
Independent-radius plant; does not reproduce unknown traction or camera truth.
--accepted-range-stress covers the entire scan tolerance at the current capture;
this intentionally reports unresolved narrow-gap cases as failures.
"""
import sys
from check_final_parking_sequence import fixture_source
from check_returned_603_and_parking_practice import run

source=fixture_source()+r'''
#include <cassert>
using namespace parking_module;
int main(){
 static_assert(PHYSICAL_WHEELBASE_MM==98);
 static_assert(OBSTACLE_FINAL_PARKING_REAR_OVERHANG_MM==35);
 static_assert(OBSTACLE_FINAL_PARKING_FRONT_OVERHANG_MM==123);
 static_assert(OBSTACLE_FINAL_PARKING_STEERED_FRONT_MM==135);
 for(int steer:{-50,0,50}){
  parking_start_footprint::Quad q[6];
  parking_start_footprint::robotQuads(0,0,0,steer>0?1:(steer<0?-1:0),0,q,
   123,35,98,123,135,Ackermann::getLeftWheelAngleLUT(steer),Ackermann::getRightWheelAngleLUT(steer));
  float front=-INFINITY,rear=INFINITY;
  for(auto &body:q)for(auto &p:body.p){front=fmaxf(front,p.x);rear=fminf(rear,p.x);}
  assert(fabsf(front-(steer?135:123))<.001f && fabsf(rear+35)<.001f);
 }
 // Legacy callers keep their established exit/lap contours.
 for(int sign:{-1,0,1})for(float h:{0.f,45.f,180.f}){
  parking_start_footprint::Quad a[6],b[6];
  parking_start_footprint::robotQuads(300,-1200,h,sign,3,a);
  parking_start_footprint::robotQuads(300,-1200,h,sign,3,b,125,40,100,125,0);
  assert(memcmp(a,b,sizeof(a))==0);
 }
 // Tight rescue must still reject the measured extended steering nose.
 for(int dir:{-1,1}){
  ::routeTurnSign=dir;resetState();baseline();for(auto&s:seats)s.confirmed=s.injected=false;
  parking_module::routeTurnSign=dir;scanPhase=5;measuredGapMm=248.9f;
  PositionEstimate p;localToField(121,125,0,p.x_mm,p.y_mm,p.heading_deg);
  assert(parkingEntryPoseSafe(p,0,1));
  assert(!parkingEntryPoseSafe(p,-50,1)&&!parkingEntryPoseSafe(p,50,1));
 }
 int total=0,bad=0;float maxTravel=0;
 for(int dir:{-1,1})for(float gap:{248.9f,249.6f})for(float scale:{1.f,1.15f,1.5f})for(float yaw:{-1.f,1.f}){
  ::routeTurnSign=dir;resetState();baseline();for(auto&s:seats)s.confirmed=s.injected=false;
  final_parking_reset();parking_module::routeTurnSign=dir;scanPhase=5;measuredGapMm=gap;
  localToField(OBSTACLE_FINAL_PARKING_CAPTURE_LOCAL_X_MM,OBSTACLE_FINAL_PARKING_CAPTURE_LOCAL_Y_MM,yaw,
   hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);truePose=hostPose;
  hostTime=1000;encoder=0;hostGyroHealthy=true;framePeriod=30;stop(false);
  robot_logger.writes=0;robot_logger.lines.str("");robot_logger.lines.clear();
  state=FP_REPLAN_BRAKE;stateStartMs=0;entryReplanReason="measured_geometry";
  float travel=0;bool collision=false;
  for(int tick=0;tick<6000&&!final_parking_complete()&&!final_parking_aborted();++tick){
   final_parking_update(dir);
   if(dc_state==DC_ENABLED){
    float remaining=fabsf(command*.02f),sign=command<0?-1.f:1.f;
    while(remaining>.0001f){float ds=sign*fminf(1,remaining);remaining-=fabsf(ds);encoder+=ds;travel+=fabsf(ds);
     advanceEntryModel(hostPose,steeringCommand,ds,scale);truePose=hostPose;
     if(!parkingEntryPoseSafe(truePose,steeringCommand,0))collision=true;
    }
   }hostTime+=20;
  }
  bool ok=final_parking_complete()&&finalFootprintContained()&&!collision&&robot_logger.writes==1;
  ++total;bad+=!ok;maxTravel=fmaxf(maxTravel,travel);
  auto p=entryLocalPose(hostPose);
  std::cout<<"MEASURED "<<dir<<" gap/radius/yaw="<<gap<<"/"<<scale<<"/"<<yaw<<" success="<<ok<<" collision="<<collision<<" end="<<p.x_mm<<"/"<<p.y_mm<<"/"<<p.heading_deg<<" travel="<<travel<<" ms="<<hostTime-1000<<"\n";
  if(!ok)std::cout<<robot_logger.lines.str();
 }
 std::cout<<"MEASURED PARKING "<<total<<" FAILED "<<bad<<" max_travel="<<maxTravel<<"\n";
 return bad?1:0;
}
'''
stress='--accepted-range-stress' in sys.argv
if stress:
    source=source.replace('for(float gap:{248.9f,249.6f})',
                          'for(float gap:{232.5f,242.5f,247.5f,252.5f,262.5f})')
    source=source.replace('for(float scale:{1.f,1.15f,1.5f})',
                          'for(float scale:{.95f,1.f,1.05f,1.1f,1.15f,1.5f})')
without_overlap='--without-brake-servo-overlap' in sys.argv
if without_overlap:
    source=source.replace('            prepositionEntrySteeringDuringBrake(pose);','')
run('parking-without-brake-servo-overlap' if without_overlap else
    'parking-current-capture-range' if stress else 'parking-measured-651-656',source)
