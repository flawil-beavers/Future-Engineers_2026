"""Measured123mm, motion-only nose reserve and unchanged parked-X acceptance."""
from check_final_parking_sequence import ROOT, fixture_source
from check_connector_servo_resume import block
from check_returned_603_and_parking_practice import run


def main():
    source=(ROOT/'src/final_parking.cpp').read_text()
    control=block(source,source.index('bool parkingEntryPoseSafe('))
    control=control.replace('bool parkingEntryPoseSafe(', 'bool noFrontReserveSafe(')
    control=control.replace('OBSTACLE_FINAL_PARKING_STEER_FRONT_RESERVE_MM','0.0f')
    run('123mm-front-reserve',fixture_source()+'\nnamespace parking_module {\n'+control+'\n}\n'+r'''
using namespace parking_module;
int main(){int checks=0;
 parking_start_footprint::Quad q[6];
 for(float width:{123.f,125.f}){
  parking_start_footprint::robotQuads(0,0,0,0,0,q,width);
  float low=1000,high=-1000;
  for(auto &body:q)for(auto &p:body.p){low=fminf(low,p.y);high=fmaxf(high,p.y);}
  if(fabsf(high-low-width)>.001f)return 1;++checks;
 }
 for(int sign:{-1,0,1}){
  parking_start_footprint::robotQuads(0,0,0,sign,0,q,123,OBSTACLE_FINAL_PARKING_REAR_OVERHANG_MM);
  float rear=1000,front=-1000;
  for(auto &body:q)for(auto &p:body.p){rear=fminf(rear,p.x);front=fmaxf(front,p.x);}
  if(fabsf(rear+30)>.001f)return 4;
  parking_start_footprint::Quad original[6];
  parking_start_footprint::robotQuads(0,0,0,sign,0,original,123);
  float oldRear=1000,oldFront=-1000;
  for(auto &body:original)for(auto &p:body.p){oldRear=fminf(oldRear,p.x);oldFront=fmaxf(oldFront,p.x);}
  if(fabsf(oldRear+40)>.001f || fabsf(front-oldFront)>.001f)return 5;
  ++checks;
 }
 for(int dir:{-1,1}){
  ::routeTurnSign=dir;resetState();baseline();for(auto&s:seats)s.confirmed=false;
  final_parking_reset();parking_module::routeTurnSign=dir;scanPhase=5;measuredGapMm=247.5f;
 entrySafetyMargin=3;
  localToField(77.3f,120,0,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
  if(!entryEndReserveReady() || OBSTACLE_FINAL_PARKING_TARGET_LOCAL_X_MM!=60)return 2;
  ++checks;
  bool found=false;
  for(int yaw:{10,20,30,40,50,60})for(int y=115;y<=260&&!found;y+=5)
  for(int x=50;x<=220&&!found;x+=5){
   PositionEstimate p;localToField(x,y,yaw,p.x_mm,p.y_mm,p.heading_deg);
   int steering=50*dir;
   if(!noFrontReserveSafe(p,steering)||parkingEntryPoseSafe(p,steering))continue;
   for(int mm:{5,10,15,20}){
    PositionEstimate back=p;const float h=p.heading_deg*PI/180.f;
    back.x_mm-=mm*cosf(h);back.y_mm-=mm*sinf(h);
    if(parkingEntryPoseSafe(back,steering)){
     std::cout<<"NOSE RESERVE dir/x/y/yaw/retreat="<<dir<<"/"<<x<<"/"<<y<<"/"<<yaw<<"/"<<mm<<"\n";
     found=true;break;
    }
   }
  }
  if(!found)return 3;++checks;
 }
 std::cout<<"123MM GEOMETRY "<<checks<<" PASS; rearward manoeuvre gains nose room, parked X unchanged\n";
}
''')


if __name__=='__main__':
    main()
