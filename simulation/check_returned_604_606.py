"""604 reserve escape and605/606 rear-space parking sensitivity, host only."""
from check_later_laps import later_lap_fixture
from check_final_parking_sequence import ROOT, fixture_source
from check_connector_servo_resume import block
from check_returned_603_and_parking_practice import run


def main():
    path=(ROOT/'src/obstacle_path.cpp').read_text()
    geometry=later_lap_fixture().split('int main()')[0]
    geometry=geometry.replace('struct Discovery {bool observedClear=false;};',
        'struct Discovery {bool observedClear=false;bool seatObservedClear[2]={};};')
    for sig in ('void seatCameraGeometry(', 'bool seatComfortablyVisible(',
                'bool cornerViewSeatMayBeOccupied(', 'bool cornerViewRetreatSafe(',
                'bool cornerViewSweepSafe(', 'PositionEstimate cornerViewArcPose(', 'bool cornerViewArcSafe('):
        geometry+=block(path,path.rindex(sig))+'\n'
    run('604-escape',geometry+r'''
int main(){int cases=0;
 routeTurnSign=1;resetState();baseline();
 for(auto &s:seats)s.confirmed=false;
 for(auto &d:discoveryStations)d.seatObservedClear[0]=d.seatObservedClear[1]=false;
 for(float dx:{-2.f,0.f,2.f})for(float dy:{-2.f,0.f,2.f})for(float dh:{-.5f,0.f,.5f}){
  PositionEstimate p{880.3f+dx,-668.2f+dy,46.12f+dh};
  // Old hard40mm initial gate refuses this positive-clearance pose.
  if(cornerViewSweepSafe(p,0))return 1;
  if(!cornerViewArcSafe(p,-20,-110))return 2;
  auto view=cornerViewArcPose(p,-20,-90);
  if(!seatComfortablyVisible(6,view))return 3;
  // Actual runtime15mm forecasts must also permit every reverse step.
  for(int travel=0;travel<90;travel+=3)
   if(!cornerViewArcSafe(cornerViewArcPose(p,-20,-float(travel)),-20,-15))return 4;
  ++cases;
 }
 PositionEstimate p{880.3f,-668.2f,46.12f};
 if(cornerViewRetreatSafe(p,-20,30)||cornerViewRetreatSafe(p,50,-90))return 5;
 // A true obstacle at the axle remains forbidden, with no margin exception.
 seats[7].x=p.x_mm;seats[7].y=p.y_mm;
 if(cornerViewRetreatSafe(p,-20,-110))return 6;
 std::cout<<"604 CHECKED RETREAT "<<cases<<" PLUS3 NEGATIVE PASS\n";
}
''')
    source=fixture_source()
    source=source.replace('PositionEstimate hostPose,truePose;',
        'bool rearEnabled=false;int rearFault=0;float rearRaw=0;PositionEstimate hostPose,truePose;')
    source=source.replace(' if(sensor==TOF_REAR)return false;',r'''
 if(sensor==TOF_REAR){
  if(!rearEnabled)return false;
  s.sequence=hostTime/100;s.sampled_ms=hostTime-(rearFault==1?300:0);
  s.selected_raw_distance_mm=rearRaw;s.filtered_distance_mm=rearRaw+(rearFault==2?20:0);
  s.selected_signal_mcps=rearFault==3?0:1;s.selected_sigma_mm=rearFault==4?20:1;
  return true;
 }''')
    run('parking-rear',source+r'''
using namespace parking_module;
int main(){int accepted=0,rejected=0;
 for(int dir:{-1,1})for(float yaw:{-10.f,0.f,10.f})for(int fault=0;fault<7;++fault){
  ::routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=false;
  final_parking_reset();parking_module::routeTurnSign=dir;scanPhase=5;measuredGapMm=247.5f;
  float y=fault==5?190:125;localToField(65,y,yaw,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
  rearEnabled=true;rearFault=fault;hostTime=1000;stateStartMs=900;measured_speed=0;
  rearRaw=(fault==6?120:80)/cosf(yaw*PI/180.f)-OBSTACLE_REAR_TOF_BEHIND_AXLE_MM;
  for(int i=0;i<3;++i){hostTime+=100;sampleRearEntryReference(hostPose);}
  bool ok=applyRearEntryReference();
  if(ok!=(fault==0)){std::cout<<"REAR FAIL "<<dir<<"/"<<yaw<<"/"<<fault<<"\n";return 7;}
  if(ok){if(fabsf(entryLocalPose(hostPose).x_mm-80)>.05f)return 8;++accepted;}
  else{if(fabsf(entryLocalPose(hostPose).x_mm-65)>.05f)return 9;++rejected;}
 }
 // A physically contained but front-crowded pose must seek rear space.
 for(int dir:{-1,1}){
  final_parking_reset();parking_module::routeTurnSign=dir;scanPhase=5;measuredGapMm=247.5f;
  localToField(112,120,0,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
  if(!finalFootprintContained()||entryEndReserveReady())return 10;
  if(!planParkingEntry(hostPose)||adaptiveEntry[0].direction!=-1)return 11;
 }
 std::cout<<"PARK REAR REFERENCE accepted/rejected="<<accepted<<"/"<<rejected<<"; FRONT RESERVE2 PASS\n";
}
''')


if __name__=='__main__':
    main()
