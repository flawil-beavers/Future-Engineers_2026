"""Production incremental park preparation, end-lap speed and rolling gate.
Known learned maps/CAD; no real servo, camera or tyre acceptance.
"""
from check_final_parking_sequence import fixture_source
from check_connector_servo_resume import block
from check_returned_603_and_parking_practice import run
from pathlib import Path
path=Path('src/obstacle_path.cpp').read_text()
s=fixture_source()+'\nbool rollingFinishAllowed=false;\nbool obstacle_path_parking_handoff_pose(PositionEstimate &,float *distanceMm=nullptr);\n'
for fn in ('bool obstacle_path_parking_handoff_pose(', 'void obstacle_path_allow_rolling_finish(', 'float parkingFinishSpeed('):
    s+=block(path,path.index(fn))+'\n'
oc=Path('src/obstacle.cpp').read_text()
finish=block(path,path.index('    if (finished)',path.index('void obstacle_path_update(')))
start=oc.index('    const int8_t parkingTurnSign = OBSTACLE_PARKING_EXIT_ENABLED')
end=oc.index('    if (OBSTACLE_FIRST_LAP_TEST_ENABLED)',start)
s+='''
bool oc_complete=false,oc_lap_finish_braking=false;unsigned long oc_lap_finish_brake_ms=0;int oc_parking_exit_steering=-50;bool newCameraFrame=false;
bool obstacle_path_complete(){return finished;}
uint8_t obstacle_path_lap(){return completedLaps;}
void obstacle_path_update(bool){finished=true;
'''+finish+'}\n'
s+='void dispatchHandoff(){using namespace parking_module;\n'+oc[start:end]+'}\n'
s+=r'''
using namespace parking_module;
int main(){int total=0,prepared=0,rolling=0,bad=0;
 for(int dir:{-1,1})for(int layout:{0,1,2})for(int incoming=0;incoming<4;++incoming){
  ::routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  if(layout==1)for(int i:{int(dir>0),4+int(dir>0)}){seats[i].confirmed=seats[i].injected=true;seats[i].red=false;}
  if(layout==2){int i=2+int(dir>0);seats[i].confirmed=seats[i].injected=true;seats[i].red=true;}
  int last=22+incoming/2;seats[last].confirmed=seats[last].injected=true;seats[last].red=incoming%2;
  learned();joinSameColourStraightEnds(livePath,true);roundKnownCornerPairs(livePath,true);
  if(!buildOptimizedPath()){++bad;continue;}
  PositionEstimate arrival;float distance=0;if(!obstacle_path_parking_handoff_pose(arrival,&distance)){++bad;continue;}
  final_parking_reset();hostPose=truePose=arrival;hostTime=1000;encoder=0;hostGyroHealthy=true;
  stop(false);servo_disabled=false;set_speed(150);set_steering(0);
  for(int tick=0;tick<10000&&!approachPreparation.ready&&!approachPreparation.failed;++tick)
   final_parking_prepare_approach(dir,arrival);
  ++total;
  if(dc_state!=DC_ENABLED||command!=150||state!=FP_IDLE||steeringCommand!=0){++bad;continue;}
  bool ready=approachPreparation.ready;prepared+=ready;
  ::progressIndex=nearestPathIndex(baselinePath,arrival.x_mm,arrival.y_mm,0,pathLength);
  for(int lap:{0,1}){::completedLaps=lap;if(parkingFinishSpeed(520,arrival)!=520)++bad;}
  ::completedLaps=2;if(parkingFinishSpeed(520,arrival)>150.01f){++bad;continue;}
  if(ready){
   PositionEstimate outside=arrival;outside.x_mm+=41;
   if(final_parking_rolling_ready(outside)){++bad;continue;}
   current_speed=measured_speed=200;
   if(final_parking_rolling_ready(arrival)){++bad;continue;}
   current_speed=measured_speed=150;
   hostGyroHealthy=false;if(final_parking_rolling_ready(arrival)){++bad;continue;}hostGyroHealthy=true;
   dc_state=DC_HOLDING;if(final_parking_rolling_ready(arrival)){++bad;continue;}dc_state=DC_ENABLED;
   measured_speed=-1;if(final_parking_rolling_ready(arrival)){++bad;continue;}measured_speed=150;
   bool started=final_parking_start_rolling(dir);
   if(started!=(dir>0)|| (started&&(dc_state!=DC_ENABLED||state!=FP_APPROACH_LANE))){++bad;continue;}
   rolling+=started;
   oc_complete=oc_lap_finish_braking=false;oc_parking_exit_steering=dir>0?-50:50;
   ::finished=false;
   if(started){
    // Exercise the real outer dispatcher again: an active rolling parking
    // controller must not receive the ordinary finish brake on tick2.
    dispatchHandoff();hostTime+=10;dispatchHandoff();
    if(oc_lap_finish_braking||dc_state!=DC_ENABLED||command<=0){++bad;continue;}
    bool collision=false,scan=false;
    for(int tick=0;tick<2500&&!final_parking_aborted();++tick){
     dispatchHandoff();
     if(state==FP_SCAN_DRIVE){scan=true;break;}
     if(dc_state==DC_ENABLED){float remaining=fabsf(command*.02f);while(remaining>0.001f){
      const float ds=(command<0?-1.f:1.f)*fminf(1.f,remaining);remaining-=fabsf(ds);encoder+=ds;
      advanceParkingPose(hostPose,steeringCommand,ds);truePose=hostPose;
      if(!parkingEntryPoseSafe(truePose,steeringCommand,0))collision=true;
     }}hostTime+=20;
    }
    if(!scan||collision){++bad;continue;}
   }else{
    dispatchHandoff();
    if(!oc_lap_finish_braking||dc_state!=DC_HOLDING){++bad;continue;}
   }
  }
  std::cout<<"DIRECT HANDOFF dir/layout/incoming/prepared="<<dir<<"/"<<layout<<"/"<<incoming<<"/"<<ready<<"\n";
 }
 // CW with an isolated GREEN before the last corner and RED on the start
 // straight. Its new forward connector must not replace the parking guide
 // that the robot tracks in reverse from the handoff.
 ::routeTurnSign=-1;resetState();baseline();
 for(auto &s:seats)s.confirmed=s.injected=false;
 for(int i:{0,4,7,10,15,20}){
  seats[i].confirmed=seats[i].injected=true;
  seats[i].red=i!=4 && i!=20;
 }
 learned();joinSameColourStraightEnds(livePath,true);
 PositionEstimate cwArrival;
 if(!buildOptimizedPath() || !obstacle_path_parking_handoff_pose(cwArrival,nullptr))return 2;
 final_parking_reset();hostPose=truePose=cwArrival;hostTime=1000;
 encoder=0;hostGyroHealthy=true;
 stop(false);servo_disabled=false;set_speed(150);set_steering(0);
 for(int tick=0;tick<10000&&!approachPreparation.ready&&!approachPreparation.failed;++tick)
  final_parking_prepare_approach(-1,cwArrival);
 if(!approachPreparation.ready)return 3;
 std::cout<<"DIRECT HANDOFF isolated CW GREEN reverse guide PASS\n";
 std::cout<<"DIRECT HANDOFF total/prepared/rolling/failed="<<total<<"/"<<prepared<<"/"<<rolling<<"/"<<bad<<"\n";
 return bad || prepared<16 || rolling<8 ? 1:0;
}
'''
run('direct-parking-handoff',s)
