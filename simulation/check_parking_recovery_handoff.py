"""Actual parking dispatch: recombined escape, late speed and real brake fault.

Uses an independent-radius host plant; no real traction/sensor acceptance.
--without-combined-envelope reproduces the pre-fix repeated stationary plan.
"""
import sys
from check_final_parking_sequence import fixture_source
from check_returned_603_and_parking_practice import run


def main():
    source=fixture_source()
    negative='--without-combined-envelope' in sys.argv
    if negative:
        gate='!entryStartRadiusEnvelopeSafe(combined,field) ||'
        assert gate in source
        source=source.replace(gate,'false ||',1)
    source+=r'''
using namespace parking_module;
void setup(int dir,float x,float y){
 ::routeTurnSign=dir;resetState();baseline();for(auto&s:seats)s.confirmed=s.injected=false;
 final_parking_reset();parking_module::routeTurnSign=dir;scanPhase=5;measuredGapMm=247.5;
 localToField(x,y,0,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);truePose=hostPose;
 hostTime=1000;hostGyroHealthy=true;encoder=0;stop(false);servo_disabled=false;steer(0);
 robot_logger.lines.str("");robot_logger.lines.clear();robot_logger.writes=0;
}
int main(){int total=0,bad=0;
 for(int dir:{-1,1}){
  // A long capture and10% radius error formerly selected the same cancelled
  // escape prefix repeatedly, without any drive progress, until abort.
  setup(dir,270.26,294.6);state=FP_REPLAN_BRAKE;stateStartMs=0;
  entryReplanReason="escape_handoff";bool collision=false;
  for(int tick=0;tick<6000&&!final_parking_complete()&&!final_parking_aborted();++tick){
   final_parking_update(dir);
   if(dc_state==DC_ENABLED){
    float remaining=fabsf(command*.02f),sign=command<0?-1.f:1.f;
    while(remaining>.0001f){const float ds=sign*fminf(1,remaining);remaining-=fabsf(ds);encoder+=ds;
     advanceEntryModel(hostPose,steeringCommand,ds,1.1f);truePose=hostPose;
     if(!parkingEntryPoseSafe(truePose,steeringCommand,0))collision=true;
    }
   }hostTime+=20;
  }
  ++total;if(!final_parking_complete()||collision||robot_logger.writes!=1){
   ++bad;std::cout<<"ESCAPE FAIL "<<dir<<" collision="<<collision
    <<" no_progress="<<(robot_logger.lines.str().find("entry_no_progress")!=std::string::npos)<<"\n";}
  // A single late speed report must brake and verify the existing good pose.
  setup(dir,85,128);hostTime+=500;state=FP_VERIFY;stateStartMs=hostTime-500;
  measured_speed=current_speed=SOFT_STOP_SPEED_THRESHOLD_MMS+1;final_parking_update(dir);
  ++total;if(final_parking_aborted()||state!=FP_REPLAN_BRAKE||dc_state==DC_ENABLED)++bad;
  measured_speed=current_speed=0;
  for(int i=0;i<30&&!final_parking_complete();++i){hostTime+=50;final_parking_update(dir);}
  if(!final_parking_complete()||final_parking_aborted())++bad;
  // A persistent inability to stop remains a hard brake fault, never a
  // permission to start the next manoeuvre or falsely finish parking.
  setup(dir,85,128);hostTime+=500;state=FP_VERIFY;stateStartMs=hostTime-500;
  measured_speed=current_speed=SOFT_STOP_SPEED_THRESHOLD_MMS+1;final_parking_update(dir);
  for(int i=0;i<100&&!final_parking_aborted();++i){
   hostTime+=50;measured_speed=current_speed=SOFT_STOP_SPEED_THRESHOLD_MMS+1;
   final_parking_update(dir);
  }
  ++total;if(!final_parking_aborted()||final_parking_complete()||dc_state==DC_ENABLED)++bad;
 }
 std::cout<<"PARKING HANDOFF "<<total<<" FAILED "<<bad<<"\n";
 return bad?1:0;
}
'''
    run('parking-handoff-negative' if negative else 'parking-handoff',source,1 if negative else 0)


if __name__=='__main__':main()
