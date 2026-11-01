"""631 measured first-arc pose: fast remaining recipe and physical completion."""
from check_final_parking_sequence import fixture_source
from check_returned_603_and_parking_practice import run


def main():
    run('631-tail',fixture_source()+r'''
using namespace parking_module;
int main(){int accepted=0;
 for(int dir:{-1,1})for(float actualScale:{1.159f,1.20f,1.235f}){
  ::routeTurnSign=dir;resetState();baseline();for(auto&s:seats)s.confirmed=s.injected=false;
  final_parking_reset();parking_module::routeTurnSign=dir;scanPhase=5;measuredGapMm=251.7f;
  localToField(269.2f,295.9f,-.5f,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
  if(!planParkingEntry(hostPose))return 1;
  for(int i=0;i<adaptiveEntryCount;++i)if(adaptiveEntry[i].steering){segmentIndex=i;break;}
  localToField(136.2f,237.4f,60.2f,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
  entryRadiusScale=1.159f;entryRadiusMeasured=true;
  const bool ok=adaptRemainingEntryRecipe(hostPose);
  if(!ok)return 2;
  std::cout<<"631 TAIL BUDGET remaining="<<entryPlanningBudget<<"\n";
  truePose=hostPose;hostTime=1000;encoder=0;hostGyroHealthy=true;
  stop(false);robot_logger.writes=0;state=FP_SEGMENT_SETTLE;stateStartMs=0;
  bool collision=false;
  for(int tick=0;tick<6000&&!final_parking_complete()&&!final_parking_aborted();++tick){
   final_parking_update(dir);
   if(dc_state==DC_ENABLED){
    float left=fabsf(command*.02f),sign=command<0?-1.f:1.f;
    while(left>.0001f){float ds=sign*fminf(1,left);left-=fabsf(ds);encoder+=ds;
     advanceEntryModel(hostPose,steeringCommand,ds,actualScale);truePose=hostPose;
     if(!parkingEntryPoseSafe(truePose,steeringCommand,0))collision=true;
    }
   }hostTime+=20;
  }
  const bool complete=final_parking_complete()&&finalFootprintContained()&&!collision
      &&entryReplans<=2;
  accepted+=complete;
  std::cout<<"631 COMPLETE "<<dir<<" "<<complete<<" replans="<<int(entryReplans)<<" collision="<<collision<<"\n";
  if(!complete)std::cout<<robot_logger.lines.str();

 }
 return accepted==6?0:2;
}
''')


if __name__=='__main__':
    main()
