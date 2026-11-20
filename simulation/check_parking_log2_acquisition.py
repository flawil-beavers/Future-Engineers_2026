"""Replay log2's post-lap pose through production parking acquisition.
CAD plant and ideal multi-object ToF; no physical slip/reflection acceptance.
"""
from check_final_parking_sequence import fixture_source
from check_returned_603_and_parking_practice import run
source=fixture_source()+r'''
using namespace parking_module;
int main(){int bad=0;
 for(float lateralBias:{-15.f,0.f,15.f}){
  ::routeTurnSign=1;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  for(int i:{1,5,6,11,14,21}){seats[i].confirmed=seats[i].injected=true;seats[i].red=i==11||i==14||i==21;}
  final_parking_reset();hostTime=1000;hostGyroHealthy=true;encoder=0;
  hostPose={-156.7f,-714.6f,357.5f};truePose=hostPose;truePose.y_mm+=lateralBias;
  stop(false);bool collision=false,scan=false;float minTip=1e9;
  for(int tick=0;tick<1800;++tick){
   final_parking_update(1);
   if(state==FP_SCAN_DRIVE){scan=true;minTip=truePose.y_mm-61.5f-(-1300.f);break;}
   if(final_parking_aborted())break;
   if(dc_state==DC_ENABLED){
    if(!parkingEntryPoseSafe(truePose,steeringCommand,0))collision=true;
    const float ds=command*.02f;encoder+=ds;
    advanceParkingPose(hostPose,steeringCommand,ds);advanceParkingPose(truePose,steeringCommand,ds);
   }hostTime+=20;
  }
  bool ok=scan&&!collision&&minTip>=5.f;
  std::cout<<"LOG2 ACQUISITION bias/scan/collision/tip_clearance/ms="<<lateralBias<<"/"<<scan<<"/"<<collision<<"/"<<minTip<<"/"<<hostTime-1000<<"\n";
  if(!ok){++bad;std::cout<<robot_logger.lines.str();}
 }
 std::cout<<"LOG2 ACQUISITION FAILED "<<bad<<"\n";return bad?1:0;
}
'''
run('parking-log2-acquisition',source,0)
