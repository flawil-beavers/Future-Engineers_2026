"""Swept free travel, economical recipes and progress-based recovery limits."""
from check_final_parking_sequence import ROOT, fixture_source
from check_returned_603_and_parking_practice import run


if __name__ == '__main__':
    run('efficient-entry', fixture_source()+r'''
using namespace parking_module;
int main(){int checked=0;
 for(int dir:{-1,1}){
  ::routeTurnSign=dir;resetState();baseline();for(auto&s:seats)s.confirmed=s.injected=false;
  final_parking_reset();parking_module::routeTurnSign=dir;scanPhase=5;measuredGapMm=245.2f;
  localToField(269.2f,295.9f,-.5f,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
  if(!planParkingEntry(hostPose))return 5;
  int arcs=0,changes=0,previous=0;
  for(int i=0;i<adaptiveEntryCount;++i){const auto&s=adaptiveEntry[i];
   arcs+=s.steering!=0;changes+=previous&&previous!=s.direction;previous=s.direction;}
  // Prior recipe at this anchor: four arcs and three direction changes.
  if(arcs>2 || changes>1)return 6;
  ++checked;measuredGapMm=251.7f;
  // Moving40mm toward the front consumes front room and releases rear room.
  float rear[2],front[2];
  for(int i=0;i<2;++i){
   localToField(60+40*i,127,0,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
   entrySafetyMargin=3;entryBrakeTravelMm=0;
   rear[i]=entryFreeTravel(hostPose,-1,0,measuredGapMm);
   front[i]=entryFreeTravel(hostPose,1,0,measuredGapMm);
   for(int direction:{-1,1}){
    float length=direction<0?rear[i]:front[i];PositionEstimate p=hostPose;
    for(int mm=0;mm<=int(length);++mm){
     if(!parkingEntryPoseSafe(p,0))return 1;
     advanceEntryModel(p,0,direction,entryRadiusScale);
    }++checked;
   }
  }
  if(fabsf(rear[1]-rear[0]-40)>1.1f || fabsf(front[0]-front[1]-40)>1.1f)return 2;
  // A ninth correction with real net progress must remain eligible to plan.
  localToField(143.8f,246.4f,56.4f,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
  truePose=hostPose;hostTime=1000;hostGyroHealthy=true;stop(false);
  entryRadiusScale=1.169f;entryRadiusMeasured=true;
  state=FP_REPLAN_BRAKE;stateStartMs=0;entryReplans=8;
  entryStalledReplans=8;entryBestReplanError=10000;
  final_parking_update(dir);
  if(final_parking_aborted() || entryReplans!=9 || entryStalledReplans!=0)return 3;
  ++checked;
  // Repeating an unproductive pose is still a safe stop, never blind driving.
  state=FP_REPLAN_BRAKE;stateStartMs=0;entryStalledReplans=8;
  entryBestReplanError=0;hostTime+=1000;stop(false);
  final_parking_update(dir);
  if(!final_parking_aborted() || dc_state==DC_ENABLED)return 4;
  ++checked;
 }
 std::cout<<"FREE ROOM/PROGRESS "<<checked<<" PASS\n";
}
''')
    # Reuse the independent plant and brake simulation for the reduced-arc
    # capture, including the measured range of radius errors.
    script=(ROOT/'simulation/check_parking_motion_632_634.py').read_text()
    code=script.split("fixture_source()+r'''",1)[1].split("'''",1)[0]
    begin=code.index(' for(auto c:');end=code.index(' for(int dir:',begin)
    code=code[:begin]+''' for(auto c:{Case{245.2f,269.2f,295.9f,-.5f,1.f},
              Case{245.2f,269.2f,295.9f,-.5f,1.136f},
              Case{245.2f,269.2f,295.9f,-.5f,1.235f}})
'''+code[end:]
    code=code.replace('RETURNED632-634','EFFICIENT COMPLETION')
    code=code.replace('entryRadiusScale=c.radius;entryRadiusMeasured=c.y<250;',
                      'entryRadiusScale=1;entryRadiusMeasured=false;')
    run('efficient-entry-completion',fixture_source()+code)
