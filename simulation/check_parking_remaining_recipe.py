"""616: adapt the accepted recipe after measured first-arc radius change."""
from check_final_parking_sequence import fixture_source
from check_returned_603_and_parking_practice import run

if __name__ == '__main__':
    run('616-tail-adapt', fixture_source() + r'''
using namespace parking_module;
int main(){int accepted=0,total=0;
for(int dir:{-1,1})for(float scale:{1.05f,1.10f,1.136f}){
 ::routeTurnSign=dir;resetState();baseline();for(auto&s:seats)s.confirmed=false;
 final_parking_reset();parking_module::routeTurnSign=dir;scanPhase=5;measuredGapMm=245.2f;
 localToField(270,293.4f,0.8f,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
 if(!planParkingEntry(hostPose))return 1;
 int arc=0;for(int i=0;i<adaptiveEntryCount;++i){
 auto seg=adaptiveEntry[i];advanceEntryModel(hostPose,seg.steering*dir,
 seg.direction*seg.distanceMm*(seg.steering?scale:1),scale);
 if(seg.steering){arc=i;break;}}
 segmentIndex=arc;entryRadiusScale=scale;entryRadiusMeasured=true;
 bool ok=adaptRemainingEntryRecipe(hostPose);++total;accepted+=ok;
 if(!ok)std::cout<<"TAIL DIAGNOSTIC "<<robot_logger.lines.str()<<" pose="<<hostPose.x_mm<<","<<hostPose.y_mm<<","<<hostPose.heading_deg<<"\n";
 std::cout<<"TAIL "<<dir<<"/"<<scale<<"/"<<ok<<"\n";
}
std::cout<<"616 REMAINING RECIPE "<<accepted<<"/"<<total<<" accepted\n";
return accepted==total?0:2;
}

''')
