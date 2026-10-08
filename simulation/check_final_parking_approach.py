"""Production post-lap approach, CAD plant, legal start cards, both directions.
Run check_later_laps.py first to regenerate the shared production fixture.
No ToF marker classification, slip or servo latency is simulated here.
"""
import shutil, subprocess
from check_cw_start_planner import ROOT
from check_connector_servo_resume import block

def main():
    fixture=(ROOT/'local_workspace/later-laps/check.cpp').read_text().split('int main()')[0]
    path=(ROOT/'src/obstacle_path.cpp').read_text()
    final=(ROOT/'src/final_parking.cpp').read_text()
    fixture+=block(path,path.index('bool obstacle_path_parking_approach_target('))
    fixture+=final[final.index('struct ParkingSegment'):final.index('FinalParkingState state =')]
    fixture+='\nint scanPhase=5;float measuredGapMm=247.5f,approachOuterShiftX=OBSTACLE_FINAL_PARKING_OUTER_SHIFT_X_MM,approachLaneY=OBSTACLE_FINAL_PARKING_APPROACH_LANE_Y_MM;\n'
    fixture+=block(final,final.index('float movingInsideFaceX('))+'\n'
    fixture+=block(final,final.index('void localToField('))+'\n'
    fixture+='\nfloat baseHeadingDeg(){return routeTurnSign>0?0:180;}\nint motorDirectionForFieldMotion(int sign){return sign*routeTurnSign;}\n'
    fixture+='''
bool connectorClearanceSafe(const PositionEstimate &p){
 for(auto &s:seats)if(s.confirmed){ObstacleClearanceSample c{};
  if(!calculateClearanceAtPose(s,p.x_mm,p.y_mm,p.heading_deg,c)||!c.valid||c.pillarMm<10||c.wallMm<5)return false;}
 return true;
}
'''
    for signature in ('bool linePoseReady(', 'bool parkingApproachSteering(',
                      'void advanceParkingPose(', 'bool parkingApproachPoseSafe(',
                      'bool parkingApproachPreflight(', 'bool chooseParkingApproach(', 'bool parkingEntryPoseSafe(',
                      'bool parkingEntryPreflight('):
        body=block(final,final.index(signature))
        fixture+=body+'\n'
    fixture+=r'''
int main(){int total=0,bad=0;
 const int configs[11][3]={{0,0,0},{1,0,0},{2,0,0},{0,1,0},{0,2,0},{0,0,1},{0,0,2},{1,0,1},{1,0,2},{2,0,1},{2,0,2}};
 for(int dir:{-1,1})for(int layout=0;layout<11;++layout)for(int next=0;next<5;++next){
  routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  for(int st=0;st<3;++st){int i=2*st+(dir>0),c=configs[layout][st];seats[i].confirmed=seats[i].injected=c;seats[i].red=c==2;}
  if(next){int i=6+(next-1)/2;seats[i].confirmed=seats[i].injected=true;seats[i].red=(next-1)%2;}
  learned();joinSameColourStraightEnds(livePath,true);roundKnownCornerPairs(livePath,true);
  bool built=buildOptimizedPath();
  int index=3;
  for(int i=0;i<pathLength;++i)if(baselinePath[i].distanceMm>=corners[3].pathEndMm && finalCornerVehicleClear(PositionEstimate{optimizedPath[i].x,optimizedPath[i].y,connectorRouteHeading(optimizedPath,i)})){index=i;break;}
  for(int dx:{-5,0,5})for(int dy:{-5,0,5})for(int dh:{-2,0,2}){
   PositionEstimate p;p.x_mm=optimizedPath[index].x+dx;p.y_mm=optimizedPath[index].y+dy;
   p.heading_deg=connectorRouteHeading(optimizedPath,index)+dh;
   bool ok=built&&chooseParkingApproach(p);++total;
   if(!ok){++bad;std::cout<<"FAIL "<<dir<<" "<<layout<<" next "<<next<<" "<<p.x_mm<<" "<<p.y_mm<<" "<<p.heading_deg<<"\n";}
  }
 }
 int entries=0,entryBad=0;
 for(int dir:{-1,1})for(float gap:{242.5f,247.5f,252.5f}){
  routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  measuredGapMm=gap;
  for(int dx:{-5,0,5})for(int dy:{-5,0,5})for(int dh:{-1,0,1}){
   PositionEstimate p;localToField(OBSTACLE_FINAL_PARKING_CAPTURE_LOCAL_X_MM,OBSTACLE_FINAL_PARKING_CAPTURE_LOCAL_Y_MM,0.0f,p.x_mm,p.y_mm,p.heading_deg);
   p.x_mm+=dx;p.y_mm+=dy;p.heading_deg+=dh;
   bool ok=parkingEntryPreflight(p);++entries;
   if(!ok){++entryBad;std::cout<<"ENTRY REJECT "<<dir<<" "<<gap<<" "<<dx<<" "<<dy<<" "<<dh<<"\n";}
   if(dx==0&&dy==0&&dh==0&&!ok)++bad;
  }
 }
 std::cout<<"ENTRY "<<entries<<" accepted "<<entries-entryBad<<" rejected "<<entryBad<<" (nominal must pass)\n";
 std::cout<<"FINAL APPROACH "<<total<<" FAILED "<<bad<<"\n";return bad?1:0;
}
'''
    destination=ROOT/'local_workspace/final-parking-approach';destination.mkdir(exist_ok=True)
    cpp=destination/'check.cpp';exe=destination/'check.exe';cpp.write_text(fixture)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),'-I',str(ROOT/'local_workspace/later-laps'),str(cpp),'-o',str(exe)],check=True)
    r=subprocess.run([str(exe)],capture_output=True,text=True);(destination/'output.txt').write_text(r.stdout);print(r.stdout);raise SystemExit(r.returncode)

if __name__=='__main__':main()
