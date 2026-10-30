"""CCW recorded pre-scan holds; production sweep and subsequent view/joins.

Known-colour joins and ideal CAD motion do not replay unavailable camera images.
"""
import shutil
import subprocess
from check_cw_start_planner import ROOT, fixture_source
from check_connector_servo_resume import block


def main():
    path=(ROOT/'src/obstacle_path.cpp').read_text()
    code=fixture_source().split('int main()')[0].replace('anchor.heading_deg=180;', 'anchor.heading_deg=0;')
    for sig in ('void seatCameraGeometry(', 'bool seatComfortablyVisible(',
                'bool preflightCcwFrontView(', 'float ccwFrontViewExtensionMm('):
        code+=block(path,path.rindex(sig))+'\n'
    code+=r'''
PositionEstimate reverseView(PositionEstimate pose,float travel){
 float k=-1.f/OBSTACLE_PARKING_ENTRY_SCAN_RADIUS_MM,h=pose.heading_deg*PI/180,n=h-k*travel;
 pose.x_mm+=(sinf(n)-sinf(h))/k;pose.y_mm+=(cosf(h)-cosf(n))/k;pose.heading_deg=n*180/PI;return pose;
}
int main(){
 routeTurnSign=1;parkingEntryTargetStation=2;parkingCwShortStart=false;parkingCcwShortStart=true;parkingCcwFirstEdgeStart=true;
 baseline();int count=0,joins=0;float minArc=100,maxRefine=0;
 const PositionEstimate anchors[4]={{267.3f,-1174.9f,358.8f},{318.4f,-1198.1f,359.9f},
                                  {267.6f,-1206.9f,1.5f},{266.8f,-1204.9f,.8f}};
 for(int index=0;index<4;++index){
  float safe=0;for(float arc=84;arc>=40;arc-=2)if(preflightCwStartArc(anchors[index],arc)){safe=arc;break;}
  buildParkingEntryPath(anchors[index]);
  const int number[4]={577,579,580,581};
  std::cout<<"ANCHOR "<<number[index]<<" modeled safe "<<safe<<" selected "<<parkingEntryScanArcMm()<<"\n";
  if(fabsf(safe-parkingEntryScanArcMm())>.01f)return 4;
 }
 for(int index=2;index<4;++index)
 for(float dx:{-5.f,0.f,5.f})for(float dy:{-5.f,0.f,5.f})for(float dh:{-2.f,0.f,2.f}){
  auto p=anchors[index];p.x_mm+=dx;p.y_mm+=dy;p.heading_deg+=dh;
  float arc=0;for(float candidate=84;candidate>=40;candidate-=2)
   if(preflightCwStartArc(p,candidate)){arc=candidate;break;}
  if(arc==0){std::cout<<"NO SAFE ARC "<<index<<" "<<dx<<" "<<dy<<" "<<dh<<"\n";return 1;}
  buildParkingEntryPath(p);
  if(fabsf(parkingEntryScanArcMm()-arc)>.01f)return 4;
  minArc=fminf(minArc,arc);
  auto view=reverseView(p,arc);
  float bearing,range;seatCameraGeometry(5,view,bearing,range);
  float extra=ccwFrontViewExtensionMm(view);maxRefine=fmaxf(maxRefine,extra);
  if(extra>0){
   float k=pursuitCurvatureForServo(-50),h=view.heading_deg*PI/180,n=h+k*extra;
   view.x_mm+=(sinf(n)-sinf(h))/k;view.y_mm+=(cosf(h)-cosf(n))/k;view.heading_deg=n*180/PI;
  }
  seatCameraGeometry(5,view,bearing,range);
  if(!seatComfortablyVisible(5,view)||fabsf(bearing)>15){
   std::cout<<"VIEW FAIL "<<index<<" "<<arc<<" extra "<<extra<<" bearing "<<bearing<<"\n";return 2;}
  for(bool red:{false,true}){
   memcpy(livePath,baselinePath,sizeof(PathPoint)*pathLength);
   seats[5].confirmed=seats[5].injected=true;seats[5].red=red;
   displaceForSeat(livePath,5,OBSTACLE_LAP1_CLEARANCE_MM,!red);
   uint16_t merge=0;if(!buildParkingEntryConnector(view,livePath,merge)){
    std::cout<<"JOIN FAIL "<<index<<" "<<dx<<" "<<dy<<" "<<dh<<" arc "<<arc<<" red "<<red<<" view "<<view.x_mm<<","<<view.y_mm<<","<<view.heading_deg<<" extra "<<extra<<"\n";
    auto origin=reverseView(p,arc);
    for(float e:{5.f,10.f,15.f,20.f,25.f,30.f,35.f}){
     float k=pursuitCurvatureForServo(-50),h=origin.heading_deg*PI/180,n=h+k*e;auto v=origin;
     v.x_mm+=(sinf(n)-sinf(h))/k;v.y_mm+=(cosf(h)-cosf(n))/k;v.heading_deg=n*180/PI;
     uint16_t m=0;std::cout<<"trial "<<e<<" safe "<<preflightCcwFrontView(origin,e)<<" join "<<buildParkingEntryConnector(v,livePath,m)<<"\n";
    }return 3;}
   ++joins;
  }
  seats[5].confirmed=seats[5].injected=false;++count;
 }
 std::cout<<"CCW "<<count<<" perturbed sweeps/views "<<joins<<" GREEN/RED joins PASS min arc "<<minArc<<" max refinement "<<maxRefine<<"\n";
}
'''
    dest=ROOT/'local_workspace/ccw-577-581';dest.mkdir(parents=True,exist_ok=True)
    for name,source,expected in [('check',code,0),('old_scan_floor',code.replace('arc>=40.0f','arc>=68.0f'),4)]:
        cpp,exe=dest/(name+'.cpp'),dest/(name+'.exe');cpp.write_text(source)
        subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),
                        '-I',str(ROOT/'local_workspace/later-laps'),str(cpp),'-o',str(exe)],check=True)
        r=subprocess.run([str(exe)],capture_output=True,text=True,timeout=180)
        (dest/(name+'.txt')).write_text(r.stdout+r.stderr);print(r.stdout,end='')
        assert r.returncode==expected,r.returncode
    print('580 old68mm search floor negative control reproduces refusal; good577/579 retain84mm')


if __name__=='__main__':main()
