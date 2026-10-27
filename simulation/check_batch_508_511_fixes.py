"""Source-extracted regression for returned start, scan and exit failures.

Known layouts/CAD and bounded synthetic tolerances are not physical acceptance.
Run check_final_parking_sequence.py for the complete parking state machine.
"""
import json
import re
import shutil
import subprocess

from check_cw_start_planner import ROOT, fixture_source
from check_connector_servo_resume import block


def main():
    dest=ROOT/'local_workspace/fix_508_511'
    dest.mkdir(parents=True,exist_ok=True)
    fixture=fixture_source().split('int main()')[0].replace(
        'anchor.heading_deg=180;', 'anchor.heading_deg=routeTurnSign>0?0:180;')
    source=(ROOT/'src/obstacle.cpp').read_text()
    fixture+='\n'+block(source,source.index('struct ParkingExitSegment'))+';\n'
    at=source.index('static constexpr ParkingExitSegment PARKING_EXIT_SEGMENTS[')
    end=source.index('};',at)+2
    fixture+=source[at:end]+'\n'+block(source,source.index('static bool parkingExitMotionSafe('))+'\n'
    fixture+=r'''
bool exitRun(bool legacy,int turn,float factor,float dead,float gap,float dx,float dy,float dh){
 float x=(turn<0?390.f:570.f-gap)+dx,y=-1365+dy,h=(turn<0?180.f:0.f)+dh;
 for(int seg=0;seg<5;++seg){
  auto s=PARKING_EXIT_SEGMENTS[seg];if(legacy){const float old[]={20,25,20,90,155};s.distanceMm=old[seg];}
  int steer=s.steeringRelativeToAway*(turn<0?50:-50);
  bool aligned=false;
  for(float travel=0;travel<=220;travel+=2){
   parking_start_footprint::Quad bodies[6];
   parking_start_footprint::robotQuads(x,y,h,(steer>0)-(steer<0),0,bodies);
   auto fixed=parking_start_footprint::rectangle(490,-1400,15,100,0);
   auto moved=parking_start_footprint::rectangle(480-gap-10,-1400,15,100,0);
   for(auto &q:bodies){for(auto &p:q.p)if(p.y<=-1495)return false;
    if(parking_start_footprint::overlaps(q,fixed)||parking_start_footprint::overlaps(q,moved))return false;}
   if(seg<4&&travel>=s.distanceMm)break;
   if(!legacy&&turn<0){PositionEstimate p{x,y,h};
    if(!parkingExitMotionSafe(p,steer,s.direction,0))return false;}
   float error=wrap180(h-(turn<0?180.f:0.f));
   if(seg==4&&travel>=OBSTACLE_PARKING_EXIT_FINAL_ALIGN_MIN_MM&&fabsf(error)<=2){aligned=true;break;}
   if(travel>=220)return false;
   float k=travel<dead?0:-factor/Ackermann::getTurnRadius(steer),a=h*PI/180,ds=s.direction*2.f,n=a+k*ds;
   if(fabsf(k)>1e-6){x+=(sinf(n)-sinf(a))/k;y+=(cosf(a)-cosf(n))/k;}
   else{x+=ds*cosf(a);y+=ds*sinf(a);}h=n*180/PI;
  }
  if(seg==4&&!aligned)return false;
 }
 return true;
}
int main(){int n;std::cin>>n;routeTurnSign=-1;baseline();
 memcpy(livePath,baselinePath,sizeof(PathPoint)*pathLength);
 seats[0].confirmed=true;seats[0].red=false;
 seats[4].confirmed=seats[4].injected=true;seats[4].red=true;displaceForSeat(livePath,4,260,false);
 parkingEntryConnectorLength=n;
 for(int i=0;i<n;++i){auto &p=parkingEntryConnector[i];std::cin>>p.x>>p.y>>p.headingDeg;}
 int failures=0,trials=0;
 for(float dx:{-5.f,0.f,5.f})for(float dy:{-5.f,0.f,5.f})for(float dh:{-2.f,0.f,2.f}){
  PositionEstimate p{139.91f+dx,-1062.61f+dy,123.66f+dh};parkingEntryConnectorLookaheadMm=150;
  bool changed=widenParkingConnectorLookahead(p,livePath,10,0,true);
  auto target=connectorLookaheadFrom(10,parkingEntryConnectorLookaheadMm,livePath,0,true);
  float a=p.heading_deg*PI/180,x=target.x-p.x_mm,y=target.y-p.y_mm;
  float command=pursuitServoForCurvature(2*(-x*sinf(a)+y*cosf(a))/fmaxf(1,x*x+y*y));
  bool ok=fabsf(command)<=38 && connectorRolloutFeasible(p,parkingEntryConnectorLookaheadMm,2,-1,0,livePath,0,true);
  if(!ok){++failures;std::cout<<"CONNECTOR FAIL "<<dx<<" "<<dy<<" "<<dh<<" "<<command<<" "<<changed<<"\n";}++trials;
 }
 std::cout<<"CONNECTOR "<<trials<<" failures "<<failures<<"\n";
 routeTurnSign=1;parkingCwShortStart=false;parkingCcwShortStart=true;parkingCcwFirstEdgeStart=true;baseline();
 int scanFailures=0,scans=0;
 for(float dx:{-5.f,0.f,5.f})for(float dy:{-3.f,0.f,3.f})for(float dh:{-1.f,0.f,1.f}){
  const float referenceChange=OBSTACLE_PARKING_CCW_FIRST_EDGE_ENTRY_X_MM-360.0f;
  PositionEstimate p{356.2f+referenceChange+dx,-1232.6f+dy,358.4f+dh};buildParkingEntryPath(p);
  bool ok=preflightCwStartArc(p,parkingEntryScanArcMm());
  if(!ok){++scanFailures;std::cout<<"SCAN FAIL "<<dx<<" "<<dy<<" "<<dh<<"\n";}++scans;
 }
 std::cout<<"CCW SCAN "<<scans<<" failures "<<scanFailures<<"\n";failures+=scanFailures;
 int exits=0,exitFailures=0,legacyFailures=0;
 for(int turn:{-1,1})for(float factor:{.8f,.9f,1.f,1.05f})for(float dead:{0.f,4.f})
 for(float gap:{242.5f,252.5f})for(float dx:{-3.f,3.f})for(float dy:{-3.f,3.f})for(float dh:{-1.f,1.f}){
  bool ok=exitRun(false,turn,factor,dead,gap,dx,dy,dh);++exits;if(!ok)++exitFailures;
  if(!exitRun(true,turn,factor,dead,gap,dx,dy,dh))++legacyFailures;
 }
 std::cout<<"EXIT "<<exits<<" failures "<<exitFailures<<" legacy_failures "<<legacyFailures<<"\n";
 failures+=exitFailures;if(!legacyFailures)++failures;
 int measured=0,measuredFailures=0;
 for(float factor:{.8f,.9f,1.f,1.05f})for(float dead:{0.f,4.f})
 for(auto pose:{PositionEstimate{395,-1366,180},PositionEstimate{392.7f,-1364,180},PositionEstimate{390,-1365,180}}){
  ++measured;if(!exitRun(false,-1,factor,dead,247.5f,pose.x_mm-390,pose.y_mm+1365,0))++measuredFailures;
 }
 std::cout<<"MEASURED EXIT STARTS "<<measured<<" failures "<<measuredFailures<<"\n";
 failures+=measuredFailures;
 PositionEstimate contact{369.7f,-1345.2f,137.84f};
 if(parkingExitMotionSafe(contact,50,1,10))++failures;
 std::cout<<"TOTAL failures "<<failures<<"\n";return failures?1:0;
}
'''
    path=ROOT/'simulation/evidence/parking_exit_diagnostics/20261026_log_508_cw.txt'
    points=[]
    for line in path.read_text().splitlines():
        if line.startswith('[CONNECTOR_POINT] kind=connector'):
            record=dict(re.findall(r'(\w+)=([^ ]+)',line))
            points.append([float(record[k]) for k in ('x','y','h')])
    cpp,exe=dest/'regression.cpp',dest/'regression.exe';cpp.write_text(fixture)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),
        '-I',str(ROOT/'simulation/host'),str(cpp),'-o',str(exe)],check=True)
    inputs=str(len(points))+'\n'+'\n'.join(' '.join(map(str,p)) for p in points)+'\n'
    run=subprocess.run([str(exe)],input=inputs,text=True,capture_output=True)
    (dest/'regression_output.txt').write_text(run.stdout+run.stderr)
    print(run.stdout,end='');raise SystemExit(run.returncode)


if __name__=='__main__':main()
