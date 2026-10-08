"""Logged C first-lap pose, B connector replans, and passed-endpoint handover.
Run check_later_laps.py first to regenerate the shared production fixture.
Uses production C++ functions; ideal bicycle for the first-lap controller.
"""
import shutil,subprocess
from check_cw_start_planner import ROOT
from check_connector_servo_resume import block

def main():
    source=(ROOT/'src/obstacle_path.cpp').read_text()
    fixture=(ROOT/'local_workspace/later-laps/check.cpp').read_text().split('int main()')[0]
    fixture=fixture.replace('struct Discovery {bool observedClear=false;};',
                            'struct Discovery {bool observedClear=false;bool seatObservedClear[2]={};};')
    fixture+='PositionEstimate cornerViewOrigin;\n'
    fixture+='enum class ColorType {NONE, RED, GREEN};struct Blob {bool found=true;ColorType color=ColorType::GREEN;int minX=0,maxX=100,minY=80,maxY=110;int width()const{return maxX-minX+1;}int height()const{return maxY-minY+1;}};\n'
    for signature in ('void seatCameraGeometry(', 'bool seatComfortablyVisible(\n',
                      'bool rejectedGreenBehindSeat(', 'bool cornerViewSeatMayBeOccupied(',
                      'bool cornerViewSweepSafe(', 'float cornerViewReverseDistance(', 'float cornerViewMinimumRange(',
                      'int cornerViewStraightSteering(', 'bool cornerViewSteeredCommandSafe(',
                      'bool distantSeatSideAmbiguous('):
        # seatComfortablyVisible also has a forward declaration.
        at=source.index(signature)
        if signature.startswith('bool seatComfortablyVisible'):
            at=source.index(signature,at+len(signature))
        fixture+=block(source,at)+'\n'
    fixture+=r'''
int main(){int bad=0,total=0;
 // Rejected broad GREEN behind the near target cannot block that target.
 Blob background;bool far=rejectedGreenBehindSeat(&background,18.4f,289.4f);
 background.maxY=165;bool local=rejectedGreenBehindSeat(&background,18.4f,289.4f);
 background.maxY=110;background.maxX=20;bool narrow=rejectedGreenBehindSeat(&background,18.4f,289.4f);
 background.maxX=100;background.color=ColorType::RED;bool red=rejectedGreenBehindSeat(&background,18.4f,289.4f);
 if(!far||local||narrow||red)++bad;total+=4;
 for(int mask=0;mask<64;++mask){
  // Enumerate Figure8c occupancy patterns, disregarding sign colours.
  // Inference is the intersection of empty stations across compatible cards.
  int expected=7,compatible=0;
  for(int card:{1,2,4,8,16,32,17,18,33,34})if((card&mask)==mask){
   int empty=0;for(int station=0;station<3;++station)if(!(card&(3<<(2*station))))empty|=1<<station;
   expected&=empty;++compatible;
  }
  if(!compatible)expected=0;
  if(obstacle_section_inferred_empty(mask)!=expected||obstacle_section_empty_for_mode(mask,true))++bad;
  ++total;
 }
 // B502/503: keep strict acquisition, choose a deeper checked observation.
 routeTurnSign=1;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
 for(auto &d:discoveryStations)d.seatObservedClear[0]=d.seatObservedClear[1]=false;
 Blob near;near.color=ColorType::RED;near.minY=80;near.maxY=238;near.maxX=70;
 if(cornerViewMinimumRange(&near)!=300)++bad;++total;
 near.maxY=108;if(cornerViewMinimumRange(&near)!=OBSTACLE_DISCOVERY_VIEW_MIN_MM)++bad;++total;
 for(int st:{0,9}){
  PositionEstimate p;p.x_mm=st==0?-761.6f:-829.3f;p.y_mm=st==0?-847.4f:768.7f;p.heading_deg=st==0?331.07f:243.73f;
  discoveryStations[st].seatObservedClear[0]=true;
  float d=cornerViewReverseDistance(p,st,300.0f);
  std::cout<<"B502/503 deeper view "<<st<<" "<<d<<"mm\n";
  if(d<=0 || d<=100 || !cornerViewSweepSafe(p,-d-20))++bad;++total;
  for(int dx:{-5,0,5})for(int dy:{-5,0,5})for(int dh:{-2,0,2}){
   PositionEstimate q=p;q.x_mm+=dx;q.y_mm+=dy;q.heading_deg+=dh;
   float r=cornerViewReverseDistance(q,st,300.0f);
   if(r<=0 || !cornerViewSweepSafe(q,-r-20))++bad;++total;
  }
  cornerViewOrigin=p;
  for(int dir:{-1,1})for(float drift:{-2.8f,2.8f}){
   PositionEstimate q=p;q.heading_deg+=drift;
   int steer=cornerViewStraightSteering(q,dir);
   float dh=-dir/Ackermann::getTurnRadius(static_cast<float>(steer));
   if(drift*dh>=0 || abs(steer)>8 || !cornerViewSteeredCommandSafe(q,steer,dir))++bad;++total;
  }
 }
 // B495: one side already clear; do not demand that it re-enter the image.
 routeTurnSign=-1;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
 for(auto &d:discoveryStations){d.seatObservedClear[0]=d.seatObservedClear[1]=false;}
 discoveryStations[6].seatObservedClear[0]=true;
 PositionEstimate recovery;recovery.x_mm=-722.7f;recovery.y_mm=861.1f;recovery.heading_deg=58.19f;
 float reverse=cornerViewReverseDistance(recovery,6);
 ++total;if(reverse<=0){++bad;std::cout<<"B495 RECOVERY FAIL\n";}else std::cout<<"B495 reverse "<<reverse<<"mm\n";
 seats[12].confirmed=true;if(!cornerViewSeatMayBeOccupied(12))++bad;
 ++total;
 routeTurnSign=1;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
 if(!distantSeatSideAmbiguous(1016,419,1139,10)||
    !distantSeatSideAmbiguous(944,431,1134,11)||
    !distantSeatSideAmbiguous(970,404,1119,11))++bad;
 total+=3;
 if(distantSeatSideAmbiguous(1100,500,1134,10)||
    distantSeatSideAmbiguous(944,431,600,11))++bad;
 total+=2;
 for(int dx:{-5,0,5})for(int dy:{-5,0,5})for(int dh:{-2,0,2}){
  routeTurnSign=-1;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  for(int i:{4,7,11,22}){seats[i].confirmed=seats[i].injected=true;seats[i].red=i==4;}
  learned();joinSameColourStraightEnds(livePath,true);roundKnownCornerPairs(livePath,true);recomputeSpeedProfile(livePath);
  float x=-1325+dx,y=-320.6f+dy,h=(64.32f+dh)*PI/180;
  progressIndex=nearestPathIndex(baselinePath,x,y,0,pathLength);
  bool ok=false;
  for(int step=0;step<750;++step){
   ObstacleClearanceSample c{};calculateClearanceAtPose(seats[11],x,y,h*180/PI,c);
   if(c.pillarMm<20||c.wallMm<20)break;
   if(y>650){ok=x<seats[11].x-125;break;}
   progressIndex=nearestPathIndex(livePath,x,y,progressIndex,OBSTACLE_PATH_PROGRESS_WINDOW);
   PositionEstimate p;p.x_mm=x;p.y_mm=y;p.heading_deg=h*180/PI;
   auto t=findLookahead(livePath,p,150);float ax=t.x-x,ay=t.y-y;
   float k=2*(-ax*sinf(h)+ay*cosf(h))/fmaxf(1,ax*ax+ay*ay);
   int servo=static_cast<int>(clampFloat(-atanf(100*k)*180/PI,-42,42));k=-tanf(servo*PI/180)/100;
   float n=h+k*2;if(fabsf(k)>1e-6){x+=(sinf(n)-sinf(h))/k;y+=(cosf(h)-cosf(n))/k;}else{x+=2*cosf(h);y+=2*sinf(h);}h=n;
  }
  ++total;if(!ok){++bad;std::cout<<"C FAIL "<<dx<<" "<<dy<<" "<<dh<<" "<<x<<" "<<y<<"\n";}
 }
 // Exact B497: GREEN start seat5, GREEN next section7; RED10 confirmed
 // late during the retained start connector, not a new start-seat obstacle.
 for(int changed:{10,11}){
  routeTurnSign=1;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  parkingCcwShortStart=true;parkingCwShortStart=false;parkingEntryConnectorActive=false;
  seats[5].confirmed=seats[5].injected=true;seats[5].red=false;
  seats[7].confirmed=seats[7].injected=true;seats[7].red=false;
  learned();joinSameColourStraightEnds(livePath,true);roundKnownCornerPairs(livePath,true);
  PositionEstimate p;p.x_mm=270.9f;p.y_mm=-1231.8f;p.heading_deg=44.74f;
  parkingEntryTargetStation=2;
  bool armed=buildParkingEntryConnector(p,livePath,parkingEntryConnectorMergeIndex);
  parkingEntryConnectorActive=armed;
  seats[changed].confirmed=seats[changed].injected=true;seats[changed].red=true;
  learned();joinSameColourStraightEnds(livePath,true);roundKnownCornerPairs(livePath,true);
  parkingEntryConnectorChangedSeat=changed;
  p.x_mm=392.5f;p.y_mm=-674.9f;p.heading_deg=45.31f;
  bool retained=armed&&retainParkingConnectorForFarGreen(p,livePath);
  ++total;if(!retained){++bad;std::cout<<"B RETAIN FAIL "<<changed<<" armed "<<armed<<"\n";}
 }
 // C489: endpoint behind the robot; tangent-aligned outgoing path is valid.
 routeTurnSign=1;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
 parkingCcwShortStart=true;parkingCwShortStart=false;
 PositionEstimate p;p.x_mm=270.3f;p.y_mm=-1244.6f;p.heading_deg=43.0f;
 learned();parkingEntryTargetStation=2;
 if(!buildParkingEntryConnector(p,livePath,parkingEntryConnectorMergeIndex))++bad;
 ++total;
 if(!connectorJoinReached(635,-959,6.42f,livePath,parkingEntryConnectorMergeIndex,true))++bad;
 ++total;
 if(connectorJoinReached(635,-959,80,livePath,parkingEntryConnectorMergeIndex,true)||
    connectorJoinReached(635,-800,6.42f,livePath,parkingEntryConnectorMergeIndex,true))++bad;
 ++total;
 std::cout<<"C / JOIN "<<total<<" FAILED "<<bad<<"\n";return bad?1:0;
}
'''
    destination=ROOT/'local_workspace/layout-c-join';destination.mkdir(exist_ok=True)
    cpp=destination/'check.cpp';exe=destination/'check.exe';cpp.write_text(fixture)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),'-I',str(ROOT/'local_workspace/later-laps'),str(cpp),'-o',str(exe)],check=True)
    r=subprocess.run([str(exe)],capture_output=True,text=True);(destination/'output.txt').write_text(r.stdout);print(r.stdout);raise SystemExit(r.returncode)

if __name__=='__main__':main()
