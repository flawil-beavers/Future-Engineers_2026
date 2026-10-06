"""Extract production map/route/lap functions; verify laps 2/3 without hardware.

Explicit layouts replace images. Ideal pursuit replaces motor/ToF dynamics.
Generated fixtures and results remain in local_workspace.
"""
import itertools
import json
import shutil
import subprocess

from check_cw_start_planner import ROOT, fixture_source
from check_connector_servo_resume import block


def main():
    source = (ROOT/'src/obstacle_path.cpp').read_text()
    fixture = fixture_source().split('int main()')[0]
    fixture = fixture.replace(
        'bool targetsOuterExtreme(const CandidateSeat &seat) {return seat.red ? seat.lateralMm<0 : seat.lateralMm>0;}',
        'bool targetsOuterExtreme(const CandidateSeat &seat);')
    fixture = fixture.replace('uint16_t pathLength=0;', 'uint16_t pathLength=0,progressIndex=0;')
    extra = r'''
#include "obstacle_section_layout.h"
constexpr int COURSE_SECTION_COUNT=4,COURSE_STATIONS_PER_SECTION=3;
constexpr int OBSTACLE_SECTION_LAYOUT_OFFICIAL=0,OBSTACLE_SECTION_LAYOUT_CHECK_ALL=1;
int sectionLayoutMode=0;
bool parkingEntryActive=false,parkingEntryObserving=false,parkingEntryScouting=false;
bool parkingEntryJoining=false,parkingEntryTestHold=false,runtimeTestMode=false;
bool optimizedBuilt=false,lapBoundaryPending=false,laterLapPlanRejected=false,lapBoundaryHoldLogged=false,lapFinishPending=false;
uint8_t completedLaps=0,runtimeLapTarget=3;bool finished=false;
PathPoint optimizedPath[OBSTACLE_MAX_PATH_WAYPOINTS];
bool plannedClearanceSnapshotValid[OBSTACLE_SEAT_COUNT]={};
ObstacleClearanceSample plannedClearanceAtInjection[OBSTACLE_SEAT_COUNT];
bool obstacle_path_get_planned_clearance(uint8_t,ObstacleClearanceSample&){return false;}
'''
    fixture += extra
    signatures = (
        'uint8_t sectionInferredEmpty(', 'bool stationResolved(', 'bool allStationsResolved(',
        'bool withinCornerGate(', 'bool nearCorner(', 'bool isExtremeAdjacentPair(',
        'bool targetsOuterExtreme(', 'bool hasConfirmedExtremeAdjacentPair(',
        'bool isSecondExtremeAdjacentSeat(', 'bool upcomingAdjacentStationUnresolved(',
        'float validatedClearanceForSeat(', 'float optimizedClearanceForSeat(',
        'bool optimizedUsesOuterPlateau(', 'bool laterLapMapValid()',
        'void preserveLaterLapSeam(', 'bool laterLapRouteSafe(', 'void roundKnownCornerPairs(',
        'bool buildOptimizedPath()', 'bool completePendingLap()', 'void updateProgress(',
        'PathPoint findLookahead(')
    fixture += '\n'.join(block(source,source.index(s)) for s in signatures)
    fixture += r'''
void resetState(){
 completedLaps=0;finished=false;optimizedBuilt=false;lapBoundaryPending=false;
 laterLapPlanRejected=false;lapBoundaryHoldLogged=false;lapFinishPending=false;runtimeTestMode=false;
 runtimeLapTarget=3;progressIndex=0;sectionLayoutMode=0;
 for(auto &d:discoveryStations)d.observedClear=true;
}
void learned(){
 memcpy(livePath,baselinePath,sizeof(PathPoint)*pathLength);
 for(int i=0;i<OBSTACLE_SEAT_COUNT;++i)if(seats[i].confirmed){
  const float clearance=validatedClearanceForSeat(i);
  displaceForSeat(livePath,i,clearance,(routeTurnSign>0&&i==5&&!seats[i].red)||
    fabsf(clearance-OBSTACLE_OUTER_SAFE_CLEARANCE_MM)<.1f);
 }
}
bool stateChecks(){
 resetState();baseline();learned();lapBoundaryPending=true;
 discoveryStations[7].observedClear=false;
 if(completePendingLap()||completedLaps||!lapBoundaryPending)return false;
 discoveryStations[7].observedClear=true;
 if(!completePendingLap()||completedLaps!=1||!optimizedBuilt)return false;
 if(completePendingLap()||completedLaps!=1)return false;
 lapBoundaryPending=true;if(!completePendingLap()||completedLaps!=2||finished)return false;
 lapBoundaryPending=true;if(!completePendingLap()||completedLaps!=3||finished||!lapFinishPending)return false;
 PositionEstimate finishPose;finishPose.x_mm=livePath[3].x;finishPose.y_mm=livePath[3].y;
 updateProgress(livePath,finishPose);
 if(!finished||lapFinishPending||completedLaps!=3)return false;
 resetState();baseline();learned();runtimeLapTarget=1;lapBoundaryPending=true;
 if(!completePendingLap()||completedLaps!=1||!finished||optimizedBuilt)return false;
 resetState();baseline();
 seats[6].confirmed=seats[7].confirmed=true;
 if(laterLapMapValid())return false;
 seats[7].confirmed=false;seats[8].confirmed=true;
 if(laterLapMapValid())return false; // Official middle plus an end.
 sectionLayoutMode=1;if(!laterLapMapValid())return false;
 seats[10].confirmed=true;if(!laterLapMapValid())return false; // O3 three stations.
 resetState();baseline();learned();runtimeTestMode=true;
 discoveryStations[7].observedClear=false;lapBoundaryPending=true;
 if(!completePendingLap()||completedLaps!=1||optimizedBuilt)return false;
 return true;
}
int failReason=0,failSeat=-1;float failX=0,failY=0,failH=0;
bool driveTwoLaps(float speedCap,float &minimumWall,float &minimumPillar){
 failReason=1;failSeat=-1;
 minimumWall=minimumPillar=1e9f;
 const auto start=livePath[0];float x=start.x,y=start.y;
 float heading=connectorRouteHeading(livePath,0)*PI/180;
 lapBoundaryPending=true;if(!completePendingLap())return false;
 uint8_t witnessed[24]={},passCounts[24]={};
 float previousForward[24];for(auto &f:previousForward)f=NAN;
 // Outer avoidance corners lengthen the route beyond two baseline laps.
 for(int step=0;step<5000;++step){
  failX=x;failY=y;failH=heading*180/PI;
  const PathPoint *route=optimizedBuilt?optimizedPath:livePath;
  PositionEstimate pose;pose.x_mm=x;pose.y_mm=y;pose.heading_deg=heading*180/PI;
  updateProgress(route,pose);
  if(finished){
   for(int seat=0;seat<24;++seat)if(seats[seat].confirmed&&passCounts[seat]<2){failReason=6;failSeat=seat;return false;}
   return completedLaps==3;
  }
  if(lapBoundaryPending||laterLapPlanRejected)return false;
  route=optimizedBuilt?optimizedPath:livePath;
  if(y < -1100 && (!parking_start_footprint::safe(x,y,heading*180/PI,-1,false)||
     !parking_start_footprint::safe(x,y,heading*180/PI,1,false))){
   failReason=5;return false;
  }
  parking_start_footprint::Quad footprints[2][6];
  parking_start_footprint::robotQuads(x,y,heading*180/PI,-1,0,footprints[0]);
  parking_start_footprint::robotQuads(x,y,heading*180/PI,1,0,footprints[1]);
  for(int seat=0;seat<OBSTACLE_SEAT_COUNT;++seat){
   if(seat && !seats[seat].confirmed)continue;
   ObstacleClearanceSample sample{};
   if(!calculateClearanceAtPose(seats[seat],x,y,heading*180/PI,sample))return false;
   minimumWall=fminf(minimumWall,sample.wallMm);
   if(seats[seat].confirmed)minimumPillar=fminf(minimumPillar,sample.pillarMm);
   if(sample.wallMm<=0||(seats[seat].confirmed&&sample.pillarMm<=0)){
    failReason=2;failSeat=seat;return false;
   }
   if(seats[seat].confirmed&&witnessed[seat]!=completedLaps){
    const float phase=cyclicDistanceForward(seats[seat].pathDistanceMm,
                                            baselinePath[progressIndex].distanceMm);
    {
     const float ref=seats[seat].headingDeg*PI/180;
     float minForward=1e9f,minSide=1e9f,maxSide=-1e9f;
     for(const auto &bodies:footprints){
      for(const auto &body:bodies)for(const auto &p:body.p){
      const float px=p.x-seats[seat].x;
      const float py=p.y-seats[seat].y;
      minForward=fminf(minForward,px*cosf(ref)+py*sinf(ref));
      const float side=-px*sinf(ref)+py*cosf(ref);
      minSide=fminf(minSide,side);maxSide=fmaxf(maxSide,side);
      }
     }
     if(phase<1000 && std::isfinite(previousForward[seat]) &&
        previousForward[seat]<=0 && minForward>0){
      if(seats[seat].red?maxSide>=-25:minSide<=25){failReason=3;failSeat=seat;return false;}
      witnessed[seat]=completedLaps;
      ++passCounts[seat];
     }
     previousForward[seat]=minForward;
    }
   }
  }
  float lookahead=OBSTACLE_LATER_LAP_LOOKAHEAD_MM;
  if(nearCorner(baselinePath[progressIndex].distanceMm))lookahead*=OBSTACLE_LOOKAHEAD_CORNER_SCALE;
  const auto target=findLookahead(route,pose,lookahead);
  const float dx=target.x-x,dy=target.y-y;
  const float forward=dx*cosf(heading)+dy*sinf(heading);
  const float lateral=-dx*sinf(heading)+dy*cosf(heading);
  const float requestedCurvature=2*lateral/fmaxf(1,dx*dx+dy*dy);
  const float requiredSteering=-atanf(OBSTACLE_WHEELBASE_MM*requestedCurvature)*180/PI;
  if(!std::isfinite(requiredSteering)){failReason=4;return false;}
  const float steering=static_cast<int>(clampFloat(requiredSteering,-OBSTACLE_MAX_PURSUIT_STEERING_DEG,
                                 OBSTACLE_MAX_PURSUIT_STEERING_DEG));
  const float curvature=-tanf(steering*PI/180)/OBSTACLE_WHEELBASE_MM;
  const float next=heading+curvature*5;
  if(fabsf(curvature)>1e-6){x+=(sinf(next)-sinf(heading))/curvature;
   y+=(cosf(heading)-cosf(next))/curvature;}
  else{x+=5*cosf(heading);y+=5*sinf(heading);}
  heading=next;
 }
 return false;
}
int main(){
 if(!stateChecks()){std::cout<<"STATE FAIL\n";return 2;}
 std::cout<<"STATE PASS\n";
 int direction,id=0,colors[24];float cap;
 while(std::cin>>direction>>cap){
  for(int &c:colors)if(!(std::cin>>c))return 3;
  routeTurnSign=direction;resetState();baseline();
  for(int i=0;i<24;++i){seats[i].confirmed=colors[i]!=0;
   seats[i].red=colors[i]==2;seats[i].injected=colors[i]!=0;}
  learned();float wall,pillar;
  const bool pass=driveTwoLaps(cap,wall,pillar);
  std::cout<<"CASE "<<id++<<" "<<pass<<" "<<wall<<" "<<pillar<<" "
    <<failReason<<" "<<failSeat<<" "<<failX<<" "<<failY<<" "<<failH<<"\n";
 }
}
'''
    # baseline() must select the supplied travel direction at runtime.
    fixture = fixture.replace('anchor.heading_deg=180;', 'anchor.heading_deg=routeTurnSign>0?0:180;')
    destination = ROOT/'local_workspace/later-laps'
    destination.mkdir(parents=True,exist_ok=True)
    cpp=destination/'check.cpp';cpp.write_text(fixture)
    compiler=shutil.which('g++') or shutil.which('clang++')
    if not compiler:raise RuntimeError('Host C++ compiler on PATH required')
    exe=destination/'check.exe'
    subprocess.run([compiler,'-std=c++17','-O2','-I',str(ROOT/'include'),str(cpp),'-o',str(exe)],check=True)
    # All 28 distinct normal-section layouts: 12 singletons plus 16 end pairs.
    section=[]
    for station,side,color in itertools.product(range(3),range(2),(1,2)):
        colors=[0]*6;colors[2*station+side]=color;section.append(colors)
    for first,last,c1,c2 in itertools.product(range(2),range(2),(1,2),(1,2)):
        colors=[0]*6;colors[first]=c1;colors[4+last]=c2;section.append(colors)
    cases=[]
    for direction,cap in itertools.product((-1,1),(0,140)):
        inner=0 if direction<0 else 1
        start=[[0]*6]
        for station,color in itertools.product(range(3),(1,2)):
            colors=[0]*6;colors[2*station+inner]=color;start.append(colors)
        for c1,c2 in itertools.product((1,2),repeat=2):
            colors=[0]*6;colors[inner]=c1;colors[4+inner]=c2;start.append(colors)
        for layout in start:
            for normal in section:
                # Repeat the normal layout on remaining straights to exercise
                # same/different colour approaches through every corner.
                cases.append(dict(direction=direction,speed_cap=cap,colors=layout+normal*3))
    # All ordered neighbouring layouts, alternating through the other three
    # straights, expose different colours and placement sides at each corner.
    for direction,a,b in itertools.product((-1,1),section,section):
        cases.append(dict(direction=direction,speed_cap=0,colors=[0]*6+a+b+a))
    inputs='\n'.join(' '.join(map(str,[c['direction'],c['speed_cap'],*c['colors']])) for c in cases)+'\n'
    run=subprocess.run([str(exe)],input=inputs,text=True,capture_output=True,check=True)
    (destination/'output.txt').write_text(run.stdout)
    for line in run.stdout.splitlines():
        if line.startswith('CASE '):
            _,number,passed,wall,pillar,reason,seat,x,y,h=line.split();cases[int(number)].update(
                passed=bool(int(passed)),minimum_wall_mm=float(wall),minimum_pillar_mm=float(pillar),
                failure_reason=0 if int(passed) else int(reason),
                failure_seat=int(seat),pose=[float(x),float(y),float(h)])
    failed=[c for c in cases if not c['passed']]
    report=dict(total=len(cases),failed=len(failed),cases=cases,
                limitations=['Known explicit layouts, ideal pursuit, no images or sensor/motor dynamics',
                             'Repeated sections and all ordered neighbouring layouts, not exhaustive full-field layouts'])
    (destination/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print('STATE PASS; LATER LAPS',len(cases),'FAILED',len(failed))
    for case in failed[:12]:print('FAIL',case)
    if failed:raise SystemExit(1)


if __name__=='__main__':main()
