"""Actual production CCW short scan/connector and front passing calculation.

Known colours and ideal tracking; no camera or robot acceptance. Archived
second-edge poses are translated east by the removed continuation distance.
This estimates the changed reference, rather than claiming new measurements.
"""
import itertools
import json
import shutil
import subprocess

from check_cw_start_planner import fixture_source
from check_connector_servo_resume import block
from review_ccw_start import ROOT, source_poses
from parking_exit_swept_search import Pose, collision, robot_polygons


def main():
    source = (ROOT/'src/obstacle_path.cpp').read_text()
    fixture = fixture_source().split('int main()')[0]
    fixture = fixture.replace('int routeTurnSign=-1,parkingEntryTargetStation=1;',
                              'int routeTurnSign=1,parkingEntryTargetStation=2;')
    fixture = fixture.replace('bool parkingCwShortStart=true,', 'bool parkingCwShortStart=false,')
    fixture = fixture.replace('bool parkingCcwShortStart=false;', 'bool parkingCcwShortStart=true;')
    fixture = fixture.replace('anchor.heading_deg=180;', 'anchor.heading_deg=0;')
    fixture = fixture.replace('uint16_t pathLength=0;', 'uint16_t pathLength=0,progressIndex=0;')
    fixture += block(source, source.index('PathPoint findLookahead('))+'\n'
    fixture += block(source, source.index('bool withinCornerGate('))+'\n'
    fixture += block(source, source.index('bool nearCorner('))+'\n'
    # Capture the actual accepted rollout's endpoint; instrumentation only.
    fixture = fixture.replace('struct NullLog {', 'float rolloutX=0,rolloutY=0,rolloutH=0;\nstruct NullLog {')
    marker = '        // Check both existing front/rear capsules'
    assert marker in fixture
    fixture = fixture.replace(marker,
        '        rolloutX=x;rolloutY=y;rolloutH=heading*180/PI;\n'+marker)
    fixture += r'''
// Additional ideal normal-route continuation uses production findLookahead,
// adaptiveLookahead, nearestPathIndex and capsule checks. Does not model ToF,
// vision/holds or motor dynamics. It witnesses full directional front passing.
bool frontPass(float x,float y,float h,uint16_t merge,int color){
 if(!color)return true;
 progressIndex=merge;float heading=h*PI/180;
 const float inflatedHalf=OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM;
 for(int step=0;step<800;++step){
  progressIndex=nearestPathIndex(livePath,x,y,progressIndex,OBSTACLE_PATH_PROGRESS_WINDOW);
  ObstacleClearanceSample sample{};
  for(int rear=0;rear<2;++rear){
   const float offset=rear?inflatedHalf:0;
   if(!calculateClearanceAtPose(seats[5],x-offset*cosf(heading),
      y-offset*sinf(heading),h,sample)||sample.wallMm<=0||sample.pillarMm<=0)return false;
  }
  // A conservative rectangle contains the entire physical body/wheels.
  float minX=1e9f,minY=1e9f,maxY=-1e9f;
  for(float bx : {-40.f,125.f})for(float by : {-80.f,80.f}){
   const float px=x+bx*cosf(heading)-by*sinf(heading);
   const float py=y+bx*sinf(heading)+by*cosf(heading);
   minX=fminf(minX,px);minY=fminf(minY,py);maxY=fmaxf(maxY,py);
  }
  if(minX>500)return color==1?minY>-875:maxY<-925;
  PositionEstimate pose;pose.x_mm=x;pose.y_mm=y;pose.heading_deg=h;
  const auto target=findLookahead(livePath,pose,adaptiveLookahead(livePath[progressIndex].speedMmS));
  const float dx=target.x-x,dy=target.y-y;
  const float forward=dx*cosf(heading)+dy*sinf(heading);
  const float lateral=-dx*sinf(heading)+dy*cosf(heading);
  const float curvature=2*lateral/fmaxf(1,dx*dx+dy*dy);
  if(forward<=1||fabsf(atanf(OBSTACLE_WHEELBASE_MM*curvature)*180/PI)>
     OBSTACLE_MAX_PURSUIT_STEERING_DEG)return false;
  const float next=heading+curvature*2;
  if(fabsf(curvature)>1e-6){x+=(sinf(next)-sinf(heading))/curvature;
    y+=(cosf(heading)-cosf(next))/curvature;}
  else{x+=2*cosf(heading);y+=2*sinf(heading);}
  heading=next;h=heading*180/PI;
 }
 return false;
}
// Later return through both behind places, activating each at the same 800 mm
// bound as firmware. Explicit colours bypass perception but not geometry.
bool behindReturn(int back,int middle,int front,float speedCap){
 baseline();memcpy(livePath,baselinePath,sizeof(PathPoint)*pathLength);
 const int colors[3]={back,middle,front};bool injected[3]={false,false,false};
 for(int s=0;s<3;++s){seats[2*s+1].confirmed=colors[s]!=0;seats[2*s+1].red=colors[s]==2;}
 if(front){displaceForSeat(livePath,5,OBSTACLE_LAP1_CLEARANCE_MM,front==1);injected[2]=true;}
 uint16_t begin=0;
 for(uint16_t i=0;i<pathLength;++i)
  if(baselinePath[i].distanceMm<=loopLengthMm-1800)begin=i;
 progressIndex=begin;float x=baselinePath[begin].x,y=baselinePath[begin].y;
 float heading=baselinePath[begin].headingDeg*PI/180;bool crossed[2]={!back,!middle};
 for(int step=0;step<1300;++step){
  progressIndex=nearestPathIndex(livePath,x,y,progressIndex,OBSTACLE_PATH_PROGRESS_WINDOW);
  for(int s=0;s<2;++s)
   if(colors[s]&&!injected[s]&&cyclicDistanceForward(baselinePath[progressIndex].distanceMm,
       seats[2*s+1].pathDistanceMm)<=OBSTACLE_PARKING_CCW_STORED_SEAT_APPROACH_MM){
    displaceForSeat(livePath,2*s+1,OBSTACLE_LAP1_CLEARANCE_MM,false);injected[s]=true;
   }
  for(int s=0;s<3;++s){
   if(!colors[s])continue;
   ObstacleClearanceSample clearance{};
   for(int rear=0;rear<2;++rear){
    const float offset=rear?OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM:0;
    if(!calculateClearanceAtPose(seats[2*s+1],x-offset*cosf(heading),y-offset*sinf(heading),
       heading*180/PI,clearance)||clearance.wallMm<=0||clearance.pillarMm<=0)return false;
   }
  }
  // Evaluate directional crossings only in the returned southern straight.
  if(y<-600 && cosf(heading)>0){
   float minX=1e9f,minY=1e9f,maxY=-1e9f;
   for(float bx:{-40.f,125.f})for(float by:{-80.f,80.f}){
    minX=fminf(minX,x+bx*cosf(heading)-by*sinf(heading));
    const float py=y+bx*sinf(heading)+by*cosf(heading);
    minY=fminf(minY,py);maxY=fmaxf(maxY,py);
   }
   for(int s=0;s<2;++s)if(!crossed[s]&&minX>-500+500*s){
    if(colors[s]==1?minY<=-875:maxY>=-925)return false;
    crossed[s]=true;
   }
   if(crossed[0]&&crossed[1]&&x>70)return true;
  }
  PositionEstimate pose;pose.x_mm=x;pose.y_mm=y;pose.heading_deg=heading*180/PI;
  float lookahead=adaptiveLookahead(speedCap>0?fminf(livePath[progressIndex].speedMmS,speedCap):
                                  livePath[progressIndex].speedMmS);
  if(nearCorner(baselinePath[progressIndex].distanceMm))lookahead*=OBSTACLE_LOOKAHEAD_CORNER_SCALE;
  const auto target=findLookahead(livePath,pose,lookahead);
  const float dx=target.x-x,dy=target.y-y;
  const float forward=dx*cosf(heading)+dy*sinf(heading);
  const float lateral=-dx*sinf(heading)+dy*cosf(heading);
  const float curvature=2*lateral/fmaxf(1,dx*dx+dy*dy);
  if(forward<=1||fabsf(atanf(OBSTACLE_WHEELBASE_MM*curvature)*180/PI)>
       OBSTACLE_MAX_PURSUIT_STEERING_DEG)return false;
  const float next=heading+curvature*2;
  if(fabsf(curvature)>1e-6){x+=(sinf(next)-sinf(heading))/curvature;
   y+=(cosf(heading)-cosf(next))/curvature;}
  else{x+=2*cosf(heading);y+=2*sinf(heading);}
  heading=next;
 }
 return false;
}
int main(){
 float x,y,h,ex,ey,eh;int back,middle,front,id=0,failed=0;
 while(std::cin>>x>>y>>h>>back>>middle>>front>>ex>>ey>>eh){
  baseline();memcpy(livePath,baselinePath,sizeof(PathPoint)*pathLength);
  int colors[3]={back,middle,front};
  for(int s=0;s<3;++s){
   // Behind observations deliberately remain unresolved in some cases;
   // known occupied seats still guard collision, but do not displace route.
   discoveryStations[s].observedClear=s==2&&colors[s]==0;
   if(colors[s]){seats[2*s+1].confirmed=true;seats[2*s+1].red=colors[s]==2;}
   if(s==2&&colors[s]){seats[5].injected=true;
    displaceForSeat(livePath,5,OBSTACLE_LAP1_CLEARANCE_MM,front==1);}
  }
  PositionEstimate start;start.x_mm=x;start.y_mm=y;start.heading_deg=h;
  const bool arc=preflightCwStartArc(start,parkingEntryScanArcMm()+parkingEntryScoutArcMm());
  buildParkingEntryPath(start);const auto end=parkingEntryPath[parkingEntryLength-1];
  PositionEstimate join;join.x_mm=end.x+ex;join.y_mm=end.y+ey;join.heading_deg=end.headingDeg+eh;
  uint16_t merge=0;const bool connector=buildParkingEntryConnector(join,livePath,merge);
  bool pass=false;
  if(connector){
   if(!connectorRolloutFeasible(join,parkingEntryConnectorLookaheadMm,5,
       front?5:-1,middle?3:-1,livePath,merge,true))return 2;
   pass=frontPass(rolloutX,rolloutY,rolloutH,merge,front);
  }
  if(!arc||!connector||!pass)++failed;
  std::cout<<"CASE "<<id++<<" "<<arc<<" "<<connector<<" "<<pass<<" "
    <<end.x<<" "<<end.y<<" "<<end.headingDeg<<"\n";
 }
 std::cout<<"TOTAL "<<id<<" FAILED "<<failed<<"\n";
 const int layouts[11][3]={{1,0,0},{2,0,0},{0,1,0},{0,2,0},{0,0,1},{0,0,2},
    {1,0,1},{1,0,2},{2,0,1},{2,0,2},{0,0,0}};
 for(int layout=0;layout<11;++layout)for(float cap:{0.f,140.f,260.f})
  std::cout<<"RETURN "<<layout<<" "<<cap<<" "<<behindReturn(
      layouts[layout][0],layouts[layout][1],layouts[layout][2],cap)<<"\n";
 // Python records detailed failures before deciding whether to reject.
}
'''
    destination = ROOT/'local_workspace/ccw-start-planner'
    destination.mkdir(parents=True, exist_ok=True)
    cpp = destination/'check.cpp'; cpp.write_text(fixture)
    compiler = shutil.which('g++') or shutil.which('clang++')
    if not compiler:
        raise RuntimeError('Host C++ compiler on PATH required')
    exe = destination/'check.exe'
    subprocess.run([compiler, '-std=c++17', '-O2', '-I', str(ROOT/'include'),
                    str(cpp), '-o', str(exe)], check=True)
    layouts = [(1,0,0),(2,0,0),(0,1,0),(0,2,0),(0,0,1),(0,0,2),
               (1,0,1),(1,0,2),(2,0,1),(2,0,2),(0,0,0)]
    rows = source_poses()
    # Changed portion of the prefix: shorter centred localization reverse.
    # Use archived corrected endpoints and measured exit poses, with +/-5 mm
    # XY / +/-1 degree and 8 mm overshoot. This is geometry only: actual ToF
    # edge timing and correction changes need new logs.
    prefix = []
    for row in rows:
        if 'exit_pose' not in row:
            continue
        start = row['exit_pose']; end = row['entry_pose']
        samples = 1+int(abs(start[0]-(end[0]+70))/2)
        failed = 0
        for dx,dy,dh in itertools.product((-5,0,5),(-5,0,5),(-1,0,1)):
            for n in range(samples+5):
                fraction = n/samples
                x = start[0]+fraction*(end[0]+70-start[0])+dx
                y = start[1]+fraction*(end[1]-start[1])+dy
                heading = start[2]+dh
                pose = Pose(480-x,y+1500,180-heading)
                polygons = robot_polygons(pose,0)
                bad = any(collision(pose,0,gap) for gap in (242.5,252.5)) or \
                    min(480-px for poly in polygons for px,_ in poly)<=0
                if bad:
                    failed += 1
                    break
        prefix.append(dict(source=row['path'],cases=27,failed=failed))
    cases = []
    # Perturb both the reference and settled scan, independently; nominal
    # reference plus settled-scan grid and reference grid plus nominal return.
    for row, layout in itertools.product(rows, layouts):
        reference = row['entry_pose']
        for stage in ('reference', 'scan'):
            headings = (-1,0,1) if stage == 'reference' else (-3,0,3)
            for delta in itertools.product((-5,0,5),(-5,0,5),headings):
                cases.append(dict(source=row['path'], layout=layout, stage=stage,
                                  delta=delta, reference=reference))
    lines = []
    for c in cases:
        delta = c['delta']; ref = c['reference']
        start = [ref[0]+70, ref[1], ref[2]]
        settled = [0,0,0]
        if c['stage'] == 'reference':
            start = [p+d for p,d in zip(start,delta)]
        else:
            settled = delta
        lines.append(' '.join(map(str,[*start,*c['layout'],*settled])))
    run = subprocess.run([str(exe)], input='\n'.join(lines)+'\n', text=True,
                         capture_output=True, check=True)
    (destination/'output.txt').write_text(run.stdout)
    returns = []
    for line in run.stdout.splitlines():
        if line.startswith('CASE '):
            _, number, arc, connector, passing, x, y, h = line.split()
            c = cases[int(number)]
            c.update(arc=bool(int(arc)), connector=bool(int(connector)),
                     front_passing=bool(int(passing)), scan_pose=[float(x),float(y),float(h)])
        elif line.startswith('RETURN '):
            _, layout, cap, passed = line.split()
            returns.append(dict(layout=layouts[int(layout)], speed_cap=float(cap), passed=bool(int(passed))))
    summary = []
    for layout in layouts:
        group = [c for c in cases if c['layout'] == layout]
        summary.append(dict(layout=layout,total=len(group),
                            arc=sum(c['arc'] for c in group),
                            connector=sum(c['connector'] for c in group),
                            front_passing=sum(c['front_passing'] for c in group)))
    report = dict(matrix=summary, source_poses=rows, cases=cases, behind_returns=returns,
                  localization_prefix=prefix,
                  limitations=['70 mm earlier reference estimated from archived poses',
                               'Known colours; no image acceptance or ToF transitions',
                               'Ideal kinematic normal-route continuation only'])
    (destination/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    for row in summary:
        print(row)
    failures = [c for c in cases if not(c['arc'] and c['connector'] and c['front_passing'])]
    print('TOTAL',len(cases),'FAILED',len(failures))
    print('RETURNS',len(returns),'FAILED',sum(not r['passed'] for r in returns))
    print('PREFIX',sum(p['cases'] for p in prefix),'FAILED',sum(p['failed'] for p in prefix))
    for r in returns:
        if not r['passed']:
            print('RETURN FAIL',r)
    for c in failures[:8]:
        print('FAIL',c)
    if failures or any(not r['passed'] for r in returns) or any(p['failed'] for p in prefix):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
