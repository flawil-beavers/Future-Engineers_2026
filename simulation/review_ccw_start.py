"""Read-only CCW audit with extracted production C++ planner functions.

Firmware remains unchanged. Results are saved below local_workspace. Explicit
fixture colours test geometry, not camera accuracy or physical acceptance.
"""
import hashlib
import itertools
import json
import math
import re
import shutil
import subprocess
from pathlib import Path

from check_cw_start_planner import fixture_source
from parking_entry_scout_sim import camera_geometry, scout_pose
from parking_exit_swept_search import Pose, collision, robot_polygons

ROOT = Path(__file__).resolve().parents[1]


def source_poses():
    paths = [ROOT/'simulation/evidence/parking_exit_diagnostics'/
             f'20260926_log_{n}_ccw.txt' for n in (388,389)]
    paths += [ROOT/'simulation/fixtures/parking_entry_scout'/f'log_{n}.txt'
              for n in (364,365,369)]
    rows = []
    for path in paths:
        text = path.read_text(errors='replace')
        primary = re.search(r'\[PARK ENTRY RESULT\].*pose_x_y_heading=([^ ]+)',text)
        entry = re.search(r'discovery armed turn=CCW.*start_x_y_heading=([^\s]+)',text)
        if not primary:
            continue
        row = dict(path=path.relative_to(ROOT).as_posix(),
                   sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                   primary_pose=list(map(float,primary.group(1).split('/'))))
        if entry:
            row['entry_pose']=list(map(float,entry.group(1).split('/')))
        exit_pose = re.search(r'\[PARK EXIT FIELD POSE\] x_y_heading=([^\s]+)',text)
        if exit_pose:
            row['exit_pose'] = list(map(float,exit_pose.group(1).split('/')))
        rows.append(row)
    return rows


def geometry(rows):
    result=[]
    for row in rows:
        pose=row['primary_pose']
        scouts=[scout_pose(*pose,'CCW',s) for s in range(0,94,2)]
        middle=scout_pose(*pose,'CCW',85)
        # Field-to-parking transform is a reflection: steering flips sign.
        pink=any(collision(Pose(480-x,y+1500,180-h),-50,gap)
                 for x,y,h in scouts for gap in (242.5,252.5))
        body_min=min(480-px for x,y,h in scouts
                     for polygon in robot_polygons(Pose(480-x,y+1500,180-h),-50)
                     for px,_ in polygon)
        result.append(dict(source=row['path'],primary_pose=pose,
                           forward_bearing_range=camera_geometry(pose,(500,-900)),
                           scout_pose=middle,
                           middle_bearing_range=camera_geometry(middle,(0,-900)),
                           left_bearing_range=camera_geometry(middle,(-500,-900)),
                           minimum_body_x_mm=body_min,
                           sampled_pink_collision=pink))
    return result


def passing_audit(cases):
    # Only classify a witnessed full crossing. A connector ending before a
    # sign's line does not establish the later passing side.
    for case in cases:
        case['witnessed_crossings']=[]
        for station,color in enumerate(case['layout']):
            if not color:
                continue
            line_x=-500+500*station
            for x,y,h in case['trace']:
                polygons=robot_polygons(Pose(480-x,y+1500,180-h),0)
                minimum_x=min(480-px for poly in polygons for px,_ in poly)
                # For the audit, initial scan/scout already straddled or moved
                # behind the middle. Stop at the first wholly eastward body.
                if station==1 and minimum_x>line_x:
                    points=[(480-px,py-1500) for poly in polygons for px,py in poly]
                    minimum_y=min(py for _,py in points)
                    maximum_y=max(py for _,py in points)
                    good=(minimum_y>-875) if color==1 else (maximum_y<-925)
                    case['witnessed_crossings'].append(dict(
                        station=station,color=color,pose=[x,y,h],
                        minimum_body_x=minimum_x,body_y=[minimum_y,maximum_y],
                        correct_side=good))
                    break


def main():
    fixture=fixture_source()
    fixture=fixture[:fixture.index('int main()')]
    fixture=fixture.replace('int routeTurnSign=-1,parkingEntryTargetStation=1;',
                            'int routeTurnSign=1,parkingEntryTargetStation=2;')
    fixture=fixture.replace('bool parkingCwShortStart=true,','bool parkingCwShortStart=false,')
    fixture=fixture.replace('anchor.heading_deg=180;','anchor.heading_deg=0;')
    # Instrument the actual rollout to export predicted poses only; no
    # controller, selection, clearance, stop or acceptance gate is replaced.
    fixture=fixture.replace('struct NullLog {',
                            'bool traceEnabled=false;\nstruct NullLog {')
    fixture=fixture.replace('        // Check both existing front/rear capsules',
        '        if(traceEnabled) std::cout<<"POSE "<<x<<" "<<y<<" "<<heading*180/PI<<"\\n";\n'
        '        // Check both existing front/rear capsules')
    fixture+=r'''
int main(){
 float x,y,h;int left,middle,front,id=0;
 while(std::cin>>x>>y>>h>>left>>middle>>front){
  baseline();memcpy(livePath,baselinePath,sizeof(PathPoint)*pathLength);
  int colors[3]={left,middle,front};
  for(int s=0;s<3;++s){
   discoveryStations[s].observedClear=colors[s]==0;
   if(colors[s]){seats[2*s+1].confirmed=true;seats[2*s+1].red=colors[s]==2;
    seats[2*s+1].injected=true;
    displaceForSeat(livePath,2*s+1,OBSTACLE_LAP1_CLEARANCE_MM,
                    s==2&&colors[s]==1);}
  }
  PositionEstimate start;start.x_mm=x;start.y_mm=y;start.heading_deg=h;
  float maxForward=-1e9f;
  for(int index=0;index<pathLength;++index){
   const float phase=cyclicDistanceForward(livePath[index].distanceMm,seats[5].pathDistanceMm);
   if(phase>=OBSTACLE_PARKING_ENTRY_CONNECTOR_MIN_BEFORE_PILLAR_MM-0.1f &&
      phase<=OBSTACLE_PARKING_ENTRY_CONNECTOR_MAX_BEFORE_PILLAR_MM+0.1f &&
      fabsf(wrap180(h-livePath[index].headingDeg))<=100){
    const float forward=(livePath[index].x-x)*cosf(h*PI/180)+(livePath[index].y-y)*sinf(h*PI/180);
    maxForward=fmaxf(maxForward,forward);
   }
  }
  uint16_t merge=0;bool pass=buildParkingEntryConnector(start,livePath,merge);
  std::cout<<"CASE "<<id++<<" "<<pass<<" "<<merge<<" "<<parkingEntryConnectorLookaheadMm<<"\n";
  std::cout<<"GATE "<<maxForward<<"\n";
  if(pass){
   traceEnabled=true;
   if(!connectorRolloutFeasible(start,parkingEntryConnectorLookaheadMm,
          5,front?5:-1,middle?3:-1,livePath,merge,front==1))return 2;
   traceEnabled=false;
  }
  std::cout<<"END\n";
 }
}
'''
    destination=ROOT/'local_workspace/ccw-start-review'
    destination.mkdir(parents=True,exist_ok=True)
    cpp=destination/'audit.cpp';cpp.write_text(fixture)
    compiler=shutil.which('g++') or shutil.which('clang++')
    if not compiler:
        raise RuntimeError('Host C++ compiler on PATH is required')
    exe=destination/'audit.exe'
    subprocess.run([compiler,'-std=c++17','-O2','-I',str(ROOT/'include'),str(cpp),'-o',str(exe)],check=True)
    rows=source_poses()
    layouts=[(1,0,0),(2,0,0),(0,1,0),(0,2,0),(0,0,1),(0,0,2),
             (1,0,1),(1,0,2),(2,0,1),(2,0,2),(0,0,0)]
    cases=[dict(source=row['path'],pose=row['primary_pose'],layout=layout,
                perturbation=[dx,dy,dh]) for row in rows for layout in layouts
           for dx,dy,dh in itertools.product((-5,0,5),(-5,0,5),(-3,0,3))]
    inputs='\n'.join(' '.join(map(str,[*(p+d for p,d in zip(c['pose'],c['perturbation'])),
                                     *c['layout']])) for c in cases)+'\n'
    run=subprocess.run([str(exe)],input=inputs,text=True,capture_output=True,check=True)
    (destination/'rollout.txt').write_text(run.stdout)
    current=None
    for line in run.stdout.splitlines():
        if line.startswith('CASE '):
            _,number,passed,merge,lookahead=line.split()
            current=cases[int(number)];current.update(passed=bool(int(passed)),
                merge=int(merge),lookahead_mm=float(lookahead),trace=[])
        elif line.startswith('POSE '):
            current['trace'].append(list(map(float,line.split()[1:])))
        elif line.startswith('GATE '):
            current['maximum_eligible_forward_mm']=float(line.split()[1])
    passing_audit(cases)
    summary=[]
    for layout in layouts:
        relevant=[c for c in cases if c['layout']==layout]
        nominal=[c for c in relevant if c['perturbation']==[0,0,0]]
        summary.append(dict(layout=layout,nominal_pass=sum(c['passed'] for c in nominal),
                            nominal_total=len(nominal),grid_pass=sum(c['passed'] for c in relevant),
                            grid_total=len(relevant),
                            witnessed_wrong_middle_crossings=sum(
                                any(not cross['correct_side'] for cross in c['witnessed_crossings'])
                                for c in relevant)))
    report=dict(source_poses=rows,view_and_sweep=geometry(rows),
                layouts_order='CCW left/back, middle/back, right/front; 0 empty, 1 GREEN, 2 RED',
                matrix=summary,cases=cases,
                limitations=['Colours supplied explicitly; images not evaluated',
                             'Older primary poses used as geometric inputs, not current physical tests',
                             'Passing side and deferred behind-seat policy require separate audit',
                             'No firmware change, upload or commit'])
    (destination/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    for row in summary:
        print(row)
    for row in report['view_and_sweep']:
        print('VIEW',row['source'],row['forward_bearing_range'],row['middle_bearing_range'],
              row['left_bearing_range'],'body_min_x',row['minimum_body_x_mm'])


if __name__=='__main__':
    main()
