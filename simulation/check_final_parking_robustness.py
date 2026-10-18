"""Broader final-parking audit: arrival poses, true bay gaps and sensor faults.

Uses the production state-machine fixture with independent physical pose and
actual barrier coordinates. Unknown-pose-error cases are reported separately
from known-pose arrivals. All generated artifacts stay in local_workspace.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import shutil
import subprocess

from check_cw_start_planner import ROOT
from check_connector_servo_resume import block
from check_final_parking_sequence import fixture_source


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers',type=int,default=4,choices=range(1,5))
    args=parser.parse_args()
    dest = ROOT/'local_workspace/final-parking-robustness'
    dest.mkdir(parents=True, exist_ok=True)
    fixture = fixture_source()
    start = fixture.index('bool get_tof_diagnostic_snapshot(TofSensor sensor,')
    old = block(fixture, start)
    fixture = fixture.replace(old, r'''
float physicalGap=247.5f;int sensorFault=0;
bool get_tof_diagnostic_snapshot(TofSensor sensor,TofDiagnosticSnapshot &s){
 if(sensorFault==1)return false;
 s={};s.sequence=hostTime/framePeriod;s.sampled_ms=s.sequence*framePeriod;
 if(sensorFault==2)s.sampled_ms=hostTime-1000;
 if(sensorFault==3)s.sequence=1;
 const float wall=parking_module::expectedOuterWallRange(sensor,truePose);
 const float markerRange=wall-200;
 auto beam=parking_module::beamFootprint(sensor,markerRange,truePose);
 bool fixed=beam.maximumX>=480&&beam.minimumX<=500;
 bool moving=beam.maximumX>=480-physicalGap-20&&beam.minimumX<=480-physicalGap;
 if(sensorFault==5)fixed=false;
 if(sensorFault==6)moving=false;
 bool marker=fixed||moving;
 s.selected_raw_distance_mm=wall;s.stored_object_count=1;
 s.objects[0].distance_mm=static_cast<int16_t>(marker?markerRange:wall);
 if(marker){s.stored_object_count=2;s.objects[1].distance_mm=static_cast<int16_t>(wall);}
 for(int i=0;i<s.stored_object_count;++i){
  s.objects[i].hardware_valid=sensorFault!=7;
  s.objects[i].filter_accepted=sensorFault!=8;
 }
 if(sensorFault==4){s.stored_object_count=0;s.selected_raw_distance_mm=-1;}
 return true;
}
''')
    fixture += r'''
#include <limits>
bool physicalSafe(const PositionEstimate &pose,int steering){
 using namespace parking_start_footprint;
 Quad bodies[6];robotQuads(pose.x_mm,pose.y_mm,pose.heading_deg,
     steering>0?1:(steering<0?-1:0),0,bodies);
 const Quad fixed=rectangle(490,-1400,10,100,0);
 const Quad moving=rectangle(480-physicalGap-10,-1400,10,100,0);
 for(const auto &body:bodies){
  for(const auto &p:body.p)if(p.y<=-1500)return false;
  if(overlaps(body,fixed)||overlaps(body,moving))return false;
 }
 // Full field wall/pillar model remains a conservative capsule check.
 for(int i=0;i<OBSTACLE_SEAT_COUNT;++i){const auto &seat=seats[i];
  if(i!=0&&!seat.confirmed)continue;ObstacleClearanceSample c{};
  if(!calculateClearanceAtPose(seat,pose.x_mm,pose.y_mm,pose.heading_deg,c)||
     c.wallMm<=0||(seat.confirmed&&c.pillarMm<=0))return false;}
 return true;
}
bool physicallyContained(){
 const auto saved=hostPose;hostPose=truePose;
 const float gap=parking_module::measuredGapMm;
 const int phase=parking_module::scanPhase;
 parking_module::measuredGapMm=physicalGap;parking_module::scanPhase=5;
 bool yes=parking_module::finalFootprintContained();
 parking_module::measuredGapMm=gap;parking_module::scanPhase=phase;hostPose=saved;
 return yes;
}
int boundaryChecks(){
 routeTurnSign=-1;resetState();baseline();
 for(auto &s:seats)s.confirmed=s.injected=false;
 hostTime=0;encoder=0;hostGyroHealthy=true;physicalGap=247.5f;sensorFault=0;
 // Invalid X/Y must never pass the final containment test or claim success.
 int bad=0,total=0;
 for(int dir:{-1,1})for(int axis:{0,1,2})for(float invalid:{NAN,INFINITY,-INFINITY}){
  parking_module::final_parking_reset();parking_module::routeTurnSign=dir;
  parking_module::scanPhase=5;parking_module::measuredGapMm=physicalGap;
  parking_module::localToField(OBSTACLE_FINAL_PARKING_TARGET_LOCAL_X_MM,
      OBSTACLE_FINAL_PARKING_TARGET_LOCAL_Y_MM,0,
      hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
  if(axis==0)hostPose.x_mm=invalid;
  if(axis==1)hostPose.y_mm=invalid;
  if(axis==2)hostPose.heading_deg=invalid;
  // Avoid an old infinite-heading wrap loop during the negative control:
  // the first NaN-X assertion already reproduces its false completion.
  if(parking_module::finalFootprintContained()){
   std::cout<<"BOUNDARY FAIL invalid pose accepted "<<dir<<" "<<axis<<"\n";
   return 1;
  }
  parking_module::state=parking_module::FP_VERIFY;stop(false);
  parking_module::final_parking_update(dir);++total;
  if(parking_module::final_parking_complete()||!parking_module::final_parking_aborted())++bad;
 }
 // Gyro loss at every phase, including initial preflight and final verify.
 for(int dir:{-1,1})for(int phase=parking_module::FP_IDLE;phase<=parking_module::FP_VERIFY;++phase){
  parking_module::final_parking_reset();parking_module::routeTurnSign=dir;
  parking_module::state=static_cast<parking_module::FinalParkingState>(phase);
  hostPose={-100,-1000,dir>0?0.f:180.f};
  hostTime=200;parking_module::stateStartMs=100;hostGyroHealthy=false;
  dc_state=DC_ENABLED;command=60;
  parking_module::final_parking_update(dir);++total;
  if(parking_module::state!=phase||dc_state!=DC_DISABLED||
     parking_module::final_parking_complete()||parking_module::final_parking_aborted())++bad;
  const auto pausedStart=parking_module::stateStartMs;
  hostTime+=1500;hostGyroHealthy=true;parking_module::final_parking_sensor_resume();
  if(parking_module::stateStartMs!=pausedStart+1500)++bad;
 }
 std::cout<<"BOUNDARY "<<total<<" FAILED "<<bad<<"\n";return bad?1:0;
}
int main(){if(boundaryChecks())return 2;
 TofDiagnosticSnapshot rejected{};rejected.selected_raw_distance_mm=260;
 rejected.stored_object_count=1;rejected.objects[0].distance_mm=260;
 rejected.objects[0].hardware_valid=false;rejected.objects[0].filter_accepted=false;
 if(parking_module::parkingOuterRange(rejected)>0){
  std::cout<<"BOUNDARY FAIL unaccepted ToF fallback used\n";return 2;}
 rejected.objects[0].hardware_valid=true;
 if(parking_module::parkingOuterRange(rejected)>0)return 2;
 rejected.stored_object_count=0;
 if(parking_module::parkingOuterRange(rejected)>0)return 2;
 rejected.stored_object_count=2;rejected.objects[0]={};rejected.objects[1]={};
 rejected.objects[0].distance_mm=60;rejected.objects[1].distance_mm=260;
 if(parking_module::parkingOuterRange(rejected)!=60)return 2;
 rejected.objects[0].filter_accepted=false;
 if(parking_module::parkingOuterRange(rejected)!=260)return 2;
 std::cout<<"QUALITY 5 FAILED 0\n";
 int id,kind,dir,layout,fault;float dx,dy,dh,bx,by,bh,gap;unsigned long period;
 const int configs[11][3]={{0,0,0},{1,0,0},{2,0,0},{0,1,0},{0,2,0},{0,0,1},{0,0,2},{1,0,1},{1,0,2},{2,0,1},{2,0,2}};
 while(std::cin>>id>>kind>>dir>>layout>>dx>>dy>>dh>>bx>>by>>bh>>gap>>period>>fault){
  routeTurnSign=dir;resetState();baseline();
  for(auto &s:seats)s.confirmed=s.injected=false;
  for(int st=0;st<3;++st){int i=2*st+(dir>0),c=configs[layout][st];
   seats[i].confirmed=seats[i].injected=c;seats[i].red=c==2;}
  learned();joinSameColourStraightEnds(livePath,true);roundKnownCornerPairs(livePath,true);
  if(!buildOptimizedPath())return 3;
  int index=3;
  for(int i=0;i<pathLength;++i)if(baselinePath[i].distanceMm>=corners[3].pathEndMm&&
      finalCornerVehicleClear(PositionEstimate{optimizedPath[i].x,optimizedPath[i].y,connectorRouteHeading(optimizedPath,i)})){index=i;break;}
  truePose={optimizedPath[index].x+dx,optimizedPath[index].y+dy,
      connectorRouteHeading(optimizedPath,index)+dh};
  if(kind==1)truePose={dx,dy,dh};
  hostPose=truePose;hostPose.x_mm+=bx;hostPose.y_mm+=by;hostPose.heading_deg+=bh;
  physicalGap=gap;framePeriod=period;sensorFault=fault;
  parking_module::final_parking_reset();hostTime=0;encoder=0;hostGyroHealthy=true;
  stop(true);robot_logger.lines.str("");robot_logger.lines.clear();robot_logger.writes=0;
  bool initialSafe=physicalSafe(truePose,0),collision=false,moved=false,frozen=false;
  int peakState=0;
  for(int step=0;step<20000&&!parking_module::final_parking_complete()&&!parking_module::final_parking_aborted();++step){
   parking_module::final_parking_update(dir);
   peakState=std::max(peakState,static_cast<int>(parking_module::state));
   if(dc_state==DC_ENABLED){
    moved=true;frozen|=servo_disabled&&steeringCommand!=0;
    // 1 mm substeps detect physical contacts between 20 ms control updates.
    float remaining=fabsf(command*.020f),sign=command<0?-1.f:1.f;
    while(remaining>.0001f){float ds=sign*fminf(1,remaining);remaining-=fabsf(ds);encoder+=ds;
     parking_module::advanceParkingPose(hostPose,servo_disabled?0:steeringCommand,ds);
     parking_module::advanceParkingPose(truePose,servo_disabled?0:steeringCommand,ds);
     collision|=!physicalSafe(truePose,servo_disabled?0:steeringCommand);
    }
   }
   hostTime+=20;
  }
  bool parked=parking_module::final_parking_complete();
  bool contained=parked&&physicallyContained();
  bool rejected=parking_module::final_parking_aborted()&&dc_state!=DC_ENABLED;
  // Unsafe initial configurations must be rejected before any motion.
  bool safe=initialSafe ? !collision&&!frozen&&((parked&&contained)||rejected)
                        : !moved&&!parked&&rejected;
  std::string reason=robot_logger.lines.str();auto pos=reason.rfind("[FINAL PARK ABORT] ");
  reason=pos==std::string::npos?"none":reason.substr(pos+19);
  reason=reason.substr(0,reason.find('\n'));
  std::cout<<"RESULT "<<id<<" "<<safe<<" "<<parked<<" "<<rejected<<" "<<collision
   <<" "<<contained<<" "<<initialSafe<<" "<<moved<<" "<<peakState<<" "<<hostTime<<" "<<reason<<"\n";
 }return 0;
}
'''
    cpp, exe = dest/'check.cpp', dest/'check.exe'
    cpp.write_text(fixture)
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'include'),
                    '-I', str(ROOT/'local_workspace/later-laps'), str(cpp), '-o', str(exe)], check=True)
    cases = []

    def add(group, kind=0, direction=-1, layout=0, offset=(0,0,0), bias=(0,0,0),
            gap=247.5, period=30, fault=0):
        cases.append(dict(group=group, kind=kind, direction=direction, layout=layout,
                          offset=offset, bias=bias, gap=gap, period=period, fault=fault))

    for direction in (-1,1):
        for layout in range(11):
            for gap in (242.5,247.5,252.5):
                for dx,dy in ((0,0),(-40,0),(40,0),(0,-40),(0,40)):
                    for dh in (-10,0,10):
                        add('known_arrival', direction=direction, layout=layout, offset=(dx,dy,dh), gap=gap)
            for bias in ((-40,0,0),(40,0,0),(0,-20,0),(0,20,0),(0,0,-3),(0,0,3)):
                add('unknown_pose_error', direction=direction, layout=layout, bias=bias)
        for x in (-300,-100,100,500,750):
            for y in (-800,-1000,-1205.4):
                for heading in range(0,360,30):
                    add('absolute_start', kind=1, direction=direction, offset=(x,y,heading))
        for x,y in ((1000,1000),(-1000,1000),(0,1000),(1000,0),(-1000,0)):
            for heading in (0,90,180,270):
                add('other_field_section', kind=1, direction=direction, offset=(x,y,heading))
        for fault in range(1,9):
            add('sensor_fault', direction=direction, fault=fault)
        add('wrong_gap', direction=direction, gap=280)
    inputs=[' '.join(map(str,[i,c['kind'],c['direction'],c['layout'],
        *c['offset'],*c['bias'],c['gap'],c['period'],c['fault']])) for i,c in enumerate(cases)]

    def run_shard(number):
        input_path,output_path=dest/f'input_{number}.txt',dest/f'output_{number}.txt'
        input_path.write_text('\n'.join(inputs[number::args.workers])+'\n')
        # Persist output during the run, including if a machine is too slow
        # for the timeout. Each child receives independent complete cases.
        with input_path.open() as stdin,output_path.open('w') as stdout:
            run=subprocess.run([str(exe)],stdin=stdin,stdout=stdout,
                stderr=subprocess.STDOUT,timeout=1200)
        return run.returncode,output_path.read_text()

    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        results=list(pool.map(run_shard,range(args.workers)))
    output=''.join(out for _,out in results)
    (dest/'output.txt').write_text(output)
    print('\n'.join(line for line in results[0][1].splitlines() if line.startswith(('BOUNDARY','QUALITY'))))
    if any(code for code,_ in results):
        raise SystemExit(2)
    for line in output.splitlines():
        if not line.startswith('RESULT '):
            continue
        fields=line.split()
        index=int(fields[1])
        for key,value in zip(('safe','parked','rejected','collision','contained','initial_safe','moved'),fields[2:9]):
            cases[index][key]=bool(int(value))
        cases[index].update(last_state=int(fields[9]), elapsed_ms=int(fields[10]), reason=fields[11])
    assert all('safe' in c for c in cases)
    (dest/'report.json').write_text(json.dumps(cases,indent=2)+'\n')
    for group in dict.fromkeys(c['group'] for c in cases):
        subset=[c for c in cases if c['group']==group]
        print(group,len(subset),'parked',sum(c['parked'] for c in subset),
              'rejected',sum(c['rejected'] for c in subset),'unsafe',sum(not c['safe'] for c in subset))
    unsafe=[c for c in cases if not c['safe']]
    for c in unsafe[:8]:
        print('UNSAFE',c)
    raise SystemExit(1 if unsafe else 0)


if __name__=='__main__':
    main()
