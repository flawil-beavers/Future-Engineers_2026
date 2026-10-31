"""Production buffer + corrected-pose acquisition; no powered camera claim.

Uses the real ring buffer and source-extracted production association/import.
Only hardware and final map/route dispatch are stubbed. Negative cases must
leave the existing scan active and the live estimator untouched.
"""
import shutil
import subprocess
import json
import re

from check_connector_servo_resume import block
from check_far_obstacle_acquisition import fixture_source
from check_cw_start_planner import ROOT


def fixture():
    source = (ROOT / 'src/obstacle_path.cpp').read_text()
    code = fixture_source()
    code = code.replace('bool mappedParkingSeatFootMatches(uint8_t,const PositionEstimate &,int,bool){return true;}', '')
    code = code.replace('using std::isfinite;', '''using std::isfinite;
#include "parking_exit_observer.h"
bool running=true,parkingEntryTestHold=false;
''')
    marker = 'unsigned long millis(){return clockMs;}'
    code = code.replace(marker, marker + '\nunsigned long micros(){return clockMs*1000U;}')
    funcs = '\n'.join(block(source, source.index(sig)) for sig in (
        'float wrap180(', 'void seatCameraGeometry(', 'bool mappedParkingSeatFootMatches('))
    code = code.replace('int nearestSeatIndex(', funcs + '\nint nearestSeatIndex(', 1)
    code += '\n' + '\n'.join(block(source, source.index(sig)) for sig in (
        'int parkingExitObservationSeat(', 'bool parkingExitObservationStable(',
        'void obstacle_path_apply_exit_observations('))
    return code + '\n#include "parking_exit_camera_buffer.cpp"\n'


def main():
    code = fixture() + r'''
int checks=0;
#define CHECK(c) do { ++checks; if(!(c)){std::cerr<<"FAIL line "<<__LINE__<<" check "<<checks<<"\n";return 1;} }while(0)
void setup(int direction){
 for(auto &s:seats){s={};s.x=s.y=9000;}
 routeTurnSign=direction;parkingSectionInnerSeatsOnly=true;
 parkingEntryActive=true;parkingEntryObserving=parkingEntryScouting=false;
 running=true;runtimeTestMode=false;completedLaps=0;parkingEntryTestHold=false;
 sectionLayoutMode=OBSTACLE_SECTION_LAYOUT_OFFICIAL;injectionCount=0;
 for(int station=0;station<3;++station)for(int side=0;side<2;++side){
  auto &s=seats[2*station+side];s.x=direction*(station-1)*500;
  s.y=side==0?-1100:-900;s.headingDeg=direction==1?0:180;
 }
 hostPose={1234,5678,42,0};clockMs=4000;
}
PositionEstimate facing(int seat,float range,float bearing=0){
 const float h=routeTurnSign==1?60:120;
 const float ray=(h+bearing)*PI/180;
 return {seats[seat].x-range*cosf(ray)-OBSTACLE_CAMERA_LOCAL_X_MM*cosf(h*PI/180),
         seats[seat].y-range*sinf(ray)-OBSTACLE_CAMERA_LOCAL_X_MM*sinf(h*PI/180),h,0};
}
Blob blobFor(int seat,const PositionEstimate &pose,ColorType color){
 float bearing,range;seatCameraGeometry(seat,pose,bearing,range);
 int x=lroundf(OBSTACLE_CAMERA_PRINCIPAL_X_PX-OBSTACLE_CAMERA_FOCAL_X_PX*tanf(bearing*PI/180));
 int foot=lroundf(OBSTACLE_CAMERA_GROUND_HORIZON_Y+
  OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX/(range*cosf(bearing*PI/180))+
  OBSTACLE_CAMERA_FOOT_EDGE_SLOPE*fabsf(x-OBSTACLE_CAMERA_PRINCIPAL_X_PX));
 Blob b{};b.found=true;b.color=color;b.minX=x-9;b.maxX=x+9;
 b.minY=60;b.maxY=foot;b.centerX=x;b.centerY=(60+foot)/2;b.area=700;return b;
}
void feed(ParkingExitCameraBuffer &b,uint32_t base,const PositionEstimate &pose,
          const Blob &blob,int frames=3,uint32_t gap=80000){
 VisionResult v{};if(blob.color==ColorType::RED)v.redPillar=blob;else v.greenPillar=blob;
 for(int i=0;i<frames;++i){
  const uint32_t end=base+(i+1)*gap;
  for(uint32_t t=end-80000;t<=end;t+=10000)b.samplePose(t,pose);
  b.capture(end+1000,end,80000,1+end/80000,v);
 }
}
int main(int argc,char **){
 if(argc>1){int direction,station;float x,y,h;
  while(std::cin>>direction>>station>>x>>y>>h){setup(direction);int seat=station*2+1;
   PositionEstimate p{x,y,h,0};float bearing,range;seatCameraGeometry(seat,p,bearing,range);
   ObstacleObservationResult observation;bool accepted=false;
   if(fabsf(bearing)<35 && range>100 && range<1100){
    auto blob=blobFor(seat,p,ColorType::GREEN);
    accepted=parkingExitObservationStable(blob,p,observation);}
   std::cout<<accepted<<" "<<bearing<<" "<<range<<"\n";
  }return 0;
 }
 // All three start places, both directions/colours, centre and off-axis views,
 // two accumulated XY corrections and 350/500/700mm distances.
 for(int direction:{-1,1})for(int station:{0,1,2})for(auto color:{ColorType::RED,ColorType::GREEN})
 for(float range:{350.f,500.f,700.f})for(float bearing:{-18.f,0.f,18.f}){
  setup(direction);int seat=2*station+1;auto p=facing(seat,range,bearing);
  Blob blob=blobFor(seat,p,color);ParkingExitCameraBuffer b;b.begin(direction);
  PositionEstimate raw=p;raw.x_mm-=30;raw.y_mm+=18;
  feed(b,1000000,raw,blob);CHECK(b.size()==3);
  b.translate(12,-8);b.translate(18,-10);
  CHECK(fabsf(b.frame(0).pose.x_mm-p.x_mm)<.01f);
  obstacle_path_apply_exit_observations(b);
  if(!seats[seat].confirmed){ObstacleObservationResult debug;
   std::cerr<<"case "<<direction<<"/"<<station<<"/"<<int(color)<<" range/bearing="<<range<<"/"<<bearing
    <<" nominal="<<parkingExitObservationSeat(blob,p,debug)<<" stable="<<parkingExitObservationStable(blob,p,debug)
    <<" snap="<<debug.snapErrorMm<<"\n";}
  CHECK(seats[seat].confirmed && seats[seat].red==(color==ColorType::RED));
  CHECK(parkingEntryActive && hostPose.x_mm==1234 && hostPose.heading_deg==42);
 }
 for(int direction:{-1,1}){
  setup(direction);auto p=facing(5,450);auto green=blobFor(5,p,ColorType::GREEN);
  // Insufficient/stale/non-consecutive/invalid-frame evidence never confirms.
  for(int mode=0;mode<8;++mode){
   setup(direction);ParkingExitCameraBuffer b;b.begin(direction);
   if(mode==0)feed(b,1000000,p,green,1);
   if(mode==1)feed(b,1000000,p,green,2);
   if(mode==2){feed(b,1000000,p,green);clockMs=25000;}
   if(mode==3)feed(b,1000000,p,green,3,600000);
   if(mode==4){feed(b,1000000,p,green);b.reset();}
   if(mode==5){feed(b,1000000,p,green);completedLaps=1;}
   if(mode==6){feed(b,1000000,p,green);routeTurnSign=-direction;}
   if(mode==7){feed(b,1000000,p,green);parkingEntryTestHold=true;}
   obstacle_path_apply_exit_observations(b);CHECK(!seats[5].confirmed && !injectionCount);
  }
  // A contradictory later colour cancels the earlier three-frame consensus.
  setup(direction);ParkingExitCameraBuffer conflict;conflict.begin(direction);
  feed(conflict,1000000,p,green);feed(conflict,1320000,p,blobFor(5,p,ColorType::RED));
  obstacle_path_apply_exit_observations(conflict);CHECK(!seats[5].confirmed);
  // Empty images cannot produce cached CLEAR or a colour vote.
  setup(direction);ParkingExitCameraBuffer empty;empty.begin(direction);
  feed(empty,1000000,p,Blob{});CHECK(empty.size()==0);
  obstacle_path_apply_exit_observations(empty);CHECK(injectionCount==0 && parkingEntryActive);
  // Opposite row (physical ambiguity in CHECK_ALL) is rejected as a batch.
  setup(direction);sectionLayoutMode=OBSTACLE_SECTION_LAYOUT_CHECK_ALL;
  ParkingExitCameraBuffer rows;rows.begin(direction);feed(rows,1000000,p,green);
  auto outer=facing(4,450);feed(rows,1320000,outer,blobFor(4,outer,ColorType::GREEN));
  obstacle_path_apply_exit_observations(rows);CHECK(!seats[4].confirmed && !seats[5].confirmed);
  // Middle plus end cannot be resolved by import ordering in official layouts.
  setup(direction);ParkingExitCameraBuffer layout;layout.begin(direction);feed(layout,1000000,p,green);
  auto middle=facing(3,450);feed(layout,1320000,middle,blobFor(3,middle,ColorType::GREEN));
  obstacle_path_apply_exit_observations(layout);CHECK(!seats[3].confirmed && !seats[5].confirmed);
 }
 // Mid-frame interpolation, angle wrapping and correction applied to history.
 setup(1);ParkingExitCameraBuffer timed;timed.begin(1);
 VisionResult v{};auto p=facing(5,450);v.greenPillar=blobFor(5,p,ColorType::GREEN);
 for(uint32_t t=1000000;t<=1080000;t+=10000){auto q=p;q.x_mm+=(t-1000000)*.0001f;
  q.heading_deg=358+(t-1000000)*.00005f;timed.samplePose(t,q);}
 CHECK(timed.capture(1081000,1080000,80000,1,v));
 CHECK(fabsf(timed.frame(0).pose.x_mm-p.x_mm-4)<.01f && fabsf(timed.frame(0).pose.heading_deg)<.01f);
 CHECK(!timed.capture(1081000,1080000,80000,1,v) && timed.size()==1);
 timed.translate(10,20);CHECK(fabsf(timed.frame(0).pose.y_mm-p.y_mm-20)<.01f);
 CHECK(!timed.capture(1300000,1080000,80000,2,v)); // stale completion
 CHECK(!timed.capture(1081000,1080000,200000,3,v)); // missing sensor frame
 CHECK(!timed.capture(1081000,1080000,80000,1,v) && timed.size()==0); // reset epoch
 // Abrupt motion, non-finite poses and history gaps cannot be interpolated.
 for(int kind=0;kind<3;++kind){ParkingExitCameraBuffer b;b.begin(1);
  for(uint32_t t=1000000;t<=1080000;t+=10000){auto q=p;
   if(kind==0)q.heading_deg+=(t-1000000)*.001f;
   if(kind==1)q.x_mm+=(t-1000000)*.001f;
   if(kind==2 && t>1000000 && t<1080000)continue;
   b.samplePose(t,q);}
  CHECK(!b.capture(1081000,1080000,80000,1,v));}
 timed.begin(1);auto invalid=p;invalid.x_mm=NAN;timed.samplePose(1000000,invalid);
 CHECK(!timed.active());
 // Bounded ring preserves chronological order after saturation.
 ParkingExitCameraBuffer full;full.begin(1);feed(full,1000000,p,v.greenPillar,180);
 CHECK(full.size()==ParkingExitCameraBuffer::CAPACITY && full.overwritten()==20);
 for(int i=1;i<full.size();++i)CHECK(full.frame(i).sampledUs>full.frame(i-1).sampledUs);
 // micros wrap still brackets frames and preserves sequence distinction.
 ParkingExitCameraBuffer wrap;wrap.begin(1);uint32_t base=0xffff0000U;
 for(uint32_t dt=0;dt<=80000;dt+=10000)wrap.samplePose(base+dt,p);
 CHECK(wrap.capture(base+81000,base+80000,80000,1,v));
 std::cout<<"EXIT OBSERVER "<<checks<<" checks PASS; buffer bytes="<<sizeof(ParkingExitCameraBuffer)<<"\n";
}
'''
    dest = ROOT / 'local_workspace/parking-exit-observer'
    dest.mkdir(parents=True, exist_ok=True)
    cpp, exe = dest / 'check.cpp', dest / 'check.exe'
    cpp.write_text(code)
    build = subprocess.run([shutil.which('g++'), '-std=c++17', '-O2',
        '-I', str(ROOT/'simulation/host'), '-I', str(ROOT/'include'),
        '-I', str(ROOT/'src'), str(cpp), '-o', str(exe)], capture_output=True, text=True)
    (dest/'build.txt').write_text(build.stdout + build.stderr)
    if build.returncode:
        print(build.stderr[:6000])
        raise SystemExit(build.returncode)
    run = subprocess.run([str(exe)], capture_output=True, text=True)
    (dest/'output.txt').write_text(run.stdout + run.stderr)
    print(run.stdout + run.stderr, end='')
    if run.returncode == 0:
        review_archived_views(exe, dest)
        verify_handover(dest)
    raise SystemExit(run.returncode)


def review_archived_views(exe, dest):
    """Possible view windows from archived odometry, not observed RGB frames."""
    cases = []
    for number in range(574, 594):
        paths = list((ROOT/'simulation/evidence/parking_exit_diagnostics').glob(f'*_log_{number}_*.txt'))
        if len(paths) != 1:
            raise RuntimeError(f'Expected indexed evidence for log {number}')
        text = paths[0].read_text()
        corrections = []
        for line in text.splitlines():
            m = re.search(r'type=correction t=(\d+).* delta=([-\d.]+),([-\d.]+),([-\d.]+)', line)
            if m:
                corrections.append(tuple(map(float, m.groups())))
        for line in text.splitlines():
            m = re.search(r'type=sample t=(\d+) state=(\w+) seg=(\d+) turn=(-?\d+).* pose=([-\d.]+),([-\d.]+),([-\d.]+)', line)
            if not m:
                continue
            t, state, segment, direction, x, y, heading = m.groups()
            t, segment, direction = int(t), int(segment), int(direction)
            x, y, heading = float(x), float(y), float(heading)
            for when, dx, dy, dh in corrections:
                if when >= t:
                    if abs(dh) > .01:
                        raise RuntimeError('XY-only replay cannot transform a heading correction')
                    x += dx
                    y += dy
            for station in range(3):
                cases.append(dict(log=number, evidence=paths[0].name, t=t, state=state,
                    segment=segment, direction=direction, station=station, pose=[x,y,heading]))
    data = ''.join(f"{c['direction']} {c['station']} {' '.join(map(str,c['pose']))}\n" for c in cases)
    result = subprocess.run([str(exe), '--geometry'], input=data, text=True, capture_output=True, check=True)
    lines = result.stdout.splitlines()
    assert len(lines) == len(cases)
    for case, line in zip(cases, lines):
        accepted, bearing, distance = line.split()
        case.update(geometrically_eligible=bool(int(accepted)), bearing=float(bearing), range_mm=float(distance))
    summary = []
    for number in range(574, 594):
        for station in range(3):
            group = [c for c in cases if c['log']==number and c['station']==station]
            if not group:
                continue
            eligible = list({c['t']: c for c in group if c['geometrically_eligible']}.values())
            summary.append(dict(log=number, direction=group[0]['direction'], station=station,
                samples=len(eligible), segments=sorted({c['segment'] for c in eligible}),
                first_t=min((c['t'] for c in eligible), default=None),
                last_t=max((c['t'] for c in eligible), default=None)))
    (dest/'archived_view_windows.json').write_text(json.dumps(dict(summary=summary, cases=cases,
        limitations=['Synthetic ideal green blob at corrected archived odometry.',
                     'No robot images during exit: no colour/detection or occlusion acceptance.']), indent=2)+'\n')
    for direction, station in [(1,2),(-1,1),(-1,0),(-1,2)]:
        group = [s for s in summary if s['direction']==direction and s['station']==station]
        print(f"VIEW {'CCW' if direction>0 else 'CW'} station={station}: "
              f"{sum(s['samples']>=3 for s in group)}/{len(group)} logs have >=3 eligible sampled poses")


def verify_handover(dest):
    source = (ROOT/'src/obstacle.cpp').read_text()
    start = source.index('    if (!obstacle_path_started())', source.index('void obstacle_challenge_update('))
    end = source.index('    // Nonblocking settling', start)
    transition = source[start:end]
    cpp = dest/'handover.cpp'
    cpp.write_text(r'''
#include <cstdint>
#include <cmath>
#include <iostream>
constexpr int A0=0,A1=1,A2=2;
constexpr float PI=3.14159265358979323846f;
#include "config.h"
bool started=false,complete=false,seenFrame=false,oc_parking_localization_first_edge_reference=false;
int calls=0,imports=0,updates=0,lastTurn=0,oc_parking_exit_steering=50;
bool obstacle_path_started(){return started;}
bool obstacle_path_complete(){return complete;}
void obstacle_path_start(int turn,bool,float,int,float,bool,bool){started=true;++calls;lastTurn=turn;}
void parking_exit_observer_apply(){if(!started)std::abort();++imports;}
void obstacle_path_update(bool frame){++updates;seenFrame=frame;}
void transition(bool newCameraFrame){
''' + transition + r'''
}
int main(){int checks=0;
 for(int turn:{-1,1})for(bool frame:{false,true}){
  started=complete=false;calls=imports=updates=0;oc_parking_exit_steering=-turn*50;
  transition(frame);++checks;
  if(calls!=1||imports!=1||updates!=1||seenFrame||lastTurn!=turn)return 1;
  transition(true);++checks;
  if(calls!=1||imports!=1||updates!=2||!seenFrame)return 2;
  transition(false);++checks;
  if(calls!=1||imports!=1||updates!=3||seenFrame)return 3;
 }
 std::cout<<"EXIT HANDOVER "<<checks<<" checks PASS; no duplicate old-frame vote, normal fresh-frame resume\n";
}
''')
    exe = dest/'handover.exe'
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'include'),
                    str(cpp), '-o', str(exe)], check=True)
    run = subprocess.run([str(exe)], capture_output=True, text=True, check=True)
    (dest/'handover_output.txt').write_text(run.stdout)
    print(run.stdout, end='')


if __name__ == '__main__':
    main()
