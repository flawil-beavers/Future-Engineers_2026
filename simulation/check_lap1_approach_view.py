"""Check actual lap1 aiming and replay archived view/avoidance geometry.

No image acceptance or hardware motion. A replay checks the current controller
at old estimated poses; it does not recreate the old camera or complete drive.
"""
import json
import re
import shutil
import subprocess
from pathlib import Path

from analyze_lap1_last20 import analyze, EVIDENCE
from check_connector_servo_resume import block
from check_later_laps import later_lap_fixture

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'local_workspace/lap1-last20'


def fixture_source():
    source = (ROOT / 'src/obstacle_path.cpp').read_text()
    fixture = later_lap_fixture().split('int main()')[0]
    fixture = fixture.replace('anchor.heading_deg=180;',
                              'anchor.heading_deg=routeTurnSign<0?180:0;')
    fixture = fixture.replace('struct Discovery {bool observedClear=false;};',
                              'struct Discovery {bool observedClear=false;bool seatObservedClear[2]={};};')
    fixture += r'''
#include "obstacle_blob_validation.h"
using DiscoveryStation=Discovery;
Blob raw{};const Blob *getLargestObstacle(){return &raw;}
bool obstacle_blob_valid_for_acquisition(const Blob *b){return obstacle_blob_shape_valid(b);}
float obstacle_camera_bearing_deg(const Blob *b){return atanf((164.4f-b->centerX)/248.9f)*180/PI;}
struct Observation{bool productionValid=false;ColorType color=ColorType::NONE;
 int seatId=-1;float rangeMm=0,bearingDeg=0;}lastDiscoveryObservation;
int discoveryScanStation=-1,discoveryScanSide=-1,lastConfirmedSeatIndex=-1;
bool extremeAdjacentReleasePending=false;
float lastDiscoveryTargetNudgeDeg=0;unsigned long clockMs=1000,lastDiscoveryNudgeUpdateMs=0;
unsigned long millis(){return clockMs;}
bool discoveryAimCommandSafe(const PositionEstimate&,int);
'''
    for signature in ('bool completingExtremeAdjacentPair(', 'bool traversingInjectedAvoidance(',
                      'bool cornerViewSeatMayBeOccupied(', 'bool cornerViewRetreatSafe(',
                      'bool cornerViewSweepSafe(', 'bool discoveryAimCommandSafe(',
                      'void seatCameraGeometry(', 'void applyDiscoveryTargetNudge(',
                      'bool earlyDiscoveryAimCommandSafe(',
                      'void applyLap1DiscoveryAim('):
        fixture += block(source, source.rindex(signature)) + '\n'
    return fixture


def main():
    fixture = fixture_source()
    fixture += r'''
void clean(int dir){routeTurnSign=dir;resetState();baseline();
 for(auto &s:seats)s.confirmed=s.injected=false;
 for(auto &d:discoveryStations){d.observedClear=true;d.seatObservedClear[0]=d.seatObservedClear[1]=true;}
 lastDiscoveryTargetNudgeDeg=0;lastDiscoveryNudgeUpdateMs=clockMs;
 raw={};lastDiscoveryObservation={};}
int main(){int tests=0;
 for(int dir:{-1,1})for(float distance:{850.f,800.f,700.f,650.f,600.f})
 for(float lateral:{-160.f,0.f,160.f})for(float angle:{-10.f,0.f,10.f})for(int mode:{0,1,2}){
  clean(dir);const int station=4;auto &d=discoveryStations[station];d.observedClear=false;
  d.seatObservedClear[0]=mode==2;d.seatObservedClear[1]=mode==1;
  const auto center=interpolateBaseline(seats[8].pathDistanceMm);
  const float h=center.headingDeg*PI/180;
  PositionEstimate p{center.x-distance*cosf(h)-lateral*sinf(h),
    center.y-distance*sinf(h)+lateral*cosf(h),center.headingDeg+angle};
  progressIndex=nearestPathIndex(baselinePath,p.x_mm,p.y_mm,0,pathLength);
  float a,b,range;seatCameraGeometry(8,p,a,range);seatCameraGeometry(9,p,b,range);
  float aim=mode==1?a:mode==2?b:wrap180(a+.5f*wrap180(b-a));
  float allowed=mode?OBSTACLE_LOOK_SINGLE_SEAT_BEARING_DEG:fmaxf(0,
    OBSTACLE_CAMERA_HORIZONTAL_FOV_DEG*OBSTACLE_DISCOVERY_FOV_FRACTION-
    OBSTACLE_LOOK_FOV_MARGIN_DEG-.5f*fabsf(wrap180(b-a)));
  PathPoint target{};target.x=p.x_mm+250*cosf(p.heading_deg*PI/180);
  target.y=p.y_mm+250*sinf(p.heading_deg*PI/180);
  for(int frame=0;frame<8;++frame){clockMs+=50;auto t=target;applyDiscoveryTargetNudge(t,p);}
  const float forward=cyclicDistanceForward(baselinePath[progressIndex].distanceMm,seats[8].pathDistanceMm);
  if(forward<840 && fabsf(aim)>allowed+.1f &&
     (discoveryScanStation!=station || lastDiscoveryTargetNudgeDeg*aim<=0))return 1;
  ++tests;
 }
 std::cout<<"POLICY "<<tests<<" early distances/directions/row states PASS\n";
 int id,dir,station,mask[24],clear[24];float x,y,h,expected0,expected1;
 while(std::cin>>id>>dir>>station>>x>>y>>h>>expected0>>expected1){
  clean(dir);for(int &v:mask)std::cin>>v;for(int &v:clear)std::cin>>v;
  for(int i=0;i<24;++i){seats[i].confirmed=seats[i].injected=mask[i]!=0;seats[i].red=mask[i]==2;
   discoveryStations[i/2].seatObservedClear[i%2]=clear[i];}
  for(int i=0;i<12;++i)discoveryStations[i].observedClear=clear[2*i]&&clear[2*i+1];
  learned();PositionEstimate p{x,y,h};
  progressIndex=nearestPathIndex(livePath,x,y,0,pathLength);
  float a,b,ra,rb;seatCameraGeometry(station*2,p,a,ra);seatCameraGeometry(station*2+1,p,b,rb);
  if(fabsf(wrap180(a-expected0))>.15f || fabsf(wrap180(b-expected1))>.15f)return 2;
  bool protectedPass=traversingInjectedAvoidance(baselinePath[progressIndex].distanceMm);
  auto target=findLookahead(livePath,p,firstLapLookahead(100));clockMs+=50;
  applyDiscoveryTargetNudge(target,p);
  std::cout<<"POSE "<<id<<" "<<protectedPass<<" "<<discoveryScanStation<<" "
   <<lastDiscoveryTargetNudgeDeg<<" "<<a<<" "<<b<<" "<<ra<<" "<<rb<<"\n";
 }
}
'''
    index = (EVIDENCE / 'README.md').read_text(encoding='utf-8')
    paths = sorted((p for p in EVIDENCE.glob('*.txt') if re.search(r'log_(\d+)', p.name)),
                   key=lambda p: int(re.search(r'log_(\d+)', p.name)[1]))[-20:]
    samples, inputs = [], []
    for path in paths:
        report = analyze(path, index)
        by_line = {r['line']: r for r in report['traces']}
        mask, clear = [0]*24, [0]*24
        for number, line in enumerate(path.read_text(errors='replace').splitlines(), 1):
            if '[PATH] Live avoidance injected seat=' in line:
                seat = int(re.search(r'seat=(\d+)', line)[1])
                mask[seat] = 2 if 'color=RED' in line else 1
            if '[SEAT CLEAR]' in line:
                clear[int(re.search(r'seat=(\d+)', line)[1])] = 1
            row = by_line.get(number)
            if not row:
                continue
            sid = len(samples)
            samples.append(dict(file=path.name, line=number, trace=row))
            inputs.append(' '.join(map(str, [sid, 1 if '_ccw' in path.name else -1,
                row['station'], *row['pose'], row['seats'][0][0], row['seats'][1][0], *mask, *clear])))
    OUT.mkdir(parents=True, exist_ok=True)
    cpp, exe = OUT/'approach.cpp', OUT/'approach.exe'
    cpp.write_text(fixture)
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'include'),
                    '-I', str(ROOT/'simulation/host'), str(cpp), '-o', str(exe)], check=True)
    run = subprocess.run([str(exe)], input='\n'.join(inputs)+'\n', text=True, capture_output=True)
    (OUT/'approach_output.txt').write_text(run.stdout+run.stderr)
    for line in run.stdout.splitlines():
        if not line.startswith('POSE '):
            print(line)
            continue
        _, sid, protected, target, nudge, a, b, ra, rb = line.split()
        samples[int(sid)]['current_replay'] = dict(protected_pass=bool(int(protected)),
            aim_station=int(target), first_tick_nudge=float(nudge),
            bearings=list(map(float, (a, b))), ranges=list(map(float, (ra, rb))))
    (OUT/'approach_report.json').write_text(json.dumps(samples, indent=2)+'\n')
    print('ARCHIVED GEOMETRY', sum('current_replay' in r for r in samples), '/', len(samples),
          'replayed; result', run.returncode)
    raise SystemExit(run.returncode)


if __name__ == '__main__':
    main()
