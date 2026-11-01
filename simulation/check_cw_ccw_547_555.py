"""Source-backed regressions for the returned CW batch; no hardware access."""
import shutil
import subprocess
import re
from check_connector_servo_resume import ROOT, block
from check_later_laps import later_lap_fixture
from check_final_parking_sequence import fixture_source as parking_fixture
from check_cw_start_planner import fixture_source as start_fixture
from check_far_obstacle_acquisition import fixture_source as acquisition_fixture


def run(name, source):
    out = ROOT/'local_workspace/cw-547-550'
    out.mkdir(exist_ok=True)
    cpp, exe = out/(name+'.cpp'), out/(name+'.exe')
    cpp.write_text(source)
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2',
                    '-I', str(ROOT/'include'), '-I', str(ROOT/'simulation/host'), '-I', str(ROOT/'local_workspace/later-laps'),
                    str(cpp), '-o', str(exe)], check=True)
    r = subprocess.run([str(exe)], capture_output=True, text=True)
    (out/(name+'.txt')).write_text(r.stdout+r.stderr)
    print(r.stdout)
    if r.returncode:
        raise SystemExit(r.returncode)


def main():
    final = (ROOT/'src/final_parking.cpp').read_text()
    # Previous production implementation, retained as an independent negative/
    # equivalence control. Check identical geometry decisions across poses.
    old = subprocess.check_output(['git', 'show', 'HEAD:src/final_parking.cpp'], text=True)
    frozen = block(old, old.index('bool connectorClearanceSafe(')).replace(
        'bool connectorClearanceSafe(', 'bool frozenConnectorClearanceSafe(', 1)
    code = parking_fixture()+ '\nnamespace parking_module {\n'+frozen+'\n}\n'
    code += r'''
int main(){int total=0;
 for(int dir:{-1,1})for(int mask:{0,1,3,7}){
  routeTurnSign=dir;resetState();baseline();
  for(int i=0;i<24;++i)seats[i].confirmed=(i%4==0)||(mask&(1<<(i%3)));
  for(float x:{-1400.f,-1100.f,-600.f,0.f,350.f,500.f,700.f,1100.f,1400.f})
  for(float y:{-1450.f,-1300.f,-1200.f,-1050.f,-850.f,0.f,850.f,1300.f})
  for(float h:{0.f,30.f,90.f,145.f,180.f,220.f,270.f,330.f}){
   PositionEstimate p{x,y,h};++total;
   if(parking_module::connectorClearanceSafe(p)!=parking_module::frozenConnectorClearanceSafe(p)){
    std::cout<<"CLEARANCE MISMATCH "<<x<<" "<<y<<" "<<h<<"\n";return 1;}
  }
 }
 std::cout<<"PARKING WALL REUSE "<<total<<" equivalent decisions\n";
}
'''
    run('parking_clearance', code)

    path = (ROOT/'src/obstacle_path.cpp').read_text()
    code = later_lap_fixture().split('int main()')[0]
    code = code.replace('struct Discovery {bool observedClear=false;};',
                        'struct Discovery {bool observedClear=false;bool seatObservedClear[2]={};};')
    code += r'''
#include "obstacle_blob_validation.h"
Blob edgeRaw{};
const Blob *getLargestObstacle(){return &edgeRaw;}
bool obstacle_blob_valid_for_acquisition(const Blob *b){return obstacle_blob_shape_valid(b);}
float obstacle_camera_bearing_deg(const Blob *b){return atanf((164.4f-b->centerX)/248.9f)*180/PI;}
struct Observation {bool productionValid=false;ColorType color=ColorType::NONE;
 int seatId=-1;float rangeMm=0,bearingDeg=0;}lastDiscoveryObservation;
int discoveryScanStation=-1,discoveryScanSide=-1;
using DiscoveryStation=Discovery;
bool extremeAdjacentReleasePending=false;int lastConfirmedSeatIndex=-1;
float lastDiscoveryTargetNudgeDeg=0;unsigned long lastDiscoveryNudgeUpdateMs=0,clockMs=1000;
unsigned long millis(){return clockMs;}
bool discoveryAimCommandSafe(const PositionEstimate &,int);
'''
    for sig in ('bool completingExtremeAdjacentPair(', 'bool cornerViewSeatMayBeOccupied(',
                'bool cornerViewRetreatSafe(', 'bool cornerViewSweepSafe(', 'bool discoveryAimCommandSafe(',
                'void applyDiscoveryTargetNudge('):
        code += block(path,path.rindex(sig))+'\n'
    code += r'''
int main(){int total=0;
 for(int dir:{-1,1})for(float dx:{-10.f,0.f,10.f})for(float dh:{-1.f,0.f,1.f}){
  routeTurnSign=dir;resetState();baseline();
  for(auto &d:discoveryStations)d.observedClear=true;
  seats[19].confirmed=seats[19].injected=seats[19].red=true;
  discoveryStations[11].observedClear=false;
  PositionEstimate p{909.f+dx,274.f,288.4f+dh};
  if(dir==1){p.x_mm=-p.x_mm;p.heading_deg=180-p.heading_deg;}
  float wanted=seats[22].pathDistanceMm-650.f;
  float nearest=1e9;for(int i=0;i<pathLength;++i){float d=fabsf(baselinePath[i].distanceMm-wanted);if(d<nearest){nearest=d;progressIndex=i;}}
  const float h=p.heading_deg*PI/180;
  PathPoint target{};target.x=p.x_mm+250*cosf(h);target.y=p.y_mm+250*sinf(h);
  lastDiscoveryObservation={true,ColorType::GREEN,-1,475.f,dir<0?-24.9f:24.9f};
  lastDiscoveryTargetNudgeDeg=0;lastDiscoveryNudgeUpdateMs=clockMs;
  for(int i=0;i<8;++i){clockMs+=50;PathPoint t=target;applyDiscoveryTargetNudge(t,p);}
  if(fabsf(lastDiscoveryTargetNudgeDeg)<8){std::cout<<"EDGE AIM FAIL "<<dir<<" "<<lastDiscoveryTargetNudgeDeg<<"\n";return 1;}
  int steering=int(pursuitServoForCurvature(2*sinf(lastDiscoveryTargetNudgeDeg*PI/180)/250));
  if(!discoveryAimCommandSafe(p,steering))return 2;
  const float hinted=fabsf(lastDiscoveryTargetNudgeDeg);
  ++total;
  // Without valid edge colour evidence, retain the original minimal rule.
  lastDiscoveryObservation.productionValid=false;lastDiscoveryTargetNudgeDeg=0;
  for(int i=0;i<8;++i){clockMs+=50;PathPoint t=target;applyDiscoveryTargetNudge(t,p);}
  if(fabsf(lastDiscoveryTargetNudgeDeg)>hinted-6.f)return 3;
  // Horizontal-margin-only rejection must retain the inward aiming hint.
  // This still cannot create a map confirmation or bypass a shape rejection.
  edgeRaw={};edgeRaw.found=true;edgeRaw.color=ColorType::GREEN;
  edgeRaw.centerX=dir<0?300:28;edgeRaw.minX=edgeRaw.centerX-17;
  edgeRaw.maxX=edgeRaw.centerX+17;edgeRaw.minY=80;edgeRaw.maxY=144;edgeRaw.area=1300;
  lastDiscoveryTargetNudgeDeg=0;
  for(int i=0;i<8;++i){clockMs+=50;PathPoint t=target;applyDiscoveryTargetNudge(t,p);}
  const float cameraX=p.x_mm+125*cosf(h),cameraY=p.y_mm+125*sinf(h);
  float hintError=180;
  for(int seat:{22,23}){
   float b=wrap180(atan2f(seats[seat].y-cameraY,seats[seat].x-cameraX)*180/PI-p.heading_deg);
   hintError=fminf(hintError,fabsf(wrap180(b-obstacle_camera_bearing_deg(&edgeRaw))));
  }
  if(hintError<=6.f && fabsf(lastDiscoveryTargetNudgeDeg)<hinted-1.f)return 5;
  if(hintError>6.f && fabsf(lastDiscoveryTargetNudgeDeg)>hinted-6.f)return 7;
  edgeRaw.minY=120;lastDiscoveryTargetNudgeDeg=0;
  for(int i=0;i<8;++i){clockMs+=50;PathPoint t=target;applyDiscoveryTargetNudge(t,p);}
  if(fabsf(lastDiscoveryTargetNudgeDeg)>hinted-6.f)return 6;
  edgeRaw={};
 }
 PositionEstimate wall{1480,0,0};if(discoveryAimCommandSafe(wall,0))return 4;
 std::cout<<"EDGE AIM "<<total<<" mirrored perturbed poses PASS; no evidence/wall guards retained\n";
}
'''
    run('edge_aim', code)

    code = start_fixture().split('int main()')[0]
    code = code.replace('int routeTurnSign=-1,parkingEntryTargetStation=1;',
                        'int routeTurnSign=1,parkingEntryTargetStation=2;')
    code = code.replace('bool parkingCwShortStart=true,','bool parkingCwShortStart=false,')
    code = code.replace('bool parkingCcwShortStart=false;','bool parkingCcwShortStart=true;')
    code = code.replace('anchor.heading_deg=180;','anchor.heading_deg=0;')
    reached = path[path.index('    const bool pathReached ='):path.index('    const bool travelLimitReached =')]
    code += '\nbool actualScanCanStop(float entryTravel,float plannedTravel,float finishError,float finishHeadingError){\n'+reached+'return pathReached;\n}\n'
    code += r'''
int main(){int total=0;
 const float starts[][3]={{322.1f,-1204.7f,359.f},{328.2f,-1205.5f,359.6f},{346.7f,-1220.5f,1.f}};
 for(const auto &a:starts)for(float gain:{.82f,.9f,1.f})for(float dx:{-5.f,0.f,5.f})for(float dy:{-5.f,0.f,5.f}){
  baseline();memcpy(livePath,baselinePath,sizeof(PathPoint)*pathLength);
  parkingCcwFirstEdgeStart=true;PositionEstimate p{a[0]+dx,a[1]+dy,a[2]};
  buildParkingEntryPath(p);float arc=parkingEntryScanArcMm();
  if(!preflightCwStartArc(p,arc)){std::cout<<"NO SAFE SCAN "<<p.x_mm<<" arc "<<arc<<"\n";return 1;}
  const float h=p.heading_deg*PI/180,k=gain/OBSTACLE_PARKING_ENTRY_SCAN_RADIUS_MM;
  float travel=arc-3.f,next=h+k*travel;
  p.x_mm-=(sinf(next)-sinf(h))/k;p.y_mm+=(cosf(next)-cosf(h))/k;p.heading_deg=next*180/PI;
  const auto &end=parkingEntryPath[parkingEntryLength-1];
  if(!actualScanCanStop(travel,arc,hypotf(p.x_mm-end.x,p.y_mm-end.y),fabsf(wrap180(p.heading_deg-end.headingDeg))))return 3;
  seats[5].confirmed=seats[5].injected=true;seats[5].red=false;
  displaceForSeat(livePath,5,260,true);
  uint16_t merge=0;bool ok=buildParkingEntryConnector(p,livePath,merge);++total;
  if(!ok)std::cout<<"SCAN JOIN FAIL "<<a[0]<<" gain "<<gain<<" arc "<<arc<<" pose "<<p.x_mm<<","<<p.y_mm<<","<<p.heading_deg<<"\n";
  if(!ok)return 2;
 }
 std::cout<<"CCW SCAN JOIN "<<total<<" PASS (assumed yaw gain, original measured start poses)\n";
}
'''
    run('ccw_scan',code)

    code=acquisition_fixture()+r'''
int main(){
 for(auto &s:seats){s={};s.x=s.y=9000;}
 seats[6].x=seats[8].x=seats[10].x=1100;
 seats[7].x=seats[9].x=seats[11].x=900;
 seats[6].y=seats[7].y=-500;seats[8].y=seats[9].y=0;seats[10].y=seats[11].y=500;
 seats[7].confirmed=true;seats[7].red=false;
 hostPose={390,-682,35.6f};parkingEntryConnectorActive=true;
 Blob b{};b.found=true;b.color=ColorType::RED;b.centerX=36;
 b.minX=26;b.maxX=46;b.minY=70;b.maxY=100;b.area=600;
 for(int i=0;i<3;++i){clockMs+=80;auto r=obstacle_path_observe(&b);
  if(r.confirmed||r.seatId!=-1||seats[9].confirmed)return 1;}
 // Genuine distant middle remains available when there is no conflicting end.
 seats[7].confirmed=false;clockMs+=1000;parkingEntryConnectorActive=false;
 seats[8].pathDistanceMm=seats[9].pathDistanceMm=775;
 auto a=obstacle_path_observe(&b);clockMs+=80;auto c=obstacle_path_observe(&b);
 if(a.seatId!=9||!c.confirmed)return 2;
 // CHECK_ALL explicitly permits the user's arbitrary test layouts.
 seats[9]={};seats[9].x=900;seats[9].y=0;seats[7].confirmed=true;
 sectionLayoutMode=1;clockMs+=1000;obstacle_path_observe(&b);clockMs+=80;
 if(!obstacle_path_observe(&b).confirmed)return 3;
 std::cout<<"554 FAR MIDDLE conflict deferred; compatible far/CHECK_ALL acquisition retained\n";
}
'''
    run('phantom_middle',code)

    motor=(ROOT/'src/motor_control.cpp').read_text()
    code=r'''
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <iostream>
constexpr int A0=0,A1=1,A2=2;
constexpr float PI=3.14159265358979323846f;
#include "config.h"
'''
    for sig in ('struct NoProgressWatchdogState','struct NoProgressWatchdogEvidence'):
        code+=block(motor,motor.index(sig))+';\n'
    for sig in ('void reset_no_progress_watchdog(', 'bool update_no_progress_watchdog('):
        code+=block(motor,motor.index(sig))+'\n'
    code+=r'''
int main(){NoProgressWatchdogState s{};NoProgressWatchdogEvidence e{};
 for(int i=0;i<=30;++i)
  if(update_no_progress_watchdog(s,true,80,80,i*100000U,i*.83f,e))return 1;
 reset_no_progress_watchdog(s,0,0);
 update_no_progress_watchdog(s,true,80,80,0,0,e);
 if(!update_no_progress_watchdog(s,true,80,80,1000000,0,e))return 2;
 reset_no_progress_watchdog(s,0,0);
 update_no_progress_watchdog(s,true,80,80,0,0,e);
 if(!update_no_progress_watchdog(s,true,80,80,1000000,-8.3f,e))return 3;
 if(update_no_progress_watchdog(s,false,80,80,2000000,0,e)||s.armed)return 4;
 std::cout<<"552 WATCHDOG moving 8.3 mm/s retained; stationary/wrong-way stop at 1s\n";
}
'''
    run('watchdog_552',code)

    # Measured tail anchors avoid treating the encoder/gyro fit as an
    # independent reconstruction of the entire exit. Fit curvature only over
    # each recorded drive phase; physical tyre slip/pose bias remain unknown.
    rows=[]
    for n in (547,548,549,550,551,553,554,555):
        p=ROOT/f'simulation/evidence/parking_exit_diagnostics/20261027_log_{n}_{"cw" if n<551 else "ccw"}.txt'
        lines=p.read_text().splitlines()
        samples=[]
        for segment in (4,5):
            part=[dict(t.split('=',1) for t in line.split() if '=' in t)
                  for line in lines if 'type=sample ' in line and
                  f'state=segment_drive seg={segment} ' in line]
            samples.append(part)
        pose=list(map(float,samples[1][0]['pose'].split(',')))
        curvatures=[]
        for part in samples:
            a,b=part[0],part[-1]
            delta=(float(b['gyro'])-float(a['gyro'])+180)%360-180
            curvatures.append(delta/(float(b['emm'])-float(a['emm'])))
        rows.append([float(n),*pose,*curvatures])
    src=(ROOT/'src/obstacle.cpp').read_text()
    away=float(re.search(r'\{\+1, \+1, ([\d.]+)f\},\s*\{\+1, -1, OBSTACLE_PARKING_EXIT_FINAL',src).group(1))
    code=r'''
#include <cmath>
#include <cstdint>
#include <iostream>
constexpr int A0=0,A1=1,A2=2;
constexpr float PI=3.14159265358979323846f;
#include "config.h"
#include "parking_start_footprint.h"
const float records[][6]={
'''+',\n'.join('{'+','.join(str(v)+'f' for v in row)+'}' for row in rows)+'};\n'
    code+='constexpr float extension='+str(away-84)+'f;\n'
    code+=r'''
bool tail(const float *r,float extra){
 float x=r[1],y=r[2],h=r[3];bool aligned=false;
 for(int s=0;s<2;++s){
  float k=r[s+4]*PI/180;int steering=(s?-1:1)*(r[0]<551?1:-1);
  for(int mm=0;mm<=(s?OBSTACLE_PARKING_EXIT_FINAL_ALIGN_MAX_MM:extra);++mm){
   if(!parking_start_footprint::safe(x,y,h,steering,false,0))return false;
   if(s&&mm>=OBSTACLE_PARKING_EXIT_FINAL_ALIGN_MIN_MM&&
      fabsf(remainderf(h-(r[0]<551?180:0),360))<=OBSTACLE_PARKING_EXIT_FINAL_HEADING_TOLERANCE_DEG){aligned=true;break;}
   float a=h*PI/180,b=a+k;
   x+=(sinf(b)-sinf(a))/k;y+=(cosf(a)-cosf(b))/k;h=b*180/PI;
  }
 }
 return aligned;
}
int main(){int total=0;bool reproduced=false;
 for(const auto &r:records){
  if(r[0]==548)reproduced=!tail(r,0);
  if(!tail(r,extension)){std::cout<<"EXIT TAIL FAIL "<<r[0]<<"\n";return 1;}++total;
 }
 if(!reproduced)return 2;
 std::cout<<"EXIT TAIL "<<total<<" measured anchors PASS; original548 obstruction reproduced\n";
}
'''
    run('exit_reserve',code)


if __name__ == '__main__':
    main()
