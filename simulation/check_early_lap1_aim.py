"""Compare early checked aiming with the previous guard on archived approaches.

Synthetic camera visibility, not colour recognition. No hardware operations.
The paired controller rollouts share seeds/plant; generated files stay local.
"""
import json
import shutil
import subprocess
from pathlib import Path

from check_connector_servo_resume import block
from check_lap1_approach_view import fixture_source

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'local_workspace/early-lap1-aim'


def policy_checks(source):
    """Actual dispatch with controlled geometry/preflight outcomes."""
    fixture=r'''
#include <cmath>
#include <cstdint>
#include <iostream>
constexpr float PI=3.14159265358979323846f;
constexpr int A0=0,A1=1,A2=2;
#include "config.h"
struct PathPoint{float x=0,y=0,distanceMm=0;};
struct PositionEstimate{float x_mm=0,y_mm=0,heading_deg=0;};
PathPoint baselinePath[1];int progressIndex=0,completedLaps=0,routeTurnSign=1;
int discoveryScanStation=3,discoveryScanSide=0,safeCalls=0;
float lastDiscoveryTargetNudgeDeg=45,normalNudge=45,hostBearing=30,hostRange=500;
unsigned long lastDiscoveryNudgeUpdateMs=980;
bool protectedPass=false,safe=true;
unsigned long millis(){return 1000;}
float clampFloat(float x,float a,float b){return fmaxf(a,fminf(b,x));}
bool traversingInjectedAvoidance(float){return protectedPass;}
void seatCameraGeometry(int,const PositionEstimate&,float &b,float &r){b=hostBearing;r=hostRange;}
float pursuitServoForCurvature(float k){return k*1000;}
bool earlyDiscoveryAimCommandSafe(const PositionEstimate&,int){++safeCalls;return safe;}
void applyDiscoveryTargetNudge(PathPoint &p,const PositionEstimate&){
 lastDiscoveryTargetNudgeDeg=normalNudge;lastDiscoveryNudgeUpdateMs=millis();
 float a=normalNudge*PI/180;p.x=200*cosf(a);p.y=200*sinf(a);}
constexpr int COURSE_SEATS_PER_STATION=2;
''' + block(source,source.index('void applyLap1DiscoveryAim('))+r'''
int main(){int checks=0;PositionEstimate p{};
 for(int dir:{-1,1})for(int lap:{0,1})for(int side:{-2,0,1})
 for(float range:{299.f,300.f,500.f,750.f,751.f})for(float bearing:{25.f,30.f,-30.f})
 for(int available:{0,1}){
  routeTurnSign=dir;completedLaps=lap;discoveryScanStation=3;discoveryScanSide=side;
  hostRange=range;hostBearing=bearing;safe=available;safeCalls=0;protectedPass=false;
  normalNudge=copysignf(45.f,bearing);lastDiscoveryTargetNudgeDeg=normalNudge;
  lastDiscoveryNudgeUpdateMs=980;PathPoint t{200,0};applyLap1DiscoveryAim(t,p);
  bool eligible=dir>0&&lap==0&&side>=0&&range>=300&&range<=750&&fabsf(bearing)>26.426f;
  float expected=copysignf(eligible&&available?46.2f:45.f,bearing);
  if(fabsf(lastDiscoveryTargetNudgeDeg-expected)>.001f || safeCalls!=(eligible?1:0))return 1;
  ++checks;
 }
 // Extra angles are rechecked during their smooth decay even after the
 // single-place/range eligibility has ended; unsafe decay falls back to45.
 for(int available:{0,1}){
  routeTurnSign=1;completedLaps=0;discoveryScanSide=-2;hostRange=200;
  normalNudge=48.8f;lastDiscoveryTargetNudgeDeg=50;lastDiscoveryNudgeUpdateMs=980;
  safe=available;safeCalls=0;PathPoint t{200,0};applyLap1DiscoveryAim(t,p);
  if(safeCalls!=1||fabsf(lastDiscoveryTargetNudgeDeg-(available?48.8f:45.f))>.001f)return 2;
  ++checks;
 }
 std::cout<<"CCW AIM DISPATCH "<<checks<<" PASS\n";
}
'''
    for name,code,expected in (
        ('policy',fixture,0),
        ('policy_no_preflight',fixture.replace('earlyDiscoveryAimCommandSafe(pose,steering)', 'true'),1)):
        cpp,exe=OUT/(name+'.cpp'),OUT/(name+'.exe')
        cpp.write_text(code)
        subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),
                        str(cpp),'-o',str(exe)],check=True)
        r=subprocess.run([str(exe)],capture_output=True,text=True)
        (OUT/(name+'.txt')).write_text(r.stdout+r.stderr)
        assert r.returncode==expected,(name,r.returncode,r.stdout,r.stderr)
        print(r.stdout.strip() if not expected else 'NEGATIVE CONTROL: missing preflight rejected')

MAIN = r'''
void clean(int dir,int previous,int station,bool red){
 routeTurnSign=dir;resetState();baseline();
 for(auto &s:seats)s.confirmed=s.injected=false;
 for(auto &d:discoveryStations){d.observedClear=true;d.seatObservedClear[0]=d.seatObservedClear[1]=true;}
 discoveryStations[station].observedClear=false;
 discoveryStations[station].seatObservedClear[0]=discoveryStations[station].seatObservedClear[1]=false;
 seats[previous].confirmed=seats[previous].injected=true;seats[previous].red=red;
 learned();lastDiscoveryTargetNudgeDeg=0;lastDiscoveryNudgeUpdateMs=clockMs;
 raw={};lastDiscoveryObservation={};}
void advance(PositionEstimate &p,float k,float ds){
 float h=p.heading_deg*PI/180,n=h+k*ds;
 p.x_mm+=fabsf(k)>1e-6f?(sinf(n)-sinf(h))/k:ds*cosf(h);
 p.y_mm+=fabsf(k)>1e-6f?(cosf(h)-cosf(n))/k:ds*sinf(h);
 p.heading_deg=wrap180(n*180/PI);}
int main(){
 // Original full bypass guard remains authoritative in both directions.
 int checks=0;
 for(int dir:{-1,1}){
  clean(-1,4,3,true);PositionEstimate p{-615,-750,180};
  if(dir>0){routeTurnSign=1;baseline();p.x_mm=-p.x_mm;p.heading_deg=0;
   for(auto &s:seats)s.confirmed=s.injected=false;
   seats[4].confirmed=seats[4].injected=true;seats[4].red=false;learned();}
  progressIndex=nearestPathIndex(livePath,p.x_mm,p.y_mm,0,pathLength);
  float d=baselinePath[progressIndex].distanceMm;
  if(!traversingInjectedAvoidance(d))return 1;
  PathPoint t{};t.x=p.x_mm-100;t.y=p.y_mm+200;auto before=t;
  lastDiscoveryTargetNudgeDeg=49;applyLap1DiscoveryAim(t,p);
  if(t.x!=before.x||t.y!=before.y||lastDiscoveryTargetNudgeDeg!=0)return 2;
  if(earlyDiscoveryAimCommandSafe(PositionEstimate{1480,0,0},0))return 7;
  checks+=3;
 }
 std::cout<<"GUARDS "<<checks<<" PASS\n";
 struct Seed{int dir,previous,target;bool red;float x,y,h;};
 const Seed seeds[]={
  {-1,4,7,true,-273.8f,-863.6f,140.9f}, //654 first GREEN approach
  {-1,4,7,true,-374.3f,-794.7f,146.2f}, //632 first GREEN approach
  {-1,10,13,true,-970.5f,146.7f,54.4f}, //651 unresolved empty station
  {1,14,19,false,226.7f,955.7f,206.2f}, //655 RED after middle GREEN
  {-1,4,7,true,-239.f,-882.f,144.3f},  //previous coupled regression seed
  {1,19,22,true,-984.f,-32.f,273.8f}, //previous CCW next GREEN regression seed
  {1,10,14,true,1089.4f,213.8f,48.4f} //655 approach to middle GREEN
 };
 int id=0;
 for(int scene=0;scene<7;++scene)for(float dx:{-5.f,0.f,5.f})
 for(float dy:{-5.f,0.f,5.f})for(float dh:{-1.f,0.f,1.f})for(float gain:{.9f,1.f,1.1f}){
  auto seed=seeds[scene];clean(seed.dir,seed.previous,seed.target/2,seed.red);
  PositionEstimate p{seed.x+dx,seed.y+dy,seed.h+dh};
  progressIndex=nearestPathIndex(livePath,p.x_mm,p.y_mm,0,pathLength);
  unsigned long nextFrame=clockMs;float visibleMs=0,longestMs=0,currentMs=0;
  float minWall=1e9f,minPillar=1e9f,firstRange=-1;int emptyVotes[2]={};
  int earlyCommands=0;float maxNudgeStep=0,oldNudge=0;
  bool reached=false;
  for(int step=0;step<1000;++step){
   progressIndex=nearestPathIndex(livePath,p.x_mm,p.y_mm,progressIndex,OBSTACLE_PATH_PROGRESS_WINDOW);
   float forward=cyclicDistanceForward(baselinePath[progressIndex].distanceMm,seats[seed.target].pathDistanceMm);
   if(forward<=OBSTACLE_DISCOVERY_HOLD_DISTANCE_MM){reached=true;break;}
   float speed=forward<=OBSTACLE_DISCOVERY_SLOW_DISTANCE_MM?100:livePath[progressIndex].speedMmS;
   unsigned dt=static_cast<unsigned>(lroundf(2000/fmaxf(1,speed)));
   for(int seat:{seed.previous,seed.target}){
    ObstacleClearanceSample c{};calculateClearanceAtPose(seats[seat],p.x_mm,p.y_mm,p.heading_deg,c);
    minWall=fminf(minWall,c.wallMm);minPillar=fminf(minPillar,c.pillarMm);
   }
   if(clockMs>=nextFrame){
    nextFrame=clockMs+80;float bearing,range;seatCameraGeometry(seed.target,p,bearing,range);
    const float limit=OBSTACLE_CAMERA_HORIZONTAL_FOV_DEG*OBSTACLE_DISCOVERY_FOV_FRACTION-
        OBSTACLE_DISCOVERY_CLEAR_FOV_MARGIN_DEG;
    bool visible=range>=230 && range<=600 && fabsf(bearing)<=limit;
    currentMs=visible?currentMs+80:0;visibleMs+=visible?80:0;
    longestMs=fmaxf(longestMs,currentMs);if(visible&&firstRange<0)firstRange=range;
    const int empty=(seed.target^1U);float eb,er;seatCameraGeometry(empty,p,eb,er);
    if(er>=230&&er<=600&&fabsf(eb)<=limit)++emptyVotes[empty%2];else emptyVotes[empty%2]=0;
    if(emptyVotes[empty%2]>=2)discoveryStations[empty/2].seatObservedClear[empty%2]=true;
   }
   float look=firstLapLookahead(speed);
   if(nearCorner(baselinePath[progressIndex].distanceMm))look*=OBSTACLE_LOOKAHEAD_CORNER_SCALE;
   auto target=findLookahead(livePath,p,look);auto before=target;
   bool guarded=traversingInjectedAvoidance(baselinePath[progressIndex].distanceMm);
   clockMs+=dt;applyLap1DiscoveryAim(target,p);
   maxNudgeStep=fmaxf(maxNudgeStep,fabsf(lastDiscoveryTargetNudgeDeg-oldNudge));oldNudge=lastDiscoveryTargetNudgeDeg;
   if(guarded&&(fabsf(target.x-before.x)>.001f||fabsf(target.y-before.y)>.001f))++earlyCommands;
   float h=p.heading_deg*PI/180,tx=target.x-p.x_mm,ty=target.y-p.y_mm;
   float k=2*(-tx*sinf(h)+ty*cosf(h))/fmaxf(1,tx*tx+ty*ty);
   int servo=int(clampFloat(pursuitServoForCurvature(k),-42,42));
   advance(p,pursuitCurvatureForServo(servo)*gain,2);
  }
  float finalBearing,finalRange;seatCameraGeometry(seed.target,p,finalBearing,finalRange);
  std::cout<<"ROUTE "<<id++<<" "<<scene<<" "<<reached<<" "<<minWall<<" "<<minPillar
   <<" "<<firstRange<<" "<<visibleMs<<" "<<longestMs<<" "<<earlyCommands<<" "<<maxNudgeStep
   <<" "<<fabsf(finalBearing)<<" "<<finalRange<<"\n";
 }
}
'''


def run(name, fixture):
    cpp, exe = OUT/(name+'.cpp'), OUT/(name+'.exe')
    cpp.write_text(fixture+MAIN)
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'include'),
                    '-I', str(ROOT/'simulation/host'), str(cpp), '-o', str(exe)], check=True)
    result = subprocess.run([str(exe)], capture_output=True, text=True)
    (OUT/(name+'.txt')).write_text(result.stdout+result.stderr)
    if result.returncode:
        raise RuntimeError(f'{name}: {result.returncode}\n{result.stdout}{result.stderr}')
    rows = []
    for line in result.stdout.splitlines():
        if line.startswith('ROUTE '):
            values = list(map(float, line.split()[1:]))
            rows.append(dict(zip(('id','scene','reached','wall','pillar','first_range',
                                  'visible_ms','longest_ms','early_commands','max_nudge_step',
                                  'final_bearing','final_range'), values)))
    return rows


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    source=(ROOT/'src/obstacle_path.cpp').read_text()
    runtime=block(source,source.index('void obstacle_path_update('))
    assert 'applyLap1DiscoveryAim(target, pose);' in runtime
    policy_checks(source)
    fixture=fixture_source()
    old=block(fixture,fixture.index('void applyLap1DiscoveryAim('))
    previous=r'''void applyLap1DiscoveryAim(PathPoint &target,const PositionEstimate &pose){
     if(!traversingInjectedAvoidance(baselinePath[progressIndex].distanceMm))
      applyDiscoveryTargetNudge(target,pose);
     else{lastDiscoveryTargetNudgeDeg=0;lastDiscoveryNudgeUpdateMs=millis();
      discoveryScanStation=discoveryScanSide=-1;}}
    '''
    current=run('current',fixture)
    baseline=run('baseline',fixture.replace(old,previous))
    (OUT/'comparison.json').write_text(json.dumps(dict(current=current,baseline=baseline),indent=2)+'\n')
    for scene in range(7):
        pairs=[(a,b) for a,b in zip(current,baseline) if a['scene']==scene]
        print('SCENE',scene,'cases',len(pairs),'early',sum(a['early_commands']>0 for a,b in pairs),
              'longer/equal/shorter',*[sum((a['longest_ms']>b['longest_ms'],
                 a['longest_ms']==b['longest_ms'],a['longest_ms']<b['longest_ms'])[i] for a,b in pairs) for i in range(3)],
              'min wall',min(a['wall'] for a,b in pairs),'min pillar',min(a['pillar'] for a,b in pairs))
        print(' final bearing change mean',sum(a['final_bearing']-b['final_bearing'] for a,b in pairs)/len(pairs),
              'worst',max(a['final_bearing']-b['final_bearing'] for a,b in pairs))
    bad=[a for a,b in zip(current,baseline) if not a['reached'] or
         a['wall']<30 or a['pillar']<30 or (b['longest_ms']>=160 and a['longest_ms']<160)]
    # CW must retain its trajectory, visibility and reserves exactly.
    for a,b in zip(current,baseline):
        if a['scene'] in (0,1,2,4) and a!=b:
            bad.append(a)
    assert any(a['longest_ms']>b['longest_ms'] for a,b in zip(current,baseline) if a['scene']==3)
    print('BAD',len(bad),bad[:2])
    raise SystemExit(bool(bad))


if __name__=='__main__':
    main()
