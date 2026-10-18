"""Frozen known-map first-lap geometry regression against the calibrated CAD plant.

Extracts actual route building, target selection and steering helpers. Covers
28 section arrangements, both directions, two caps and ordered neighbours.
This does not model perception, discovery holds, slip or servo/motor dynamics.
All artifacts are generated under ignored local_workspace.
"""
import sys,json,itertools,subprocess,shutil,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'simulation'))
from check_connector_servo_resume import block
dest=ROOT/'local_workspace/first-lap-geometry';dest.mkdir(parents=True,exist_ok=True)
from check_later_laps import later_lap_fixture
f=later_lap_fixture().split('int main()')[0]
s=(ROOT/'src/obstacle_path.cpp').read_text()
# Read the runtime expressions too, so reverting only the controller wiring
# cannot leave a green test that still calls the corrected helper directly.
runtime=block(s,s.index('void obstacle_path_update('))
steering_expression=re.search(r'const float requiredSteering = (.*?);',runtime,re.S).group(1)
lookahead_expression=re.search(r'float lookahead = (.*?);',runtime,re.S).group(1)
f+='\nfloat runtimeSteering(float curvature){return '+steering_expression+';}\n'
f+='float runtimeLookahead(float commandedSpeed){return '+lookahead_expression+';}\n'
f+='\nbool earlyMiddleViewActive[OBSTACLE_SEAT_COUNT]={};\n'
for signature in ('bool earlyMiddleViewEligible(', 'bool earlyMiddleViewPathSafe(', 'void rebuildLivePath()'):
    f+=block(s,s.index(signature))+'\n'
f+=r'''
bool rolloutFirstLap(bool cad,float cap,float &wall,float &pillar,int &reason,int &badSeat,float &x,float &y,float &h){
 wall=pillar=1e9f;reason=0;badSeat=-1;progressIndex=0;
 x=livePath[0].x;y=livePath[0].y;h=connectorRouteHeading(livePath,0)*PI/180;
 bool armed=false;float travel=0;
 bool witnessed[24]={};float previousForward[24];
 for(auto &v:previousForward)v=NAN;
 for(int step=0;step<6000;++step){
  uint16_t old=progressIndex;
  progressIndex=nearestPathIndex(livePath,x,y,progressIndex,OBSTACLE_PATH_PROGRESS_WINDOW);
  if(progressIndex>pathLength/4&&progressIndex<3*pathLength/4)armed=true;
  if(armed&&old>3*pathLength/4&&progressIndex<pathLength/4){
   for(int i=0;i<24;++i)if(seats[i].confirmed&&!witnessed[i]){reason=5;badSeat=i;return false;}
   return true;
  }
  parking_start_footprint::Quad footprints[2][6];
  for(int sign:{-1,1})parking_start_footprint::robotQuads(x,y,h*180/PI,sign,0,footprints[sign>0]);
  for(int i=0;i<24;++i){if(i&&!seats[i].confirmed)continue;
   ObstacleClearanceSample c{};calculateClearanceAtPose(seats[i],x,y,h*180/PI,c);
   wall=fminf(wall,c.wallMm);if(seats[i].confirmed)pillar=fminf(pillar,c.pillarMm);
   if(c.wallMm<=0||(seats[i].confirmed&&c.pillarMm<=0)){reason=c.wallMm<=0?1:2;badSeat=i;return false;}
   if(seats[i].confirmed&&!witnessed[i]){
    const float ref=seats[i].headingDeg*PI/180;
    float minForward=1e9f,minSide=1e9f,maxSide=-1e9f;
    for(const auto &bodies:footprints)for(const auto &body:bodies)for(const auto &p:body.p){
     float px=p.x-seats[i].x,py=p.y-seats[i].y;
     minForward=fminf(minForward,px*cosf(ref)+py*sinf(ref));
     float side=-px*sinf(ref)+py*cosf(ref);
     minSide=fminf(minSide,side);maxSide=fmaxf(maxSide,side);
    }
    const float phase=cyclicDistanceForward(seats[i].pathDistanceMm,baselinePath[progressIndex].distanceMm);
    if(phase<1000 && std::isfinite(previousForward[i]) && previousForward[i]<=0 && minForward>0){
     if(seats[i].red?maxSide>=-25:minSide<=25){reason=4;badSeat=i;return false;}
     witnessed[i]=true;
    }
    previousForward[i]=minForward;
   }
  }
  float speed=livePath[progressIndex].speedMmS;if(cap>0)speed=fminf(speed,cap);
  float look=runtimeLookahead(speed);
  if(nearCorner(baselinePath[progressIndex].distanceMm))look*=OBSTACLE_LOOKAHEAD_CORNER_SCALE;
  PositionEstimate pose{x,y,h*180/PI};auto t=findLookahead(livePath,pose,look);
  float dx=t.x-x,dy=t.y-y,k=2*(-dx*sinf(h)+dy*cosf(h))/fmaxf(1,dx*dx+dy*dy);
  int servo=static_cast<int>(clampFloat(runtimeSteering(k),-42,42));
  k=cad?(abs(servo)<2?0:-1.f/Ackermann::getTurnRadius(servo)):-tanf(servo*PI/180)/100;
  float next=h+k*2;
  if(fabsf(k)>1e-6){x+=(sinf(next)-sinf(h))/k;y+=(cosf(h)-cosf(next))/k;}
  else{x+=2*cosf(h);y+=2*sinf(h);}h=next;travel+=2;
 }
 reason=3;return false;
}
int main(){int id,dir,colors[24];float cap;
 while(std::cin>>id>>dir>>cap){for(int &c:colors)if(!(std::cin>>c))return 2;
  routeTurnSign=dir;resetState();baseline();
  for(int i=0;i<24;++i){seats[i].confirmed=seats[i].injected=colors[i]!=0;seats[i].red=colors[i]==2;}
  rebuildLivePath();
  for(int cad:{1}){float w,p,x,y,h;int why,seat;
   bool ok=rolloutFirstLap(cad,cap,w,p,why,seat,x,y,h);
   std::cout<<"ROUTE "<<id<<" "<<cad<<" "<<ok<<" "<<w<<" "<<p<<" "<<why<<" "<<seat<<" "<<x<<" "<<y<<" "<<h*180/PI<<"\n";
  }
 }
}
'''
(dest/'routes.cpp').write_text(f)
subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),'-I',str(ROOT/'simulation/host'),str(dest/'routes.cpp'),'-o',str(dest/'routes.exe')],check=True)
sections=[]
for station,side,color in itertools.product(range(3),range(2),(1,2)):
    colors=[0]*6;colors[2*station+side]=color;sections.append(colors)
for side1,side2,color1,color2 in itertools.product(range(2),range(2),(1,2),(1,2)):
    colors=[0]*6;colors[side1]=color1;colors[4+side2]=color2;sections.append(colors)
cases=[]
for direction,cap,section in itertools.product((-1,1),(0,140,175),sections):
    cases.append(dict(direction=direction,cap=cap,colors=[0]*6+section*3,kind='repeated'))
for direction,a,b in itertools.product((-1,1),sections,sections):
    cases.append(dict(direction=direction,cap=0,colors=[0]*6+a+b+a,kind='neighbours'))
inputs='\n'.join(' '.join(map(str,[i,c['direction'],c['cap'],*c['colors']])) for i,c in enumerate(cases))+'\n'
run=subprocess.run([str(dest/'routes.exe')],input=inputs,text=True,capture_output=True,check=True)
(dest/'route_output.txt').write_text(run.stdout)
for line in run.stdout.splitlines():
    _,index,cad,ok,wall,pillar,reason,seat,x,y,h=line.split()
    cases[int(index)]['cad' if int(cad) else 'ideal']=dict(passed=bool(int(ok)),min_wall=float(wall),min_pillar=float(pillar),reason=int(reason),seat=int(seat),pose=list(map(float,[x,y,h])))
(dest/'route_report.json').write_text(json.dumps(cases,indent=2)+'\n')
failed=[]
for direction in (-1,1):
    subset=[c for c in cases if c['direction']==direction]
    bad=[c for c in subset if not c['cad']['passed'] or c['cad']['min_pillar']<30 or c['cad']['min_wall']<30]
    failed+=bad
    print('FIRST LAP',direction,'total',len(subset),'failed',len(bad),'min_wall',min(c['cad']['min_wall'] for c in subset),'min_pillar',min(c['cad']['min_pillar'] for c in subset))
for c in failed[:4]:print('FAIL',c)
raise SystemExit(bool(failed))
