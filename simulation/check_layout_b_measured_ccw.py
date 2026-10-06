"""Production CCW connector replay for measured GREEN-front B starts.
Known colour and ideal bicycle; no camera/ToF/servo-delay acceptance.
Each planned perturbed start is also replayed against a fixed retained path
with independent actual-start perturbations and the physical42deg envelope.
Generated code and outputs remain ignored in local_workspace.
"""
import sys,subprocess,itertools,re,shutil
sys.path.insert(0,'simulation')
from check_cw_start_planner import fixture_source,ROOT
from check_connector_servo_resume import block
f=fixture_source().split('int main()')[0]
f=f.replace('int routeTurnSign=-1,parkingEntryTargetStation=1;', 'int routeTurnSign=1,parkingEntryTargetStation=2;').replace('bool parkingCwShortStart=true,','bool parkingCwShortStart=false,').replace('bool parkingCcwShortStart=false;','bool parkingCcwShortStart=true;').replace('anchor.heading_deg=180;','anchor.heading_deg=0;')
source=(ROOT/'src/obstacle_path.cpp').read_text()
f+=block(source,source.index('bool connectorRolloutFeasible(')).replace('connectorRolloutFeasible(', 'runtimePerturbedRollout(',1).replace('OBSTACLE_PARKING_CONNECTOR_PLAN_STEERING_DEG','OBSTACLE_MAX_PURSUIT_STEERING_DEG')
f+='''
int main(){float x,y,h;int n=0,failed=0;while(std::cin>>x>>y>>h){baseline();memcpy(livePath,baselinePath,sizeof(PathPoint)*pathLength);for(auto &d:discoveryStations)d.observedClear=false;seats[5].confirmed=seats[5].injected=true;seats[5].red=false;displaceForSeat(livePath,5,260,true);PositionEstimate p;p.x_mm=x;p.y_mm=y;p.heading_deg=h;uint16_t merge=0;bool ok=buildParkingEntryConnector(p,livePath,merge);if(ok){for(float dx:{-3.f,0.f,3.f})for(float dy:{-3.f,0.f,3.f})for(float dh:{-1.f,0.f,1.f}){PositionEstimate actual=p;actual.x_mm+=dx;actual.y_mm+=dy;actual.heading_deg+=dh;if(!runtimePerturbedRollout(actual,parkingEntryConnectorLookaheadMm,5,5,3,livePath,merge,true))ok=false;}}std::cout<<n++<<" "<<ok<<" "<<parkingEntryConnectorLookaheadMm<<"\\n";failed+=!ok;}return failed?1:0;}
'''
p=ROOT/'local_workspace/b_measured_ccw';p.mkdir(exist_ok=True);(p/'check.cpp').write_text(f);subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),str(p/'check.cpp'),'-o',str(p/'check.exe')],check=True)
rows=[]
for n in (479,480,481):
 s=(ROOT/f'simulation/evidence/parking_exit_diagnostics/20261006_log_{n}_ccw.txt').read_text();m=re.search(r'kind=connector index=0 x=([-\d.]+) y=([-\d.]+) h=([-\d.]+)',s);assert m;pose=list(map(float,m.groups()))
 for dx,dy,dh in itertools.product((-3,0,3),(-3,0,3),(-1,0,1)):rows.append([pose[0]+dx,pose[1]+dy,pose[2]+dh])
r=subprocess.run([str(p/'check.exe')],input='\n'.join(' '.join(map(str,r)) for r in rows)+'\n',text=True,capture_output=True);(p/'output.txt').write_text(r.stdout);print('Measured CCW perturbed:',len(rows),'failures',sum(l.split()[1]=='0' for l in r.stdout.splitlines()));sys.exit(r.returncode)
