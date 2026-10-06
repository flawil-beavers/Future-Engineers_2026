"""Production projected-foot gate: failed start471 vs plausible close pillars."""
import shutil,subprocess
from check_cw_start_planner import ROOT,fixture_source
from check_connector_servo_resume import block
s=(ROOT/'src/obstacle_path.cpp').read_text();f=fixture_source().split('int main()')[0]
f+='\nbool parkingSectionInnerSeatsOnly=true;\n'
f+=block(s,s.index('void seatCameraGeometry('))
f+=block(s,s.index('bool mappedParkingSeatFootMatches('))
f+=r'''
int main(){
 baseline();PositionEstimate pose;
 pose.x_mm=355;pose.y_mm=-1332;pose.heading_deg=78.9;
 if(mappedParkingSeatFootMatches(0,pose,174))return 1;
 if(!mappedParkingSeatFootMatches(0,pose,152))return 2;
 pose.x_mm=300;pose.y_mm=-1222;pose.heading_deg=147.7;
 if(mappedParkingSeatFootMatches(2,pose,184))return 3;
 if(!mappedParkingSeatFootMatches(2,pose,158))return 4;
 if(!mappedParkingSeatFootMatches(6,pose,184))return 5;
 parkingSectionInnerSeatsOnly=false;
 if(!mappedParkingSeatFootMatches(2,pose,184))return 6;
 std::cout<<"START FOOT log471 false close projections rejected; plausible feet/distant/nonparking compatibility PASS\n";
}
'''
p=ROOT/'local_workspace/start-foot';p.mkdir(parents=True,exist_ok=True);(p/'check.cpp').write_text(f)
subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),str(p/'check.cpp'),'-o',str(p/'check.exe')],check=True)
subprocess.run([str(p/'check.exe')],check=True)
