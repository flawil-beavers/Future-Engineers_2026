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
 if(mappedParkingSeatFootMatches(0,pose,174,false))return 1;
 if(!mappedParkingSeatFootMatches(0,pose,152,false))return 2;
 pose.x_mm=300;pose.y_mm=-1222;pose.heading_deg=147.7;
 if(mappedParkingSeatFootMatches(2,pose,184,false))return 3;
 if(!mappedParkingSeatFootMatches(2,pose,158,false))return 4;
 if(!mappedParkingSeatFootMatches(6,pose,184,false))return 5;
 // Returned log589: real GREEN5 at the stationary parking-entry view.
 pose.x_mm=192.2f;pose.y_mm=-1204.1f;pose.heading_deg=38.7f;
 float h=pose.heading_deg*PI/180.0f;
 float cameraX=pose.x_mm+OBSTACLE_CAMERA_LOCAL_X_MM*cosf(h);
 float cameraY=pose.y_mm+OBSTACLE_CAMERA_LOCAL_X_MM*sinf(h);
 seats[5].x=cameraX+300.0f*cosf(h);seats[5].y=cameraY+300.0f*sinf(h);
 float bearing=0,range=0;seatCameraGeometry(5,pose,bearing,range);
 int expected=int(lroundf(OBSTACLE_CAMERA_GROUND_HORIZON_Y+
     OBSTACLE_CAMERA_GROUND_RANGE_SCALE_MM_PX/(range*cosf(bearing*PI/180.0f))));
 // The returned true observations were 22--28 px below the mapped foot.
 if(!mappedParkingSeatFootMatches(5,pose,expected+24,true))return 6;
 if(mappedParkingSeatFootMatches(5,pose,expected+24,false))return 7;
 // False middle GREEN576 remains outside the original tight gate.
 pose.x_mm=300;pose.y_mm=-1222;pose.heading_deg=147.7;
 if(mappedParkingSeatFootMatches(2,pose,208,true))return 8;
 parkingSectionInnerSeatsOnly=false;
 if(!mappedParkingSeatFootMatches(2,pose,184,false))return 9;
 std::cout<<"START FOOT log471/576 false projections rejected; log589 end GREEN accepted; compatibility PASS\n";
}
'''
p=ROOT/'local_workspace/start-foot';p.mkdir(parents=True,exist_ok=True);(p/'check.cpp').write_text(f)
subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),'-I',str(ROOT/'simulation/host'),str(p/'check.cpp'),'-o',str(p/'check.exe')],check=True)
subprocess.run([str(p/'check.exe')],check=True)
