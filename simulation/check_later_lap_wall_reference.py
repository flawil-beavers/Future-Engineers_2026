"""Host checks of production later-lap rectangular-wall geometry (no hardware)."""
import shutil, subprocess
from check_cw_start_planner import ROOT, fixture_source
from check_connector_servo_resume import block
s=(ROOT/'src/obstacle_path.cpp').read_text()
fixture=fixture_source().split('int main()')[0]
fixture+=block(s,s.index('bool laterLapWallReference('))
fixture+=r'''
int main(){
 for(auto &seat:seats)seat.confirmed=false;
 baseline();
 float worst=0;
 for(int i=0;i<pathLength;++i){
  const auto &p=baselinePath[i];const float h=p.headingDeg*PI/180;
  for(float side:{-1.f,1.f})for(float offset:{0.f,200.f,300.f}){
   float rx=-sinf(h)*side,ry=cosf(h)*side,d,nx,ny;
   if(!laterLapWallReference(p.x+rx*offset,p.y+ry*offset,rx,ry,d,nx,ny))continue;
   // Perfect pose and perfect sensor reading: old radial wall approximation
   // can still produce an accepted nonzero residual.
   const float oldResidual=side*(500-offset-d);
   if(fabsf(oldResidual)<150)worst=fmaxf(worst,fabsf(oldResidual));
  }
 }
 if(worst<20)return 8;
 std::cout<<"OLD perfect-pose accepted fictitious residual_mm="<<worst<<"\n";
 int checked=0;
 for(int turn=0;turn<4;++turn)for(int sign:{-1,1})for(float a:{-1000.f,0.f,1000.f}){
  const float angle=turn*PI/2,rx=cosf(angle),ry=sinf(angle);
  const float nx=-ry,ny=rx;
  const float sx=rx*1100+nx*a,sy=ry*1100+ny*a;
  float d,n1,n2;
  if(!laterLapWallReference(sx,sy,rx,ry,d,n1,n2)||fabsf(d-400)>0.01)return 1;
  // 40mm erroneous pose shift toward wall: residual must move pose back.
  float shifted,n3,n4;
  if(!laterLapWallReference(sx+rx*40,sy+ry*40,rx,ry,shifted,n3,n4))return 2;
  float residual=(shifted-d)*(rx*n3+ry*n4);
  if(fabsf(n3*residual+rx*40)>0.01||fabsf(n4*residual+ry*40)>0.01)return 3;
  ++checked;
 }
 float d,nx,ny;
 if(laterLapWallReference(1450,1100,0,1,d,nx,ny))return 4; // corner ambiguity
 if(laterLapWallReference(1000,1000,.7071,.7071,d,nx,ny))return 5;
 seats[0].confirmed=true;seats[0].x=1300;seats[0].y=0;
 if(laterLapWallReference(1100,0,1,0,d,nx,ny))return 6;
 seats[0].y=200;
 if(!laterLapWallReference(1100,0,1,0,d,nx,ny))return 7;
 std::cout<<"RECTANGULAR WALL PASS "<<checked<<" residual signs; cone/vertex/grazing gates PASS\n";
}
'''
p=ROOT/'local_workspace/later-wall';p.mkdir(parents=True,exist_ok=True)
(p/'check.cpp').write_text(fixture)
subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),str(p/'check.cpp'),'-o',str(p/'check.exe')],check=True)
subprocess.run([str(p/'check.exe')],check=True)
