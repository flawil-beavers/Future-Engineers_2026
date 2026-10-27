"""Compare optimized parking check with exhaustive SAT at identical polygons."""
from pathlib import Path
import shutil,subprocess
ROOT=Path(__file__).resolve().parents[1]
p=ROOT/'local_workspace/fix_528_530';p.mkdir(parents=True,exist_ok=True)
code=r'''
#include "parking_start_footprint.h"
#include <iostream>
using namespace parking_start_footprint;
bool exhaustive(float x,float y,float h,int sign,bool behind,float margin,bool ahead){
 if(!isfinite(x)||!isfinite(y)||!isfinite(h))return false;
 Quad bodies[6];robotQuads(x,y,h,sign,margin,bodies);
 Quad fixed=rectangle(490,-1400,15,100,0);
 Quad moved[2]={rectangle(480-242.5f-10,-1400,15,100,0),rectangle(480-252.5f-10,-1400,15,100,0)};
 for(auto &q:bodies){for(auto &pt:q.p)
  if(pt.y<=-1495||(behind&&pt.x>=500)||(ahead&&pt.x<=0))return false;
  if(overlaps(q,fixed)||overlaps(q,moved[0])||overlaps(q,moved[1]))return false;
 }return true;
}
int main(){int n=0,bad=0;
 for(float x=100;x<=600;x+=25)for(float y=-1450;y<=-1100;y+=25)
 for(float h=-15;h<=180;h+=15)for(int sign:{-1,0,1})for(bool ccw:{false,true})for(float margin:{0.f,5.f}){
  ++n;if(safe(x,y,h,sign,!ccw,margin,ccw)!=exhaustive(x,y,h,sign,!ccw,margin,ccw))++bad;
 }
 // Returned starts: the new reference permits80mm more reverse here. The
 // deeper historical pose must retain a higher X instead of hitting pink.
 for(auto sample:{std::pair<float,float>{-1163.6f,357.4f},
                  {-1174.9f,359.1f},{-1187.6f,359.4f}}){
  float x=ccwObservationEntryX(sample.first,sample.second,270,350,109,84);
  ++n;if(x!=270)++bad;std::cout<<"RETURNED entry_x "<<x<<"\n";
 }
 ++n;if(ccwObservationEntryX(-1226,-.9f,270,350,109,84)<=270)++bad;
 std::cout<<"FOOTPRINT SAT equivalence "<<n<<" failures "<<bad<<"\n";return bad?1:0;
}
'''
(p/'footprint.cpp').write_text(code)
subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),str(p/'footprint.cpp'),'-o',str(p/'footprint.exe')],check=True)
r=subprocess.run([str(p/'footprint.exe')],capture_output=True,text=True);(p/'footprint_output.txt').write_text(r.stdout+r.stderr);print(r.stdout,end='');raise SystemExit(r.returncode)
