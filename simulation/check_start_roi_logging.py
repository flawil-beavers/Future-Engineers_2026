"""Actual bounded start-ROI logger: stationary gate, rotation, budget, once/run."""
import subprocess,shutil
from check_connector_servo_resume import block
from check_cw_start_planner import ROOT
p=ROOT/'local_workspace/start-roi';p.mkdir(parents=True,exist_ok=True)
s=(ROOT/'src/obstacle_path.cpp').read_text()
f=r'''
#include <cmath>
#include <cstdint>
#include <sstream>
#include <iostream>
#include <iomanip>
using std::isfinite;
constexpr int A0=0,A1=1,A2=2;constexpr float PI=3.14159265358979323846;
#include "config.h"
struct PositionEstimate{float x_mm=300,y_mm=-1222,heading_deg=147.7;};
bool parkingStartRoiLogged=false,parkingCwShortStart=true,parkingEntryObserving=true;
int parkingEntryTargetStation=1;
uint32_t millis(){return 123;}
float clampFloat(float v,float a,float b){return v<a?a:(v>b?b:v);}
void seatCameraGeometry(uint8_t,const PositionEstimate&,float&b,float&r){b=-20;r=321;}
struct Vision{static bool rotates180(){return true;}static bool rgb565MsbFirst(){return true;}};
struct Camera{uint8_t b[153600];uint8_t*getBuffer(){return b;}int getWidth(){return 320;}int getHeight(){return 240;}}camera;
struct Log{std::ostringstream out;template<class T>void print(T x){out<<x;}
 template<class T>void print(T x,int){out<<x;}template<class T>void println(T x){out<<x<<"\n";}
 template<class T>void println(T x,int){out<<x<<"\n";}}Serial;
'''+block(s,s.index('void logParkingStartClearImage('))+r'''
int main(){
 for(int y=0;y<240;++y)for(int x=0;x<320;++x){
  const uint16_t raw=static_cast<uint16_t>((x*17+y*31)&65535);
  const int offset=((239-y)*320+319-x)*2;camera.b[offset]=raw>>8;camera.b[offset+1]=raw&255;
 }
 PositionEstimate pose;
 parkingEntryObserving=false;logParkingStartClearImage(2,pose);
 parkingEntryObserving=true;logParkingStartClearImage(4,pose);
 parkingCwShortStart=false;logParkingStartClearImage(2,pose);
 if(parkingStartRoiLogged||!Serial.out.str().empty())return 1;
 parkingCwShortStart=true;logParkingStartClearImage(2,pose);
 const auto once=Serial.out.str();logParkingStartClearImage(2,pose);
 if(Serial.out.str()!=once||once.size()+60>8192)return 2;
 std::cout<<once;
}
'''
(p/'check.cpp').write_text(f);exe=p/'check.exe'
subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),str(p/'check.cpp'),'-o',str(exe)],check=True)
t=subprocess.check_output([str(exe)],text=True);import re
m=re.search(r'x0/y0=(\d+)/(\d+)',t);x0,y0=map(int,m.groups())
rows=[l.split()[-1] for l in t.splitlines() if l.startswith('[START_ROI_ROW]')]
assert len(rows)==48
for j,row in enumerate(rows):
 expected=''.join(f'{((x0+2*i)*17+(y0+2*j)*31)&65535:04x}' for i in range(32))
 assert row==expected
assert t.rstrip().endswith('[START_ROI_END]')
print('START ROI stopped/CW/middle gate, once/run,48rows,RGB565 rotation PASS; bytes',len(t)+len(t.splitlines()))
