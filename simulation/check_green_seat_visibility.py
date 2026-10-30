"""Compile actual Vision and replay mapped-seat checks; no firmware upload."""
from pathlib import Path
import subprocess,shutil,json
ROOT=Path(__file__).resolve().parents[1]
p=ROOT/'local_workspace/green-seat-visibility';p.mkdir(parents=True,exist_ok=True)
(p/'Arduino.h').write_text('''#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <cmath>
using std::max; using std::min;
constexpr int A0=0,A1=1,A2=2;
constexpr double PI=3.141592653589793;
inline uint32_t micros(){return 0;}
''')
(p/'check.cpp').write_text('''#include "vision.h"
#include <fstream>
#include <iostream>
#include <vector>
int main(int argc,char**argv){
 std::vector<uint8_t>b(153600);std::ifstream f(argv[1],std::ios::binary);
 f.read(reinterpret_cast<char*>(b.data()),b.size());if(f.gcount()!=153600)return 2;
 static Vision v;v.begin();GreenSeatCandidate c;
 bool found=v.findGreenSeatCandidate(b.data(),320,240,atoi(argv[2]),atoi(argv[3]),c);
 std::cout<<found<<" "<<c.silhouetteFound<<" "<<c.blob.centerX<<" "<<c.blob.maxY<<" "<<c.greenSamples<<"\\n";
}
''')
exe=p/'check.exe';subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(p),'-I',str(ROOT/'include'),str(ROOT/'src/vision.cpp'),str(p/'check.cpp'),'-o',str(exe)],check=True)
def run(path,x,y):return list(map(int,subprocess.check_output([str(exe),str(path),str(x),str(y)],text=True).split()))
def raw(rects):
 b=bytearray(153600)
 for y in range(240):
  for x in range(320):
   rgb=(255,255,255)
   for x0,y0,x1,y1,c in rects:
    if x0<=x<=x1 and y0<=y<=y1:rgb=c
   r,g,bb=rgb;v=((r>>3)<<11)|((g>>2)<<5)|(bb>>3)
   i=((239-y)*320+319-x)*2;b[i]=v>>8;b[i+1]=v&255
 return b
cases=[('green_dark',[(214,80,246,142,(25,70,25))],1,1),
 ('green_bright',[(214,80,246,142,(55,170,55))],1,1),
 ('red',[(214,80,246,142,(160,20,20))],0,1),
 ('gray_unknown',[(214,80,246,142,(70,70,70))],0,1),
 ('empty',[],0,0),('green_wall',[(0,80,319,94,(25,70,25))],0,0),
 ('floor_line',[(0,142,319,145,(25,70,25))],0,0),
 ('wall_and_floor',[(0,80,319,94,(25,70,25)),(214,142,246,145,(20,20,20))],0,0)]
for name,rects,color,occupied in cases:
 path=p/(name+'.rgb565');path.write_bytes(raw(rects));r=run(path,230,142)
 print(name,r);assert r[:2]==[color,occupied],(name,r)

#576: a dark green candidate's actual208px foot is44px below the164px
#projection of the middle place. The old function still returned it, allowing
#three false middle votes and inference that skipped the preceding scout.
path=p/'wrong_middle_foot.rgb565'
path.write_bytes(raw([(246,96,280,208,(25,70,25))]))
r=run(path,265,164);print('576_wrong_middle_foot',r);assert r[:2]==[0,0]
r=run(path,265,208);print('same_close_green_correct_foot',r);assert r[:2]==[1,1]

# Negative control keeps every other colour/shape condition and removes only
# the actual mapped-foot rejection added for576.
vision_source=(ROOT/'src/vision.cpp').read_text()
guard='''        else
            continue;
        // Count existing HSV-green pixels'''
assert guard in vision_source
old=p/'vision_without_foot_gate.cpp'
old.write_text(vision_source.replace(guard,'        // Count existing HSV-green pixels',1))
old_exe=p/'without_foot_gate.exe'
subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(p),'-I',str(ROOT/'include'),
               str(old),str(p/'check.cpp'),'-o',str(old_exe)],check=True)
old_result=list(map(int,subprocess.check_output([str(old_exe),str(path),'265','164'],text=True).split()))
print('576_removed_foot_gate_negative_control',old_result);assert old_result[0]==1
evidence=ROOT/'simulation/evidence/camera_diagnostics'
same=0;accepted=0
for image in sorted(evidence.glob('20261026_green*400mm*.rgb565')):
 current=run(image,160,136)
 previous=list(map(int,subprocess.check_output([str(old_exe),str(image),'160','136'],text=True).split()))
 assert current==previous,(image.name,current,previous)
 same+=1;accepted+=current[0]
print('Archived Oct26 400mm fixed-query old/new matches',same,'accepted',accepted)
for name,color in [('20261005_green_middle_same_pose_01',1),('20261005_red_green_room_same_pose_01',0)]:
 r=run(evidence/(name+'.rgb565'),196,124);print(name,r);assert r[0]==color
# These images contain a distant rear GREEN and an empty front: a false close
# foot at the front projection must neither confirm GREEN nor block empty.
for name in ['20260928_s1_empty_front_green_rear_stationary_01','20260929_s1_empty_front_recheck_01']:
 r=run(evidence/(name+'.rgb565'),220,170);print(name,r);assert r[0]==0
print('VISION archived GREEN/RED and synthetic brightness/occupancy/background checks PASS')
