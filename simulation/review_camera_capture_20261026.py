"""Replay current Vision on archived field captures; quantify the observed wrap.

Images and physical distances are evidence, not universal camera acceptance.
Generated code/results stay in ignored local_workspace. Requires Pillow and g++.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import shutil
import subprocess
from PIL import Image, ImageChops, ImageStat
from check_connector_servo_resume import block

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir',type=Path,default=ROOT/'simulation/evidence/camera_diagnostics')
    parser.add_argument('--output-dir',type=Path,default=ROOT/'local_workspace/camera-test-20261026/replay')
    args=parser.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    spec=importlib.util.spec_from_file_location('camshot',ROOT/'scripts/capture-camera-image.py')
    capture=importlib.util.module_from_spec(spec);spec.loader.exec_module(capture)
    shim="""#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <cmath>
using std::isfinite;using std::min;using std::max;
constexpr int A0=0,A1=1,A2=2;
constexpr double PI=3.141592653589793;
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
inline uint32_t micros(){return 0;}
template<class T>T constrain(T x,T a,T b){return x<a?a:(x>b?b:x);}
"""
    (args.output_dir/'Arduino.h').write_text(shim)
    source=(ROOT/'src/obstacle_vision.cpp').read_text()
    code='''#include "vision.h"
#include "obstacle_blob_validation.h"
#include <iostream>
#include <fstream>
#include <vector>
Vision vision;
bool obstacle_blob_valid_for_acquisition(const Blob*b){return obstacle_blob_shape_valid(b);}
'''
    for sig in ('float obstacle_camera_bearing_deg(', 'float obstacle_estimate_camera_forward_mm(',
                'float obstacle_estimate_camera_range_mm(', 'const Blob *getLargestValidObstacle()'):
        code+=block(source,source.index(sig))+'\n'
    code+=r'''void show(const char*name,const Blob&b){std::cout<<name<<" found="<<b.found<<" valid="<<obstacle_blob_shape_valid(&b)<<" x="<<b.centerX<<" foot="<<b.maxY<<" width="<<b.width()<<" height="<<b.height()<<" area="<<b.area<<" bearing="<<obstacle_camera_bearing_deg(&b)<<" forward="<<obstacle_estimate_camera_forward_mm(&b)<<" range="<<obstacle_estimate_camera_range_mm(&b)<<"\n";}
int main(int argc,char**argv){if(argc!=2)return 2;std::vector<uint8_t>b(153600);std::ifstream f(argv[1],std::ios::binary);f.read((char*)b.data(),b.size());if(f.gcount()!=153600)return 3;vision.begin();vision.update(b.data(),320,240);const auto&r=vision.getResult();show("RED_raw",r.red);show("RED_pillar",r.redPillar);show("GREEN_raw",r.green);show("GREEN_pillar",r.greenPillar);const Blob*selected=getLargestValidObstacle();std::cout<<"selected="<<(selected?(int)selected->color:0)<<"\n";
for(int x:{150,156,164}){GreenSeatCandidate c;bool ok=vision.findGreenSeatCandidate(b.data(),320,240,x,138,c);std::cout<<"local_green expected_x="<<x<<" found="<<ok<<" occupied="<<c.silhouetteFound<<" foot="<<c.blob.maxY<<"\n";}}
'''
    cpp=args.output_dir/'inspect.cpp';exe=args.output_dir/'inspect.exe';cpp.write_text(code)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(args.output_dir),'-I',str(ROOT/'include'),
                    str(ROOT/'src/vision.cpp'),str(cpp),'-o',str(exe)],check=True)
    records=[]
    for path in sorted(args.source_dir.glob('20261026_*.serial.bin')):
        if not any(token in path.name for token in ('red_center','red_after_reset','green_same_pose','green_new_pose','green_syncdiag','green_autosync','green_final','red_same_pose')):continue
        fields,payload=capture.decode_wire(path.read_bytes())
        base=path.name.removesuffix('.serial.bin');raw=args.output_dir/(base+'.rgb565');raw.write_bytes(payload)
        result=subprocess.check_output([str(exe),str(raw)],text=True)
        records.append(dict(source=path.name,metadata=fields,production_replay=result))
        print(base,result.splitlines()[0],result.splitlines()[2])
    wraps=[]
    for first_name,second_name in [('20261026_red_center_400mm_01.png','20261026_red_center_400mm_02.png'),
            ('20261026_green_new_pose_400mm_01.png','20261026_green_new_pose_400mm_02.png'),
            ('20261026_green_syncdiag_400mm_01.png','20261026_green_syncdiag_400mm_02.png')]:
        a=args.source_dir/first_name;b=args.source_dir/second_name
        if not (a.exists() and b.exists()):continue
        first=Image.open(a).convert('RGB');second=Image.open(b).convert('RGB')
        scores=[]
        for shift in range(320):
            difference=ImageChops.difference(ImageChops.offset(first,shift,0),second).crop((0,85,320,225))
            scores.append(sum(ImageStat.Stat(difference).mean)/3)
        best=min(range(320),key=scores.__getitem__)
        wrap=dict(comparison=[a.name,b.name],best_cyclic_x_shift_px=best,
                  mean_absolute_rgb_difference=scores[best],unshifted_difference=scores[0],
                  limitation='Observed cyclic displacement; does not establish wiring, sensor, DMA or startup root cause.')
        wraps.append(wrap);print('WRAP',wrap)
    references=[('20261026_red_after_reset_01.serial.bin','RED_raw',400),
        ('20261026_green_same_pose_400mm_01.serial.bin','GREEN_raw',400),
        ('20261026_red_same_pose_600mm_01.serial.bin','RED_raw',600),
        ('20261026_green_same_pose_600mm_01.serial.bin','GREEN_raw',600),
        ('20261026_green_new_pose_400mm_02.serial.bin','GREEN_raw',400)]
    calibration=[]
    for name,kind,distance in references:
        rec=next((r for r in records if r['source']==name),None)
        if rec is None:continue
        line=next(line for line in rec['production_replay'].splitlines() if line.startswith(kind+' '))
        values=dict(token.split('=',1) for token in line.split()[1:])
        current=float(values['range']);foot=float(values['foot']);bearing=float(values['bearing'])
        legacy=24000/(foot-78)/math.cos(math.radians(bearing))
        calibration.append(dict(source=name,actual_mm=distance,current_mm=current,
            current_error_mm=current-distance,legacy_mm=legacy,legacy_error_mm=legacy-distance))
    if len(calibration)==5:
        maximum=max(abs(r['current_error_mm']) for r in calibration)
        legacy=max(abs(r['legacy_error_mm']) for r in calibration)
        print('CALIBRATION measured references',len(calibration),'maximum_error_mm',maximum,'legacy',legacy)
        if maximum>40 or legacy<=140:
            raise SystemExit('Current source fails the recorded reference-range regression; review camera mounting/calibration.')
    (args.output_dir/'report.json').write_text(json.dumps(dict(records=records,wraps=wraps,
        calibration=calibration,calibration_limit='Reported400/600mm field placements; fitted data, not universal or independent accuracy proof.'),indent=2)+'\n')

if __name__=='__main__':main()
