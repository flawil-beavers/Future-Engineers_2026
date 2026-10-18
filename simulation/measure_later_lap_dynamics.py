"""Compare production route/target-time budgets with a saved earlier fixture.

Run check_later_laps.py first. Reference fixture/config arguments can be saved
from the earlier checkout under local_workspace. No hardware timing claims.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

from check_cw_start_planner import ROOT


METRICS = r'''
int main(){int direction,id=0,colors[24];float cap;
 while(std::cin>>direction>>cap){
  for(int &c:colors)if(!(std::cin>>c))return 3;
  routeTurnSign=direction;resetState();baseline();
  for(int i=0;i<24;++i){seats[i].confirmed=colors[i]!=0;
   seats[i].red=colors[i]==2;seats[i].injected=colors[i]!=0;}
  learned();joinSameColourStraightEnds(livePath,true);roundKnownCornerPairs(livePath,true);
  if(!buildOptimizedPath())return 4;
  float length,bending,peak,time=0,minSpeed=1e9f,maxSpeed=0,wall,pillar;
  bending=laterLapBending(optimizedPath,length,peak);
  if(!laterLapRouteSafe(optimizedPath,&wall,&pillar))return 5;
  for(int i=0;i<pathLength;++i){int n=(i+1)%pathLength;
   float v=@@SPEED@@,vn=@@NEXT_SPEED@@;
   if(cap>0){v=fminf(v,cap);vn=fminf(vn,cap);}
   minSpeed=fminf(minSpeed,v);maxSpeed=fmaxf(maxSpeed,v);
   time+=2*hypotf(optimizedPath[n].x-optimizedPath[i].x,
                 optimizedPath[n].y-optimizedPath[i].y)/fmaxf(1,v+vn);
  }
  std::cout<<"METRIC "<<id++<<" "<<length<<" "<<bending<<" "<<peak
   <<" "<<time<<" "<<minSpeed<<" "<<maxSpeed<<" "<<wall<<" "<<pillar<<"\n";
 }
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference-fixture', type=Path, required=True)
    parser.add_argument('--reference-config', type=Path, required=True)
    parser.add_argument('--examples', action='store_true', help='Only the seven A/B/C cases')
    args = parser.parse_args()
    dest = ROOT/'local_workspace/later-lap-optimization'
    dest.mkdir(parents=True, exist_ok=True)
    current = ROOT/'local_workspace/later-laps'
    cases = json.loads((current/'report.json').read_text())['cases']
    if args.examples:
        cases = cases[-7:]
    inputs = '\n'.join(' '.join(map(str, [c['direction'], c['speed_cap'], *c['colors']]))
                       for c in cases)+'\n'
    (dest/'config_reference.h').write_text(args.reference_config.read_text())
    outputs = {}
    for mode, fixture_path in [('before', args.reference_fixture), ('after', current/'check.cpp')]:
        fixture = fixture_path.read_text().split('int main()')[0]
        if mode == 'before':
            fixture = fixture.replace('#include "config.h"', '#include "config_reference.h"')
            speed = 'ceilf(optimizedPath[i].speedMmS*OBSTACLE_LATER_LAP_SPEED_FACTOR)'
            next_speed = speed.replace('[i]', '[n]')
        else:
            speed, next_speed = 'laterLapSpeedLimits[i]', 'laterLapSpeedLimits[n]'
        fixture += METRICS.replace('@@SPEED@@', speed).replace('@@NEXT_SPEED@@', next_speed)
        cpp, exe = dest/f'metrics_{mode}.cpp', dest/f'metrics_{mode}.exe'
        cpp.write_text(fixture)
        subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'include'),
                        '-I', str(current), str(cpp), '-o', str(exe)], check=True)
        run = subprocess.run([str(exe)], input=inputs, text=True, capture_output=True, check=True)
        metrics = []
        for line in run.stdout.splitlines():
            if line.startswith('METRIC '):
                fields = line.split()
                names = ('length_mm', 'bending', 'peak_curvature', 'target_time_s',
                         'minimum_target_mms', 'maximum_target_mms', 'wall_mm', 'pillar_mm')
                metrics.append(dict(case=int(fields[1]), **dict(zip(names, map(float, fields[2:])))))
        assert len(metrics) == len(cases)
        outputs[mode] = metrics
    comparison = []
    for c, before, after in zip(cases, outputs['before'], outputs['after']):
        comparison.append(dict(case=before['case'], direction=c['direction'], speed_cap=c['speed_cap'],
            saved_mm=before['length_mm']-after['length_mm'],
            target_time_reduction_pct=100*(1-after['target_time_s']/before['target_time_s']),
            bending_reduction_pct=100*(1-after['bending']/before['bending']),
            before=before, after=after))
    (dest/'comparison.json').write_text(json.dumps(comparison, indent=2)+'\n')
    uncapped = [c for c in comparison if c['speed_cap'] == 0]
    print('Cases', len(comparison), 'uncapped', len(uncapped))
    for key in ('saved_mm', 'target_time_reduction_pct', 'bending_reduction_pct'):
        values = [c[key] for c in uncapped]
        print(key, 'min/mean/max', min(values), sum(values)/len(values), max(values))
    for c in comparison[-7:]:
        print('A/B/C', c['case'], c['direction'], 'saved_mm', c['saved_mm'],
              'target_time_reduction_pct', c['target_time_reduction_pct'],
              'before_s', c['before']['target_time_s'], 'after_s', c['after']['target_time_s'])


if __name__ == '__main__':
    main()
