"""Measure production route length and discrete bending; no hardware claims.

Run check_later_laps.py first. --baseline disables only the optional refinement
in the extracted current production fixture, keeping all prior planning stages.
Working fixtures/results stay under local_workspace/later-laps.
"""
import argparse
import json
import shutil
import subprocess
from check_cw_start_planner import ROOT


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', action='store_true')
    args = parser.parse_args()
    dest = ROOT/'local_workspace/later-laps'
    fixture = (dest/'check.cpp').read_text().split('int main()')[0]
    if args.baseline:
        call='smoothKnownLaterLapRoute(optimizedPath,beforeLaneCarry,routeWall,routePillar);'
        assert call in fixture
        fixture=fixture.replace(call,'(void)0;')
    fixture += r'''
int main(){int direction,id=0,colors[24];float cap;
 while(std::cin>>direction>>cap){
  for(int &c:colors)if(!(std::cin>>c))return 3;
  routeTurnSign=direction;resetState();baseline();
  for(int i=0;i<24;++i){seats[i].confirmed=colors[i]!=0;
   seats[i].red=colors[i]==2;seats[i].injected=colors[i]!=0;}
  learned();joinSameColourStraightEnds(livePath,true);roundKnownCornerPairs(livePath,true);
  if(!buildOptimizedPath())return 4;
  float length=0,bending=0,peak=0;
  for(int i=0;i<pathLength;++i){
   int p=(i+pathLength-1)%pathLength,n=(i+1)%pathLength;
   float a=hypotf(optimizedPath[i].x-optimizedPath[p].x,optimizedPath[i].y-optimizedPath[p].y);
   float b=hypotf(optimizedPath[n].x-optimizedPath[i].x,optimizedPath[n].y-optimizedPath[i].y);
   length+=b;if(a<1||b<1)continue;
   float h1=atan2f(optimizedPath[i].y-optimizedPath[p].y,optimizedPath[i].x-optimizedPath[p].x);
   float h2=atan2f(optimizedPath[n].y-optimizedPath[i].y,optimizedPath[n].x-optimizedPath[i].x);
   float turn=wrap180((h2-h1)*180/PI)*PI/180;
   float ds=(a+b)*.5f;bending+=turn*turn/ds;peak=fmaxf(peak,fabsf(turn)/ds);
  }
  std::cout<<"METRIC "<<id++<<" "<<length<<" "<<bending<<" "<<peak<<"\n";
 }
}
'''
    mode='baseline' if args.baseline else 'smoothed'
    cpp=dest/f'measure_{mode}.cpp';exe=dest/f'measure_{mode}.exe'
    cpp.write_text(fixture)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),'-I',str(dest),str(cpp),'-o',str(exe)],check=True)
    cases=json.loads((dest/'report.json').read_text())['cases']
    inputs='\n'.join(' '.join(map(str,[c['direction'],c['speed_cap'],*c['colors']])) for c in cases)+'\n'
    run=subprocess.run([str(exe)],input=inputs,text=True,capture_output=True,check=True)
    metrics=[]
    for line in run.stdout.splitlines():
        if line.startswith('METRIC '):
            _,index,length,bending,peak=line.split()
            metrics.append(dict(case=int(index),length_mm=float(length),bending=float(bending),peak_curvature=float(peak)))
    (dest/f'{mode}_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    print(mode,len(metrics),'mean length',sum(m['length_mm'] for m in metrics)/len(metrics))


if __name__=='__main__':
    main()
