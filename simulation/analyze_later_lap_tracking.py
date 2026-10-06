"""Read bounded later tracking and replay capsule clearances (onboard pose only)."""
from pathlib import Path
import re,json,subprocess,shutil
from check_cw_start_planner import ROOT,fixture_source
p=ROOT/'local_workspace/later-tracking-471-476';p.mkdir(parents=True,exist_ok=True)
f=fixture_source().split('int main()')[0]+r'''
int main(){float x,y,h;int direction;
 while(std::cin>>direction>>x>>y>>h){
 routeTurnSign=direction;baseline();ObstacleClearanceSample c{};
 if(!calculateClearanceAtPose(seats[0],x,y,h,c))return 1;
 std::cout<<c.wallMm<<" "<<static_cast<int>(c.wallFeature)<<"\n";
 }
}
'''
(p/'check.cpp').write_text(f);exe=p/'check.exe'
subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),str(p/'check.cpp'),'-o',str(exe)],check=True)
rows=[]
for n in range(471,477):
 direction=-1 if n<475 else 1
 src=ROOT/'simulation/evidence/parking_exit_diagnostics'/f'20261006_log_{n}_{"cw" if direction<0 else "ccw"}.txt'
 s=src.read_text(errors='replace');samples=[]
 for line in s.splitlines():
  if not line.startswith('[LATER_TRACK]'):continue
  m=re.search(r't=(\d+) lap=(\d+) idx=(\d+) pose=([^ ]+) target=([^ ]+) speed/servo=([^ ]+) tof_used=([^ ]+) correction=([^ ]+)',line)
  if not m:raise ValueError(line)
  t,lap,idx,pose,target,cmd,used,correction=m.groups();samples.append(dict(t=int(t),lap=int(lap),idx=int(idx),pose=list(map(float,pose.split(','))),target=list(map(float,target.split(','))),cmd=list(map(float,cmd.split('/'))),used=used,correction=list(map(float,correction.split(',')))))
 if samples:
  inputs='\n'.join(' '.join(map(str,[direction,*r['pose']])) for r in samples)+'\n'
  output=subprocess.check_output([str(exe)],input=inputs,text=True).splitlines()
  for r,l in zip(samples,output):r['modeled_wall_mm']=float(l.split()[0])
 laps={}
 for lap in (2,3):
  a=[r for r in samples if r['lap']==lap]
  if not a:continue
  east=[r for r in a if r['pose'][0]>500 and abs(r['pose'][1])<400]
  laps[lap]=dict(samples=len(a),first_ms=a[0]['t'],last_ms=a[-1]['t'],sampled_wall_min_mm=min(r['modeled_wall_mm'] for r in a),sampled_east_wall_min_mm=min((r['modeled_wall_mm'] for r in east),default=None),eastern_middle_min_x=min((r['pose'][0] for r in east),default=None))
 rows.append(dict(log=n,direction=direction,laps=laps,samples=samples));print(n,laps)
(p/'report.json').write_text(json.dumps(dict(runs=rows,limitations=['250ms samples may miss closest point; poses are onboard estimates, not independent clearance measurements']),indent=2)+'\n')
