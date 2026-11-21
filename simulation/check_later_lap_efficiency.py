"""Compare selected route time using the final physical speed table.
Known-layout CAD planner, unlimited host planning budget; not measured lap time.
Run check_later_laps.py first. All generated data stays below local_workspace.
"""
import json, shutil, subprocess
from pathlib import Path
from check_later_laps import later_lap_fixture
from measure_later_lap_dynamics import METRICS
ROOT=Path(__file__).resolve().parents[1]
out=ROOT/'local_workspace/later-lap-efficiency-20261120'
out.mkdir(exist_ok=True)
cases=json.loads((ROOT/'local_workspace/later-laps/report.json').read_text())['cases']
cases += [dict(direction=d,speed_cap=0,colors=[0]*24) for d in (-1,1)]
inputs='\n'.join(' '.join(map(str,[c['direction'],c['speed_cap'],*c['colors']])) for c in cases)+'\n'
# Reproduce the optional inner-turn experiment, which is no longer active.
source=subprocess.check_output(['git','show','7870340:src/obstacle_path.cpp'],cwd=ROOT,text=True)
fixture=later_lap_fixture(source).split('int main()')[0]
fixture=fixture.replace('OBSTACLE_OPTIMIZED_MIDDLE_CLEARANCE_MM','190.0f')
metrics=METRICS.replace('@@SPEED@@','laterLapSpeedLimits[i]').replace('@@NEXT_SPEED@@','laterLapSpeedLimits[n]')
results={}
for mode in ('before','after'):
    s=fixture
    if mode=='before':
        assert 'float maximumExtraTurn=0.20f;' in s
        s=s.replace('float maximumExtraTurn=0.20f;','float maximumExtraTurn=0.01f;')
    cpp=out/(mode+'_current_metric.cpp');exe=out/(mode+'_current_metric.exe');cpp.write_text(s+metrics)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),'-I',str(ROOT/'local_workspace/later-laps'),str(cpp),'-o',str(exe)],check=True)
    r=subprocess.run([str(exe)],input=inputs,text=True,capture_output=True,check=True)
    (out/(mode+'_current_metrics.txt')).write_text(r.stdout)
    rows=[list(map(float,line.split()[1:])) for line in r.stdout.splitlines() if line.startswith('METRIC ')]
    assert len(rows)==len(cases)
    results[mode]=rows
comparison=[]
for i,(a,b) in enumerate(zip(results['before'],results['after'])):
    comparison.append(dict(case=i,direction=cases[i]['direction'],cap=cases[i]['speed_cap'],saved_mm=a[1]-b[1],saved_target_s=a[4]-b[4]))
slower=[c for c in comparison if c['saved_target_s']<-.001]
faster=[c for c in comparison if c['saved_target_s']>.001]
report=dict(total=len(cases),faster=len(faster),slower=len(slower),maximum_saved_target_s=max(c['saved_target_s'] for c in comparison),changed=faster+slower)
(out/'efficiency_current_report.json').write_text(json.dumps(report,indent=2)+'\n')
print('EFFICIENCY',report)
assert not slower
assert len(faster)>=8
