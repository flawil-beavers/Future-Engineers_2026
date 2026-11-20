"""Audit laps2/3 with measured123mm wheel/body polygons and50mm signs.
Run check_later_laps.py first for its portable known-map case matrix.
Pose/map/servo/traction errors remain unmeasured. Generates only local_workspace.
"""
import json, shutil, subprocess, sys
from pathlib import Path
from check_later_laps import later_lap_fixture
ROOT=Path(__file__).resolve().parents[1]
out=ROOT/'local_workspace/later-lap-clearance-50'
out.mkdir(exist_ok=True)
cases=json.loads((ROOT/'local_workspace/later-laps/report.json').read_text())['cases']
s=later_lap_fixture()
negative='--without-reserve' in sys.argv
if negative:
    cases=sorted([c for c in cases if c['colors'][2] or c['colors'][3]],
                 key=lambda c:c['minimum_pillar_mm'])[:24]
    s=s.replace('        addLaterLapStartMiddleReserve(optimizedPath,beforeLaneCarry);','')
name='physical_without_reserve' if negative else 'physical_current'
helper=r'''
float physicalPillar=1e9f,physicalInner=1e9f;
int physicalSeat=-1;
float quadGap(const parking_start_footprint::Quad &a,const parking_start_footprint::Quad &b){
 if(parking_start_footprint::overlaps(a,b))return 0;
 float best=1e9f;
 for(int i=0;i<4;++i)for(int j=0;j<4;++j){
 auto u=a.p[i],v=b.p[j],un=a.p[(i+1)%4],vn=b.p[(j+1)%4];
 best=fminf(best,pointSegmentDistance(u.x,u.y,v.x,v.y,vn.x,vn.y));
 best=fminf(best,pointSegmentDistance(v.x,v.y,u.x,u.y,un.x,un.y));}return best;
}
void physicalAudit(float x,float y,float h,int seat){
 using namespace parking_start_footprint;
 const Quad pillar=seat>=0?rectangle(seats[seat].x,seats[seat].y,25,25,0):Quad{};
 const Quad inner=rectangle(0,0,FIELD_INNER_HALF_MM,FIELD_INNER_HALF_MM,0);
 for(int sign:{-1,0,1}){Quad bodies[6];robotQuads(x,y,h,sign,0,bodies,123,35,98,123,135);
 for(const auto &body:bodies){
 if(seat>=0){float gap=quadGap(body,pillar);if(gap<physicalPillar){physicalPillar=gap;physicalSeat=seat;}}
 physicalInner=fminf(physicalInner,quadGap(body,inner));}}
}
'''
at=s.index('bool driveTwoLaps(');s=s[:at]+helper+s[at:]
s=s.replace('minimumPillar=fminf(minimumPillar,sample.pillarMm);',
 'minimumPillar=fminf(minimumPillar,sample.pillarMm);\n   if(seats[seat].confirmed && sample.pillarMm<100)physicalAudit(x,y,heading*180/PI,seat);\n   if(seat==0 && sample.wallMm<100)physicalAudit(x,y,heading*180/PI,-1);')
s=s.replace('learned();joinSameColourStraightEnds(livePath,true);float wall,pillar;',
 'learned();joinSameColourStraightEnds(livePath,true);float wall,pillar;physicalPillar=physicalInner=1e9f;physicalSeat=-1;')
s=s.replace('std::cout<<"CASE "<<id++',
 'std::cout<<"PHYSICAL "<<id<<" "<<physicalPillar<<" "<<physicalInner<<" "<<physicalSeat<<"\\n";\n  std::cout<<"CASE "<<id++')
# Bound the difference to the existing capsule. A100mm capsule clearance
# then implies at least50mm to these polygons, so far objects need no SAT work.
s=s.replace('if(!stateChecks())',r'''
 for(int sign:{-1,0,1}){parking_start_footprint::Quad bodies[6];
 parking_start_footprint::robotQuads(0,0,0,sign,0,bodies,123,35,98,123,135);
 for(const auto &b:bodies)for(const auto &p:b.p)
 if(pointSegmentDistance(p.x,p.y,0,0,OBSTACLE_ROBOT_ENVELOPE_AXIS_FRONT_MM,0)-OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM>=50)return 9;}
 if(!stateChecks())''')
cpp=out/(name+'.cpp');exe=out/(name+'.exe');cpp.write_text(s)
subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),'-I',str(ROOT/'local_workspace/later-laps'),str(cpp),'-o',str(exe)],check=True)
inputs='\n'.join(' '.join(map(str,[c['direction'],c['speed_cap'],*c['colors']])) for c in cases)+'\n'
r=subprocess.run([str(exe)],input=inputs,text=True,capture_output=True,check=True)
(out/(name+'_output.txt')).write_text(r.stdout)
results=[]
for line in r.stdout.splitlines():
 if not line.startswith('PHYSICAL '):continue
 _,i,pillar,inner,seat=line.split();results.append(dict(case=int(i),pillar_mm=float(pillar),inner_mm=float(inner),seat=int(seat)))
bad=[r for r in results if r['pillar_mm']<50 or r['inner_mm']<50]
report=dict(total=len(results),failed=len(bad),minimum_pillar_mm=min(r['pillar_mm'] for r in results),minimum_inner_mm=min(r['inner_mm'] for r in results),failures=bad)
(out/(name+'_report.json')).write_text(json.dumps(report,indent=2)+'\n')
print('PHYSICAL CLEARANCE',name,{k:v for k,v in report.items() if k!='failures'})
assert len(results)==len(cases)
if negative:
    assert bad, 'negative control did not reproduce the previous small clearance'
    raise SystemExit(0)
raise SystemExit(bool(bad))
