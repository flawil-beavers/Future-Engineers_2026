"""Direct connectors with/without optional-search time: CAD, not hardware proof.

Compare against the preceding construction by disabling only direct connectors
and restoring its 260 mm non-extreme end clearance. Exercise the lap-1 seam,
unmodified learned route, full two-lap passing and final-corner finish gate.
Artifacts and before/after polylines remain under local_workspace.
"""
import json
import shutil
import subprocess

from check_cw_start_planner import ROOT
from check_later_laps import later_lap_fixture


def main():
    dest = ROOT / 'local_workspace/later-lap-direct-lanes'
    dest.mkdir(parents=True, exist_ok=True)
    # Reproduce the retired connector experiment from its committed source.
    source = subprocess.check_output(
        ['git', 'show', 'c19b0b0:src/obstacle_path.cpp'], cwd=ROOT, text=True)
    fixture = later_lap_fixture(source).split('int main()')[0]
    fixture = fixture.replace('OBSTACLE_OPTIMIZED_MIDDLE_CLEARANCE_MM', '190.0f')
    fixture = fixture.replace('uint32_t micros(){return 0;}',
        'uint32_t clockValue=0,clockTick=0;uint32_t micros(){clockValue+=clockTick;return clockValue;}')
    fixture += r'''
int main(){int dir,id=0,colors[24],limited;
 while(std::cin>>dir>>limited){for(int &c:colors)std::cin>>c;
  routeTurnSign=dir;resetState();baseline();clockTick=limited?30000:0;
  for(int i=0;i<24;++i){seats[i].confirmed=seats[i].injected=colors[i]!=0;seats[i].red=colors[i]==2;}
  learned();joinSameColourStraightEnds(livePath,true);roundKnownCornerPairs(livePath,true);
  PathPoint learnedCopy[OBSTACLE_MAX_PATH_WAYPOINTS];memcpy(learnedCopy,livePath,sizeof(learnedCopy));
  float trackedWall,trackedPillar;
  if(!driveTwoLaps(0,trackedWall,trackedPillar) || trackedWall<40 || trackedPillar<30)return 2;
  if(memcmp(learnedCopy,livePath,sizeof(PathPoint)*pathLength))return 3;
  // No altered lookahead target at the 1->2 seam, including both sides of it.
  for(int i=0;i<pathLength;++i){float d=fminf(baselinePath[i].distanceMm,loopLengthMm-baselinePath[i].distanceMm);
   if(d<=OBSTACLE_LATER_LAP_SEAM_KEEP_MM && hypotf(livePath[i].x-optimizedPath[i].x,livePath[i].y-optimizedPath[i].y)>.001f)return 4;
  }
  float length,peak,turn,wall,pillar;
  float bend=laterLapBending(optimizedPath,length,peak,&turn),seconds=laterLapTargetTime(optimizedPath);
  if(!laterLapRouteSafe(optimizedPath,&wall,&pillar))return 5;
  // Independently exercise the connector's protected final-corner region.
  PathPoint before[OBSTACLE_MAX_PATH_WAYPOINTS],scratch[OBSTACLE_MAX_PATH_WAYPOINTS];memcpy(before,optimizedPath,sizeof(before));
  connectKnownLaterLapRoute(optimizedPath,scratch);
  for(int i=0;i<pathLength;++i)if(baselinePath[i].distanceMm>=corners[3].pathStartMm-OBSTACLE_LATER_LAP_LOOKAHEAD_MM ||
       baselinePath[i].distanceMm<=OBSTACLE_LATER_LAP_SEAM_KEEP_MM+OBSTACLE_LATER_LAP_SEAM_BLEND_MM)
    if(optimizedPath[i].x!=before[i].x || optimizedPath[i].y!=before[i].y)return 6;
  memcpy(optimizedPath,before,sizeof(before));
  std::cout<<"DIRECT "<<id<<" "<<length<<" "<<seconds<<" "<<bend<<" "<<peak<<" "<<turn<<" "<<wall<<" "<<pillar<<" "<<trackedWall<<" "<<trackedPillar<<"\n";
  for(int i=0;i<pathLength;++i)std::cout<<"POINT "<<id<<" "<<optimizedPath[i].x<<" "<<optimizedPath[i].y<<"\n";
  ++id;
 }
}
'''
    layouts = [((0,1),(4,2),(7,1),(10,2),(15,1),(19,2),(22,1)),
               ((4,2),(7,1),(11,1),(17,2),(22,1)),
               ((11,1),(14,2)), ((2,1),(6,2),(11,1),(12,1),(17,2),(20,2))]
    cases = []
    for name, items in zip(('B', 'C', 'outer_to_middle', 'A'), layouts):
        cw = [0]*24
        for seat, color in items:
            cw[seat] = color
        ccw = [0]*24
        for i, color in enumerate(cw):
            ccw[((4-i//6)%4)*6+(2-(i%6)//2)*2+((i%2)^1)] = color
        for direction, colors in ((-1, cw), (1, ccw)):
            for limited in (0, 1):
                cases.append(dict(name=name, direction=direction, limited=limited, colors=colors))
    inputs = '\n'.join(' '.join(map(str,[c['direction'],c['limited'],*c['colors']])) for c in cases)+'\n'
    results = {}
    for name in ('before', 'after'):
        code = fixture
        if name == 'before':
            code = code.replace('        connectKnownLaterLapRoute(optimizedPath,beforeLaneCarry);', '')
            code = code.replace('OBSTACLE_OPTIMIZED_END_CLEARANCE_MM', 'OBSTACLE_LAP1_CLEARANCE_MM')
        cpp, exe = dest/f'{name}.cpp', dest/f'{name}.exe'
        cpp.write_text(code)
        build = subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'include'),
            '-I', str(ROOT/'local_workspace/later-laps'), str(cpp), '-o', str(exe)], capture_output=True, text=True)
        (dest/f'{name}_build.txt').write_text(build.stdout+build.stderr)
        if build.returncode:
            raise RuntimeError(build.stderr)
        run = subprocess.run([str(exe)], input=inputs, capture_output=True, text=True)
        (dest/f'{name}_output.txt').write_text(run.stdout+run.stderr)
        assert run.returncode == 0, (name, run.returncode, run.stdout[-2000:])
        rows, points = [], [[] for _ in cases]
        for line in run.stdout.splitlines():
            f = line.split()
            if f[0] == 'DIRECT':
                rows.append(dict(zip(('length_mm','target_s','bending','peak','turn','wall_mm',
                    'pillar_mm','tracked_wall_mm','tracked_pillar_mm'),map(float,f[2:]))))
            elif f[0] == 'POINT':
                points[int(f[1])].append(list(map(float,f[2:])))
        assert len(rows) == len(cases)
        results[name] = dict(metrics=rows, points=points)
    for i,c in enumerate(cases):
        old,new = (results[n]['metrics'][i] for n in ('before','after'))
        print(c['name'], c['direction'], 'budget_exhausted',c['limited'],
              'saved_mm',round(old['length_mm']-new['length_mm'],2),
              'target_s_saved',round(old['target_s']-new['target_s'],3),
              'tracked_wall_pillar_mm',new['tracked_wall_mm'],new['tracked_pillar_mm'])
        if c['name']=='B':
            assert new['length_mm'] < old['length_mm']-15
            assert new['target_s'] < old['target_s']-.05
    (dest/'comparison.json').write_text(json.dumps(dict(cases=cases,**results),indent=2)+'\n')
    print('DIRECT LANES: 16 before/after maps PASS, first-lap/seam/final-corner preserved')


if __name__ == '__main__':
    main()
