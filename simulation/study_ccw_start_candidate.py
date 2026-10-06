"""Replay the original translated-pose CCW design inputs against the planner.

Run review_ccw_start.py first. Its reproducible generated fixture and report
are inputs. The approximation moves the old primary pose 70 mm east and defers
both behind seats. Production CCW guards remain enabled. For the implemented
complete scan and later return use check_ccw_start_planner.py instead.
"""
import json
import shutil
import subprocess

from review_ccw_start import ROOT
from parking_entry_scout_sim import camera_geometry
from parking_exit_swept_search import Pose, collision, robot_polygons


def main():
    destination = ROOT / 'local_workspace/ccw-start-review'
    source = (destination / 'audit.cpp').read_text()
    source = source.replace('bool parkingCcwShortStart=false;',
                            'bool parkingCcwShortStart=true;')
    source = source.replace(
        '    displaceForSeat(livePath,2*s+1,OBSTACLE_LAP1_CLEARANCE_MM,',
        '    if(s==2) displaceForSeat(livePath,2*s+1,OBSTACLE_LAP1_CLEARANCE_MM,')
    source = source.replace('merge,front==1)', 'merge,true)')
    cpp = destination / 'candidate.cpp'
    cpp.write_text(source)
    compiler = shutil.which('g++') or shutil.which('clang++')
    if not compiler:
        raise RuntimeError('Host C++ compiler on PATH is required')
    exe = destination / 'candidate.exe'
    subprocess.run([compiler, '-std=c++17', '-O2', '-I', str(ROOT/'include'),
                    str(cpp), '-o', str(exe)], check=True)
    original = json.loads((destination/'report.json').read_text())['cases']
    inputs = '\n'.join(' '.join(map(str, [
        c['pose'][0]+70+c['perturbation'][0],
        c['pose'][1]+c['perturbation'][1],
        c['pose'][2]+c['perturbation'][2], *c['layout']])) for c in original)+'\n'
    run = subprocess.run([str(exe)], input=inputs, text=True,
                         capture_output=True, check=True)
    (destination/'candidate-rollout.txt').write_text(run.stdout)
    cases = []
    for line in run.stdout.splitlines():
        if line.startswith('CASE '):
            _, number, passed, merge, lookahead = line.split()
            c = original[int(number)]
            cases.append(dict(source=c['source'], pose=c['pose'],
                              perturbation=c['perturbation'], layout=c['layout'],
                              passed=bool(int(passed)), merge=int(merge),
                              lookahead_mm=float(lookahead), trace=[]))
        elif line.startswith('POSE '):
            cases[-1]['trace'].append(list(map(float, line.split()[1:])))
    minimum_x = float('inf')
    pink_conflicts = middle_conflicts = 0
    for c in cases:
        pink = middle = False
        for x, y, h in c['trace']:
            pose = Pose(480-x, y+1500, 180-h)
            for steer in (-50, 50):
                body_x = min(480-px for poly in robot_polygons(pose, steer)
                             for px, _ in poly)
                minimum_x = min(minimum_x, body_x)
                middle |= body_x <= 0
                pink |= any(collision(pose, steer, gap) for gap in (242.5, 252.5))
        pink_conflicts += pink
        middle_conflicts += middle
    matrix = []
    for layout in sorted({tuple(c['layout']) for c in cases}):
        group = [c for c in cases if tuple(c['layout']) == layout]
        matrix.append(dict(layout=layout, passed=sum(c['passed'] for c in group),
                           total=len(group)))
    views = [dict(source=c['source'], bearing_range=camera_geometry(
        (c['pose'][0]+70, c['pose'][1], c['pose'][2]), (500, -900)))
        for c in cases if c['layout'] == [0, 0, 1] and c['perturbation'] == [0, 0, 0]]
    footprint = dict(minimum_body_x_mm=minimum_x, pink_conflicts=pink_conflicts,
                     behind_middle_conflicts=middle_conflicts)
    report = dict(matrix=matrix, footprint=footprint, forward_view=views,
                  cases=cases, limitations=[
                      'Translated old primary poses, not a full scan check',
                      '70 mm translation approximates an earlier localization stop',
                      'Full exit/localization/braking prefix and later return not tested',
                      'Known colours supplied; no image recognition acceptance'])
    (destination/'candidate.json').write_text(json.dumps(report, indent=2)+'\n')
    print(matrix)
    print(footprint)


if __name__ == '__main__':
    main()
