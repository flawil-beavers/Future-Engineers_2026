"""Offline candidate search: straight reverse from logged first-curve hold poses.

This does not command the robot. It checks field walls and all 24 legal seats
with the front/rear capsule union and 40 mm margin, including 20 mm extra reverse
braking travel. The view remains model-derived. Dynamic yaw, camera images and
actual retrace are not simulated.
"""
import argparse
import itertools
import json
import math
from pathlib import Path
from corner_forward_view_search import margins


def hold_poses(path):
    for line in path.read_text().splitlines():
        if line.startswith('[DISCOVERY_TRACE]') and 'reason=hold_start ' in line:
            fields = dict(token.split('=', 1) for token in line.split() if '=' in token)
            yield tuple(map(float, fields['pose'].split(',')))


def reverse_pose(pose, distance):
    x, y, h = pose
    angle = math.radians(h)
    return x-distance*math.cos(angle), y-distance*math.sin(angle), h


def visibility(pose, seat):
    x, y, h = pose
    angle = math.radians(h)
    dx = seat[0] - (x+125*math.cos(angle))
    dy = seat[1] - (y+125*math.sin(angle))
    bearing = (math.degrees(math.atan2(dy, dx))-h+180)%360-180
    distance = math.hypot(dx, dy)
    return 230 <= distance <= 600 and abs(bearing) <= 26.426


def evaluate(pose, distance, shift):
    perturbed = tuple(value+delta for value, delta in zip(pose, shift))
    seats = ((-900,-500),(-1100,-500))
    minimum_wall = minimum_pillar = math.inf
    for traveled in range(0, int(distance)+21, 2):
        at = reverse_pose(perturbed, traveled)
        wall, pillar = margins(at)
        minimum_wall = min(minimum_wall, wall)
        minimum_pillar = min(minimum_pillar, pillar)
    view = reverse_pose(perturbed, distance)
    return dict(passed=all(visibility(view, seat) for seat in seats) and
                min(minimum_wall, minimum_pillar) > 40,
                wall_mm=minimum_wall,pillar_mm=minimum_pillar)


def search(path):
    poses = list(hold_poses(path))
    if not poses:
        raise ValueError('No logged hold-start pose')
    result = []
    for distance in range(80, 221, 10):
        cases = [evaluate(pose, distance, shift) for pose in poses
                 for shift in itertools.product((-10,0,10),(-10,0,10),(-2,0,2))]
        result.append(dict(reverse_mm=distance, cases=len(cases),
                           failures=sum(not case['passed'] for case in cases),
                           min_wall_mm=min(case['wall_mm'] for case in cases),
                           min_pillar_mm=min(case['pillar_mm'] for case in cases)))
    return dict(log=path.name, candidates=result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    args = parser.parse_args()
    reports = [search(path) for path in args.logs]
    destination = Path('local_workspace/corner-view-search/report.json')
    destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_text(json.dumps(reports,indent=2))
    for report in reports:
        print(report['log'], [r for r in report['candidates'] if r['failures']==0])
    return 0


if __name__=='__main__':
    raise SystemExit(main())
