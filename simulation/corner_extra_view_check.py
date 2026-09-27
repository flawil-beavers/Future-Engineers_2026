"""Check the additional reverse arc against recorded scan poses.

Geometric checks only: camera segmentation, steering transients and firmware
state transitions are not simulated. The later red seat is an offline scenario
from the reported setup, never knowledge supplied to the controller.
"""
import hashlib
import itertools
import json
import math
from pathlib import Path

from corner_forward_view_search import advance, margins, visible
from parking_entry_scout_sim import camera_geometry, wrap180


def scan_poses(path):
    for line in path.read_text().splitlines():
        if not line.startswith('[CORNER VIEW POSE]'):
            continue
        fields = dict(token.split('=', 1) for token in line.split() if '=' in token)
        if fields.get('phase') == 'scan_return_start' and fields.get('station') == '3':
            yield tuple(map(float, fields['pose'].split(',')))


def check(path):
    results = []
    for pose in scan_poses(path):
        for shift in itertools.product((-10, 0, 10), (-10, 0, 10), (-2, 0, 2)):
            start = tuple(a+b for a, b in zip(pose, shift))
            for gain in (.85, 1, 1.15):
                # Sweep includes the full 20 mm braking reserve.
                sweep = [advance(start, -20, -d, gain) for d in range(111)]
                wall = min(margins(at)[0] for at in sweep)
                pillar = min(margins(at)[1] for at in sweep)
                for coast in (0, 10, 20):
                    end = advance(start, -20, -90-coast, gain)
                    near = camera_geometry(end, (-900, -500))[0]
                    far = camera_geometry(end, (-1100, 500))[0]
                    back = advance(end, -20, 90+coast, gain)
                    results.append(dict(wall_mm=wall, pillar_mm=pillar,
                        visible=visible(end, (-900, -500)),
                        separation_deg=abs(wrap180(near-far)),
                        retrace_xy_mm=math.hypot(back[0]-start[0], back[1]-start[1]),
                        retrace_heading_deg=abs(wrap180(back[2]-start[2]))))
    if not results:
        raise ValueError('No complete first-corner scan poses found')
    return dict(log=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        scan_poses=len(list(scan_poses(path))), cases=len(results),
        minimum_wall_mm=min(r['wall_mm'] for r in results),
        minimum_pillar_mm=min(r['pillar_mm'] for r in results),
        view_failures=sum(not r['visible'] for r in results),
        clearance_failures=sum(min(r['wall_mm'], r['pillar_mm']) <= 40 for r in results),
        minimum_ray_separation_deg=min(r['separation_deg'] for r in results),
        maximum_ideal_retrace_xy_mm=max(r['retrace_xy_mm'] for r in results),
        maximum_ideal_retrace_heading_deg=max(r['retrace_heading_deg'] for r in results),
        limitations='24 occupied seats; model capsules; XY +/-10mm, heading +/-2deg; '
            'constant yaw gain .85/1/1.15 in both directions; coast0/10/20mm; '
            '1mm sweep; no colour evidence, actuator transient, slip or state-machine acceptance')


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    args = parser.parse_args()
    print(json.dumps(check(args.log), indent=2))
