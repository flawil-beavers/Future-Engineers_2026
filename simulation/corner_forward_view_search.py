"""Search forward-only first-corner view arcs from recorded CW approach poses.

This is a geometric candidate search, not firmware or physical acceptance.
Every legal pillar seat remains occupied for collision checking. Camera colour
evidence, controller tracking and a subsequent route join are not simulated.
"""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path

from parking_entry_scout_sim import (camera_geometry, point_segment_distance,
                                    segment_distance, WALLS, wrap180)

TARGETS = ((-900.0, -500.0), (-1100.0, -500.0))
ALL_SEATS = tuple((along, side) for along in (-500, 0, 500)
                  for side in (-1100, -900, 900, 1100)) + tuple(
                      (side, along) for along in (-500, 0, 500)
                      for side in (-1100, -900, 900, 1100))


def advance(pose, steering_deg, distance, yaw_gain=1.0):
    x, y, heading_deg = pose
    angle = math.radians(heading_deg)
    curvature = -math.tan(math.radians(steering_deg)) / 100.0 * yaw_gain
    delta = curvature * distance
    if abs(curvature) < 1e-9:
        return x + distance * math.cos(angle), y + distance * math.sin(angle), heading_deg
    return (x + (math.sin(angle + delta) - math.sin(angle)) / curvature,
            y + (math.cos(angle) - math.cos(angle + delta)) / curvature,
            wrap180(heading_deg + math.degrees(delta)))


def margins(pose):
    x, y, h = pose
    c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
    # Union of front capsule and the existing conservative rear-shifted capsule.
    axis = (x - 70*c, y - 70*s, x + 60*c, y + 60*s)
    return (min(segment_distance(axis, wall) for wall in WALLS) - 70,
            min(point_segment_distance(*seat, *axis) for seat in ALL_SEATS) - 112.5)


def visible(pose, seat):
    bearing, distance = camera_geometry(pose, seat)
    return 230 <= distance <= 600 and abs(bearing) <= 26.426


def approach_poses(path):
    for line in path.read_text().splitlines():
        if not line.startswith('[DISCOVERY_TRACE]'):
            continue
        fields = dict(token.split('=', 1) for token in line.split() if '=' in token)
        if fields.get('station') == '3' and fields.get('reason') == 'approach':
            yield int(fields['t']), tuple(map(float, fields['pose'].split(',')))


def evaluate(pose, steering, travel, shift=(0, 0, 0), gain=1.0, coast=0):
    start = tuple(a + b for a, b in zip(pose, shift))
    wall = pillar = math.inf
    end_travel = travel + coast
    # <=5 mm spacing, including the final braking endpoint.
    for distance in list(range(0, int(end_travel), 5)) + [end_travel]:
        at = advance(start, steering, distance, gain)
        w, p = margins(at)
        wall, pillar = min(wall, w), min(pillar, p)
        if min(wall, pillar) <= 25:
            return dict(passed=False, reason='clearance', wall_mm=wall, pillar_mm=pillar)
    end = advance(start, steering, end_travel, gain)
    return dict(passed=all(visible(end, seat) for seat in TARGETS),
                reason='view', wall_mm=wall, pillar_mm=pillar, endpoint=end)


def search(path):
    candidates = []
    for time, pose in approach_poses(path):
        for steering in range(4, 43, 2):
            for travel in range(40, 301, 10):
                # Reject unusable endpoints before the more expensive sweep.
                if not all(visible(advance(pose, steering, travel), seat) for seat in TARGETS):
                    continue
                nominal = evaluate(pose, steering, travel)
                if not nominal['passed']:
                    continue
                cases = [evaluate(pose, steering, travel, shift, gain, coast)
                         for shift in itertools.product((-10, 0, 10), (-10, 0, 10), (-2, 0, 2))
                         for gain in (0.85, 1.0, 1.15) for coast in (0, 10, 20)]
                candidates.append(dict(t=time, start=pose, steering_deg=steering,
                                       travel_mm=travel, endpoint=nominal['endpoint'],
                                       cases=len(cases), failures=sum(not c['passed'] for c in cases),
                                       wall_mm=min(c['wall_mm'] for c in cases),
                                       pillar_mm=min(c['pillar_mm'] for c in cases)))
    candidates.sort(key=lambda c: (c['failures'], c['travel_mm'], c['steering_deg']))
    return dict(log=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                nominal_candidates=len(candidates), robust_candidates=sum(c['failures'] == 0 for c in candidates),
                best=candidates[:10],
                assumptions='24 occupied legal seats; capsule radius70/front60/rear140; margin25; '
                            'XY +/-10mm, heading +/-2deg, yaw gain .85/1/1.15, coast0/10/20mm; '
                            'constant steering; no settling transient, segmentation or route join')


def sequential_search(path):
    """Seek two distinct fixed stops on one arc, each seeing one seat robustly.

    Each stop permits 0..20mm coast. The second nominal stop must lie at least
    30mm beyond the first so the modeled first braking overshoot cannot pass it.
    The maximum swept distance includes both stops' accumulated 20mm coast.
    """
    candidates = []
    shifts = tuple(itertools.product((-10, 0, 10), (-10, 0, 10), (-2, 0, 2)))
    for time, pose in approach_poses(path):
        # Reject starting envelopes that already fail the conservative model.
        if any(min(margins(tuple(a+b for a, b in zip(pose, shift)))) <= 25 for shift in shifts):
            continue
        for steering in range(4, 43, 2):
            stops = [[], []]
            for travel in range(30, 251, 10):
                for side, seat in enumerate(TARGETS):
                    if all(visible(advance(tuple(a+b for a, b in zip(pose, shift)),
                                           steering, travel+coast, gain), seat)
                           for shift in shifts for gain in (0.85, 1, 1.15)
                           for coast in (0, 10, 20, 30, 40)):
                        stops[side].append(travel)
            pairs = [(a, b) for a in stops[0] for b in stops[1] if abs(a-b) >= 30]
            if not pairs:
                continue
            first, second = min(pairs, key=lambda pair: max(pair))
            limit = max(first, second) + 40
            wall = pillar = math.inf
            safe = True
            for shift in shifts:
                start = tuple(a+b for a, b in zip(pose, shift))
                for gain in (0.85, 1, 1.15):
                    for distance in range(0, limit+1, 5):
                        w, p = margins(advance(start, steering, distance, gain))
                        wall, pillar = min(wall, w), min(pillar, p)
                        if min(wall, pillar) <= 25:
                            safe = False
                            break
                    if not safe:
                        break
                if not safe:
                    break
            if safe:
                candidates.append(dict(t=time, start=pose, steering_deg=steering,
                                       seat0_stop_mm=first, seat1_stop_mm=second,
                                       wall_mm=wall, pillar_mm=pillar,
                                       swept_travel_mm=limit, motion_cases=81))
    candidates.sort(key=lambda c: c['swept_travel_mm'])
    return dict(robust_candidates=len(candidates), best=candidates[:10])


def heading_stop(pose, steering, goal, gain, coast):
    curvature = -math.tan(math.radians(steering)) / 100 * gain
    travel = math.radians(wrap180(goal-pose[2])) / curvature
    end = advance(pose, steering, travel, gain)
    # Ideal steering centred at the gyro stop; actual settling remains unmodeled.
    return advance(end, 0, coast), travel


def heading_search(path):
    candidates = []
    shifts = tuple(itertools.product((-10, 0, 10), (-10, 0, 10), (-2, 0, 2)))
    for time, pose in approach_poses(path):
        if any(min(margins(tuple(a+b for a, b in zip(pose, shift)))) <= 25 for shift in shifts):
            continue
        for steering in range(20, 43, 2):
            for goal in range(90, 141, 2):
                nominal, travel = heading_stop(pose, steering, goal, 1, 0)
                if not 20 <= travel <= 300 or not all(visible(nominal, seat) for seat in TARGETS):
                    continue
                cases = [(tuple(a+b for a, b in zip(pose, shift)), gain, coast, error)
                         for shift in shifts for gain in (.85, 1, 1.15)
                         for coast in (0, 10, 20) for error in (-2, 0, 2)]
                if not all(all(visible(heading_stop(start, steering, goal+error, gain, coast)[0], seat)
                               for seat in TARGETS) for start, gain, coast, error in cases):
                    continue
                wall = pillar = math.inf
                safe = True
                # Coast scenarios share the arc; check the longest straight coast.
                for shift in shifts:
                    start = tuple(a+b for a, b in zip(pose, shift))
                    for gain in (.85, 1, 1.15):
                        for error in (-2, 0, 2):
                            end, distance = heading_stop(start, steering, goal+error, gain, 0)
                            poses = [advance(start, steering, d, gain) for d in range(0, int(distance), 5)]
                            poses += [advance(end, 0, d) for d in (0, 5, 10, 15, 20)]
                            for at in poses:
                                w, p = margins(at)
                                wall, pillar = min(wall, w), min(pillar, p)
                                if min(wall, pillar) <= 25:
                                    safe = False
                                    break
                            if not safe:
                                break
                        if not safe:
                            break
                    if not safe:
                        break
                if safe:
                    candidates.append(dict(t=time, start=pose, steering_deg=steering,
                                           goal_deg=goal, nominal_travel_mm=travel,
                                           endpoint=nominal, cases=len(cases),
                                           wall_mm=wall, pillar_mm=pillar))
    candidates.sort(key=lambda c: c['nominal_travel_mm'])
    return dict(robust_candidates=len(candidates), best=candidates[:10],
                limitation='Ideal gyro stop +/-2deg, instant steering centring, straight coast0..20mm; '
                           'no servo transient, camera colour evidence or subsequent route join')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--heading-only', action='store_true')
    args = parser.parse_args()
    reports = []
    for path in args.logs:
        report = (dict(log=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                  if args.heading_only else search(path))
        if not args.heading_only:
            report['sequential'] = sequential_search(path)
        report['heading_feedback'] = heading_search(path)
        reports.append(report)
        print(path.name, 'heading candidates:', report['heading_feedback']['robust_candidates'],
              'best:', report['heading_feedback']['best'][:1], flush=True)
    destination = Path('local_workspace/corner-forward-view') / ('heading.json' if args.heading_only else 'report.json')
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(reports, indent=2))


if __name__ == '__main__':
    main()
