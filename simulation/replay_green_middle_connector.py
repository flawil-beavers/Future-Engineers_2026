"""Replay the logged GREEN-middle connector with a continued route lookahead.

This is an ideal kinematic fixed-field and pink-rail clearance check, not a
physical validation. Pink pieces use the repository's measured ideal geometry.
The sensitivity grid is assumed rather than measured uncertainty.
Run from the repository root with the archived logs 430 and 432 present.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path('simulation').resolve()))
from analyze_connector_tracking import fields, parse_point, lookahead, steering
from parking_entry_scout_sim import clearance
from parking_exit_swept_search import Pose, collision


def points(path):
    connector, route = [], []
    tail = None
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if line.startswith('[CONNECTOR_POINT]'):
            item = fields(line)
            (connector if item['kind'] == 'connector' else route).append(parse_point(item))
        elif line.startswith('[CONNECTOR_TRACK]'):
            tail = fields(line)
    return connector, route, tail


def wrap(angle):
    return (angle + 180.0) % 360.0 - 180.0


def run(connector, route, extend, dx=0, dy=0, dh=0, yaw_gain=1,
        start=None, start_progress=0):
    pose = dict(connector[0] if start is None else start)
    pose['x'] += dx
    pose['y'] += dy
    pose['h'] += dh
    progress = start_progress
    min_wall = min_pillar = float('inf')
    for step in range(251):
        angle = math.radians(pose['h'])
        for rear in (0.0, 70.0):
            x = pose['x'] - rear * math.cos(angle)
            y = pose['y'] - rear * math.sin(angle)
            wall, pillar = clearance((x, y, pose['h']), (0.0, -900.0))
            min_wall, min_pillar = min(min_wall, wall), min(min_pillar, pillar)
        if min_wall <= 10.0 or min_pillar <= 10.0:
            return 'clearance', step * 2, min_wall, min_pillar
        distance = math.hypot(pose['x'] - connector[-1]['x'], pose['y'] - connector[-1]['y'])
        heading = abs(wrap(pose['h'] - connector[-1]['h']))
        if distance <= 60 and heading <= 15:
            return 'pass', step * 2, min_wall, min_pillar
        while progress + 1 < len(connector) and math.hypot(
            pose['x'] - connector[progress + 1]['x'],
            pose['y'] - connector[progress + 1]['y'],
        ) < math.hypot(
            pose['x'] - connector[progress]['x'],
            pose['y'] - connector[progress]['y'],
        ):
            progress += 1
        target = lookahead(connector, route, progress, 82.5, extend=extend)
        forward, _, command = steering(pose, target, 100)
        if forward <= 1 or abs(command) > 42:
            return 'steering', step * 2, min_wall, min_pillar
        local_pose = Pose(480.0 - pose['x'], pose['y'] + 1500.0,
                          180.0 - pose['h'])
        if any(collision(local_pose, -round(command), gap)
               for gap in (242.5, 252.5)):
            return 'pink', step * 2, min_wall, min_pillar
        curvature = -math.tan(math.radians(command)) / 100 * yaw_gain
        delta = curvature * 2
        if abs(curvature) > 1e-8:
            pose['x'] += (math.sin(angle + delta) - math.sin(angle)) / curvature
            pose['y'] += (math.cos(angle) - math.cos(angle + delta)) / curvature
        else:
            pose['x'] += 2 * math.cos(angle)
            pose['y'] += 2 * math.sin(angle)
        pose['h'] += math.degrees(delta)
    return 'travel', 500, min_wall, min_pillar


for number in (430, 432):
    path = Path(f'simulation/evidence/parking_exit_diagnostics/20261005_log_{number}_cw.txt')
    connector, route, tail = points(path)
    for extend in (False, True):
        trials = [run(connector, route, extend, dx, dy, dh, gain)
                  for dx in (-10, 0, 10)
                  for dy in (-10, 0, 10)
                  for dh in (-2, 0, 2)
                  for gain in (0.85, 1.0, 1.15)]
        nominal = run(connector, route, extend)
        failures = sum(result[0] != 'pass' for result in trials)
        print(number, extend, 'nominal=', nominal,
              'failures=', failures, '/', len(trials),
              'min_wall=', round(min(r[2] for r in trials), 1),
              'min_pillar=', round(min(r[3] for r in trials), 1))
        actual = dict(x=float(tail['x']), y=float(tail['y']), h=float(tail['h']))
        result = run(connector, route, extend, start=actual,
                     start_progress=int(tail['progress']))
        tail_trials = [run(connector, route, extend, dx, dy, dh, gain,
                           start=actual, start_progress=int(tail['progress']))
                       for dx in (-10, 0, 10)
                       for dy in (-10, 0, 10)
                       for dh in (-2, 0, 2)
                       for gain in (0.85, 1.0, 1.15)]
        print('  actual_tail=', result,
              'failures=', sum(r[0] != 'pass' for r in tail_trials), '/', len(tail_trials),
              'min_wall=', round(min(r[2] for r in tail_trials), 1),
              'min_pillar=', round(min(r[3] for r in tail_trials), 1))
        if extend:
            assert result[0] == 'pass'
            assert all(r[0] == 'pass' and r[2] > 10 and r[3] > 10
                       for r in trials + tail_trials)
        else:
            assert result[0] == 'steering'
