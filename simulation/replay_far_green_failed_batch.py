"""Reproduce the 449/451 connector stops and check route continuation.

This is a fixed-field ideal model and a sensitivity grid, not a robot safety
proof. It does not model post-handoff driving, whose actual pose is unlogged.
"""
from pathlib import Path

from analyze_connector_tracking import fields, lookahead, steering
from connector_transition_sim import geometry, simulate


root = Path(__file__).resolve().parent
for number in (449, 450, 451):
    log = root / 'evidence' / 'parking_exit_diagnostics' / f'20261005_log_{number}_cw.txt'
    connector, route = geometry(log)
    records = [fields(line) for line in log.read_text(errors='replace').splitlines()
               if line.startswith('[CONNECTOR_TRACK]')]
    assert len(connector) >= 20 and len(route) >= 5 and records
    tail = records[-1]
    pose = {key: float(tail[key]) for key in ('x', 'y', 'h')}
    progress = int(tail['progress'])
    commands = []
    for continued in (False, True):
        target = lookahead(connector, route, progress, 150, extend=continued)
        commands.append(steering(pose, target, 100)[2])
    results = [simulate(connector, route, L=150, start_index=progress,
                        start_pose=pose, shift=(dx, dy, dh),
                        continue_route=True, pillar_seats=((-500, -900),),
                        check_pink=True)
               for dx in (-10, 0, 10) for dy in (-10, 0, 10)
               for dh in (-2, 0, 2)]
    from_start = [simulate(connector, route, L=150, shift=(dx, dy, dh),
                           yaw_gain=gain, continue_route=True,
                           pillar_seats=((-500, -900),), check_pink=True)
                  for dx in (-10, 0, 10) for dy in (-10, 0, 10)
                  for dh in (-2, 0, 2) for gain in (0.85, 1, 1.15)]
    tail_failures = sum(result['status'] != 'pass' for result in results)
    start_failures = sum(result['status'] != 'pass' for result in from_start)
    print(log.name, 'finite/continued_steer=',
          '/'.join(f'{value:.1f}' for value in commands),
          'tail_failures=', tail_failures, '/', len(results),
          'start_failures=', start_failures, '/', len(from_start),
          'max_start_travel_mm=', max(result['travel_mm'] for result in from_start))
    assert tail_failures == 0 and start_failures == 0
