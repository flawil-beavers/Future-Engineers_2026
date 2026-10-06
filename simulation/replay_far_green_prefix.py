"""Replay the originally armed connectors of failed far-GREEN logs 453/454.

The later connector's start point is the recorded pose at the rejected
retention. This ideal model checks the retained first connector from there;
it cannot establish physical tracking or post-handoff safety.
"""
from pathlib import Path
import math

from analyze_connector_tracking import fields, parse_point
from connector_transition_sim import simulate


root = Path(__file__).resolve().parent
for number in (453, 454):
    source = root / 'evidence' / 'parking_exit_diagnostics' / f'20261006_log_{number}_cw.txt'
    first, route, later_start = [], [], None
    for line in source.read_text(errors='replace').splitlines():
        if not line.startswith('[CONNECTOR_POINT]'):
            continue
        record = fields(line)
        point = parse_point(record)
        if record['kind'] == 'connector' and record['index'] == '0' and first:
            later_start = point
            break
        (first if record['kind'] == 'connector' else route).append(point)
    assert len(first) >= 20 and len(route) >= 5 and later_start is not None
    progress = min(range(len(first)), key=lambda i: math.hypot(
        later_start['x'] - first[i]['x'], later_start['y'] - first[i]['y']))
    # Initial merge is x=150, its 150 mm outgoing lookahead ends at x=0.
    # The next x=-50 point is in the GREEN taper and must be allowed to change.
    assert round(route[0]['x']) == 150
    assert round(route[3]['x']) == 0
    trials = [simulate(first, route, L=150, start_index=progress,
                       start_pose=later_start, shift=(dx,dy,dh),
                       yaw_gain=gain, continue_route=True,
                       pillar_seats=((-500,-900),), check_pink=True)
              for dx in (-10,0,10) for dy in (-10,0,10)
              for dh in (-2,0,2) for gain in (0.85,1,1.15)]
    nominal = simulate(first, route, L=150, start_index=progress,
                       start_pose=later_start, continue_route=True,
                       pillar_seats=((-500,-900),), check_pink=True)
    failures = sum(result['status'] != 'pass' for result in trials)
    print(source.name, 'replan_pose=', later_start, 'progress=', progress,
          'nominal=', nominal, 'failures=', failures, '/', len(trials))
    assert nominal['status'] == 'pass' and failures == 0
