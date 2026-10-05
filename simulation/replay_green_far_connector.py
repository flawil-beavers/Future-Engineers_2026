"""Check the saved CW connector after late confirmation of start seat4 GREEN.

The original logs stop at replanning, so intermediate poses are sampled from
logged connector waypoints. This is an ideal kinematic sensitivity check, not
a measurement of the robot's actual pose at the confirmation instant.
"""
from pathlib import Path

from connector_transition_sim import geometry, simulate

root = Path(__file__).resolve().parent
for number in (433, 434, 435):
    source = root / 'evidence' / 'parking_exit_diagnostics' / f'20261005_log_{number}_cw.txt'
    connector, route = geometry(source)
    assert len(connector) >= 18 and len(route) >= 5, source
    assert all(round(point['y']) == -1000 for point in route[:5]), source
    assert all(route[i + 1]['x'] <= route[i]['x'] for i in range(4)), source
    assert route[4]['x'] >= -100, source
    cases = []
    for index in (0, 5, 10, 15):
        for x in (-10, 0, 10):
            for y in (-10, 0, 10):
                for heading in (-2, 0, 2):
                    result = simulate(connector, route, shift=(x, y, heading),
                                      start_index=index, check_pink=True,
                                      pillar_seats=((-500, -900),))
                    cases.append((index, result))
    failures = [(index, result) for index, result in cases if result['status'] != 'pass']
    print(source.name, 'cases=', len(cases), 'failures=', len(failures),
          'max_remaining_mm=', max(result['travel_mm'] for _, result in cases),
          'min_wall_mm=', round(min(result.get('wall_mm', float('inf')) for _, result in cases), 1),
          'min_pillar_mm=', round(min(result.get('pillar_mm', float('inf')) for _, result in cases), 1))
    if failures:
        print('  first_failure=', failures[0])
    assert not failures
