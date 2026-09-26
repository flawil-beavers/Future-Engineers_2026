"""Replay logged connector targets; route continuation is an offline candidate only."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def fields(line):
    return dict(token.split('=', 1) for token in line.split() if '=' in token)


def steering(pose, target, wheelbase):
    dx, dy = target['x'] - pose['x'], target['y'] - pose['y']
    angle = math.radians(pose['h'])
    forward = dx * math.cos(angle) + dy * math.sin(angle)
    lateral = -dx * math.sin(angle) + dy * math.cos(angle)
    command = -math.degrees(math.atan(wheelbase * 2 * lateral / max(1, dx * dx + dy * dy)))
    return forward, lateral, command


def lookahead(connector, route, progress, distance, extend=False):
    points = connector + route[1:] if extend else connector
    index, accumulated = progress, 0.0
    while index + 1 < len(points) and accumulated < distance:
        accumulated += math.hypot(points[index + 1]['x'] - points[index]['x'],
                                  points[index + 1]['y'] - points[index]['y'])
        index += 1
    if extend and accumulated < distance:
        raise ValueError('Route excerpt does not cover requested lookahead')
    return points[index]


def parse_point(record):
    return {name: float(record[name]) for name in ('x', 'y', 'h')}


def analyze(path):
    connector, route, rows = [], [], []
    preflight_rejection = None
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if line.startswith('[PARK ENTRY CONNECTOR] Preflight FAIL'):
            preflight_rejection = line
        if line.startswith('[CONNECTOR_POINT]'):
            record = fields(line)
            if record['kind'] == 'connector' and record['index'] == '0':
                connector, route = [], []  # A replan starts another geometry snapshot.
            destination = connector if record['kind'] == 'connector' else route
            if int(record['index']) != len(destination):
                raise ValueError('Missing/reordered geometry point')
            destination.append(parse_point(record))
        elif line.startswith('[CONNECTOR_TRACK]'):
            raw = fields(line)
            record = {key: float(value) for key, value in raw.items()}
            if len(connector) < 3 or not route:
                raise ValueError('Tracking record without complete geometry')
            if math.hypot(connector[-1]['x'] - route[0]['x'],
                          connector[-1]['y'] - route[0]['y']) > 0.1:
                raise ValueError('Route does not start at connector merge')
            target = lookahead(connector, route, int(record['progress']), record['lookahead'])
            if math.hypot(target['x'] - record['tx'], target['y'] - record['ty']) > 0.1:
                raise ValueError('Logged target disagrees with finite-path replay')
            forward, lateral, command = steering(record, target, record['wheelbase'])
            # Printed XY/heading is rounded. This is a consistency check,
            # not a bit-exact reconstruction of floating-point firmware.
            if abs(command - record['steering']) > 0.2:
                raise ValueError('Logged steering disagrees with replay')
            alternative = lookahead(connector, route, int(record['progress']),
                                    record['lookahead'], extend=True)
            alt_forward, _, alt_command = steering(record, alternative, record['wheelbase'])
            perturbed_commands, perturbed_forward = [], []
            for delta_x in (-10, 0, 10):
                for delta_y in (-10, 0, 10):
                    for delta_heading in (-2, 0, 2):
                        perturbed = dict(record, x=record['x'] + delta_x,
                                         y=record['y'] + delta_y,
                                         h=record['h'] + delta_heading)
                        p_forward, _, p_command = steering(perturbed, alternative, record['wheelbase'])
                        perturbed_forward.append(p_forward)
                        perturbed_commands.append(p_command)
            rows.append({
                't': int(record['t']), 'progress': int(record['progress']),
                'recorded_rejection': bool(record['rejected']),
                'finite_forward_mm': forward, 'finite_lateral_mm': lateral,
                'finite_steering_deg': command,
                'end_distance_mm': record['end_distance'],
                'end_heading_deg': record['end_heading'],
                'handoff_gate_pass': record['end_distance'] <= record['gate_distance'] and
                                     record['end_heading'] <= record['gate_heading'],
                'continuation_forward_mm': alt_forward,
                'continuation_steering_deg': alt_command,
                'continuation_tracking_guard_pass': alt_forward > 1 and
                                                     abs(alt_command) <= record['limit'],
                'continuation_assumed_grid_cases': len(perturbed_commands),
                'continuation_assumed_grid_guard_failures': sum(
                    forward <= 1 or abs(command) > record['limit']
                    for forward, command in zip(perturbed_forward, perturbed_commands)),
                'continuation_assumed_grid_max_steering_deg': max(map(abs, perturbed_commands)),
            })
    return {'log': path.name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'status': ('replayed' if rows else 'preflight_rejected_no_tracking'
                       if preflight_rejection else 'missing_connector_geometry_and_tail_pose'),
            'preflight_rejection': preflight_rejection,
            'limitation': 'Continuation is counterfactual at recorded poses; no closed-loop or swept collision validation. Grid +/-10 mm XY, +/-2 deg is assumed, not measured error bounds or probability.',
            'samples': rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--output', type=Path,
                        default=Path('local_workspace/connector-tracking/report.json'))
    args = parser.parse_args()
    reports = [analyze(path) for path in args.logs]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(reports, indent=2), encoding='utf-8')
    for report in reports:
        print(report['log'], report['status'], len(report['samples']), 'samples')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
