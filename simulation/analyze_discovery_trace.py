"""Summarize why per-seat clear evidence is blocked in cached discovery traces."""
import argparse
import hashlib
import json
from pathlib import Path


def analyze(path, minimum_range=230.0, maximum_range=600.0, bearing_limit=26.426):
    rows = []
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.startswith('[DISCOVERY_TRACE]'):
            continue
        record = dict(token.split('=', 1) for token in line.split() if '=' in token)
        if record.get('v') != '1':
            raise ValueError('Unsupported discovery trace schema')
        row = {key: record[key] for key in ('t', 'frame_t', 'reason', 'station', 'pose', 'obs', 'valid')}
        row['frame_age_ms'] = int(record['t']) - int(record['frame_t'])
        row['seats'] = []
        for side in range(2):
            values = record[f's{side}'].split(',')
            if len(values) != 7:
                raise ValueError('Incomplete per-seat discovery record')
            bearing, distance = map(float, values[:2])
            visible, raw_block, clear_allowed, count, stored_clear = map(int, values[2:])
            reasons = []
            if abs(bearing) > bearing_limit:
                reasons.append('outside_bearing_window')
            if distance < minimum_range:
                reasons.append('too_near')
            elif distance > maximum_range:
                reasons.append('too_far')
            if raw_block:
                reasons.append('rejected_colour_overlap')
            if not clear_allowed:
                reasons.append('observation_does_not_allow_clear')
            row['seats'].append(dict(side=side, bearing_deg=bearing, range_mm=distance,
                                     visible=bool(visible), raw_block=bool(raw_block),
                                     clear_allowed=bool(clear_allowed), clear_frames=count,
                                     stored_clear=bool(stored_clear), reasons=reasons))
        rows.append(row)
    stations = []
    for station in sorted({row['station'] for row in rows}, key=int):
        group = [row for row in rows if row['station'] == station]
        stations.append(dict(station=int(station), records=len(group),
                             visible_records=[sum(row['seats'][side]['visible'] for row in group)
                                              for side in range(2)],
                             hold_records=[row for row in group if row['reason'].startswith('hold_')]))
    return dict(log=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                status='analyzed' if rows else 'missing_discovery_trace',
                thresholds=dict(minimum_range_mm=minimum_range, maximum_range_mm=maximum_range,
                                bearing_limit_deg=bearing_limit), stations=stations,
                limitation='Cached onboard pose/visibility; a stored CLEAR is not independent ground truth. Frame age is receipt/processing age, not camera acquisition age.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    args = parser.parse_args()
    reports = [analyze(path) for path in args.logs]
    destination = Path('local_workspace/discovery-trace/report.json')
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(reports, indent=2), encoding='utf-8')
    for report in reports:
        print(report['log'], report['status'])
        for station in report['stations']:
            if station['hold_records']:
                print('station', station['station'], 'visible record counts', station['visible_records'])
                for row in station['hold_records']:
                    print(row['reason'], [(s['side'], s['range_mm'], s['bearing_deg'], s['reasons']) for s in row['seats']])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
