"""Audit the latest 20 archived files; original bytes stay unchanged.

Cached poses and sparse diagnostics explain decisions, not physical ground
truth or the appearance of an unavailable camera exposure.
"""
import hashlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / 'simulation/evidence/parking_exit_diagnostics'
OUTPUT = ROOT / 'local_workspace/lap1-last20'


def fields(line):
    return dict(re.findall(r'(\S+)=([^ ]+)', line))


def analyze(path, index):
    data = path.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if digest not in index:
        raise ValueError(f'Hash missing from evidence index: {path.name}')
    lines = data.decode('utf-8', errors='replace').splitlines()
    traces = []
    for number, line in enumerate(lines, 1):
        if not line.startswith('[DISCOVERY_TRACE]'):
            continue
        row = fields(line)
        row.update(line=number, t=int(row['t']), station=int(row['station']))
        row['pose'] = list(map(float, row['pose'].split(',')))
        row['nudge'] = float(row['nudge'])
        row['seats'] = [list(map(float, row[f's{side}'].split(','))) for side in (0, 1)]
        traces.append(row)
    recoveries = []
    for number, line in enumerate(lines, 1):
        if line != '[CORNER VIEW] Stopped; settle steering before preflight':
            continue
        # The expired hold trace follows this event, before the first motion.
        hold = next((r for r in traces if r['line'] > number and r['reason'] == 'hold_expired'), None)
        if hold is None:
            raise ValueError(f'No matching hold trace: {path.name}:{number}')
        end = next((n for n in range(number, len(lines)) if
                    '[CORNER VIEW] Resolved; resume' in lines[n] or
                    '[CORNER VIEW] Locked:' in lines[n]), len(lines) - 1)
        window = lines[number - 1:end + 1]
        prior = [r for r in traces if r['station'] == hold['station'] and
                 r['t'] <= hold['t'] and r['reason'] == 'approach']
        recovery = dict(line=number, station=hold['station'], hold=hold,
                        before=prior, end_line=end + 1,
                        events=[v for v in window if v.startswith('[CORNER VIEW]') or
                                v.startswith('[CORNER VIEW POSE]') or
                                v.startswith('[SEAT CLEAR]') or
                                v.startswith('[PATH] Live avoidance')],
                        observations=[v for v in window if v.startswith('[CORNER VIEW OBS]')],
                        result=lines[end])
        recoveries.append(recovery)
    return dict(file=path.name, sha256=digest, bytes=len(data), excerpt='_excerpt' in path.stem,
                firmware=[v for v in lines if ('control_revision=' in v or
                           '[PARK_DIAG_CONFIG]' in v)],
                run_end=[v for v in lines if v.startswith('[RUN_END]')],
                entered_lap1=any('control_revision=' in v for v in lines),
                traces=traces, recoveries=recoveries,
                acquisition=[v for v in lines if ('decision=confirmed' in v or
                             '[PATH INJECT POSE]' in v or '[SEAT CLEAR]' in v)])


def main():
    paths = sorted((p for p in EVIDENCE.glob('*.txt') if re.search(r'log_(\d+)', p.name)),
                   key=lambda p: int(re.search(r'log_(\d+)', p.name)[1]))[-20:]
    index = (EVIDENCE / 'README.md').read_text(encoding='utf-8')
    rows = [analyze(p, index) for p in paths]
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / 'report.json').write_text(json.dumps(rows, indent=2) + '\n', encoding='utf-8')
    print('FILES', len(rows), 'EXCERPTS', sum(r['excerpt'] for r in rows),
          'RECOVERIES', sum(len(r['recoveries']) for r in rows))
    for row in rows:
        print(row['file'], 'recoveries', len(row['recoveries']),
              'lap1', row['entered_lap1'], 'footer', bool(row['run_end']))
        for r in row['recoveries']:
            print(' station', r['station'], 'hold', r['hold']['t'],
                  'bearing/range', [s[:2] for s in r['hold']['seats']])
            for side in (0, 1):
                usable = [v for v in r['before'] if v['seats'][side][2]]
                print('  side', side, 'visible approach records', len(usable),
                      'first', (usable[0]['t'], usable[0]['seats'][side][:2]) if usable else None)
            print(' ', ' | '.join(v for v in r['events'] if
                                 any(k in v for k in ('Reverse station', 'No safe', 'Active parallax',
                                                      'Arc mismatch', 'Resolved; resume', 'extend approach'))))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
