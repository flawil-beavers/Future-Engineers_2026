"""Portable summary of the latest six files; no whole-run inference from650."""
import hashlib
import json
import re
import statistics
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'simulation/evidence/parking_exit_diagnostics'
OUTPUT = ROOT / 'local_workspace/returned-645-650'


def main():
    rows = []
    for number in range(645, 651):
        matches = list(SOURCE.glob(f'20261105_log_{number}_*.txt'))
        if len(matches) != 1:
            raise RuntimeError(f'Expected one portable evidence file for{number}')
        path = matches[0]
        raw = path.read_bytes()
        text = raw.decode('utf-8', errors='replace')
        row = dict(file=path.name, bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest(),
                   excerpt='_excerpt' in path.stem,
                   run_end=re.findall(r'\[RUN_END\][^\r\n]*', text),
                   identity=re.findall(r'\[(?:PARK_DIAG_CONFIG|PATH|FINAL PARK)\][^\r\n]*(?:build=|control_revision=|revision=)[^\r\n]*', text),
                   planning_us=[int(v) for v in re.findall(r'Handoff planning_us=(\d+)', text)])
        laps = {}
        for lap in (2, 3):
            speeds = [float(v) for v in re.findall(
                rf'\[LATER_TRACK\][^\r\n]*lap={lap} [^\r\n]*speed/servo=([\d.]+)', text)]
            if speeds:
                laps[lap] = dict(samples=len(speeds), minimum=min(speeds),
                                 median=statistics.median(speeds), maximum=max(speeds))
        row['sampled_command_speed_mm_s'] = laps
        row['parking_wall_corrections_mm'] = [float(v) for v in re.findall(
            r'initial_y_correction_mm=(-?[\d.]+)', text)]
        row['parking_abort'] = re.findall(r'\[FINAL PARK ABORT\] ([^\r\n]+)', text)
        rows.append(row)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / 'summary.json').write_text(json.dumps(rows, indent=2)+'\n', encoding='utf-8')
    lines = ['# Latest six returned files', '',
             'Speed statistics are bounded LATER_TRACK command samples, not measured speed or full-run coverage.',
             '650 is an incomplete original excerpt, not a completed run.', '',
             '| File | Bytes | Outcome | Handoff ms | Wall correction mm |',
             '| --- | ---: | --- | ---: | ---: |']
    for r in rows:
        outcome = r['parking_abort'] or r['run_end'] or ['incomplete']
        lines.append(f"| {r['file']} | {r['bytes']} | {'; '.join(outcome)} | "
                     f"{', '.join(str(v/1000) for v in r['planning_us'])} | "
                     f"{', '.join(map(str,r['parking_wall_corrections_mm']))} |")
    (OUTPUT / 'summary.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    print('RETURNED FILES', len(rows), 'EXCERPTS', sum(r['excerpt'] for r in rows))


if __name__ == '__main__':
    main()
