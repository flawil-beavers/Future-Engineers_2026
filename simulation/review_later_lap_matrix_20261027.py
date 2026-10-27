"""Expand the actual-ramp review to every uncapped geometry case.

First run check_later_laps.py and check_later_lap_dynamics.py to regenerate
the production-derived fixture. This review is diagnostic and returns nonzero
when the existing reserve/acceleration acceptance limits are breached.
"""
import hashlib
import json
import shutil
import subprocess

from check_cw_start_planner import ROOT


def main():
    dest = ROOT/'local_workspace/code_audit_20261027'
    dest.mkdir(parents=True, exist_ok=True)
    shared = ROOT/'local_workspace/later-laps'
    dynamic = ROOT/'local_workspace/later-lap-dynamics'
    cases = [c for c in json.loads((shared/'report.json').read_text())['cases']
             if c['speed_cap'] == 0]
    fixture = (dynamic/'check.cpp').read_text()
    call = 'shortenKnownInnerRoute(optimizedPath,beforeLaneCarry);'
    assert fixture.count(call) == 1
    results = {}
    for name, source in [('current', fixture),
                         ('without_new_inner_search', fixture.replace(call, ''))]:
        cpp, exe = dest/f'dynamic_{name}.cpp', dest/f'dynamic_{name}.exe'
        cpp.write_text(source)
        subprocess.run([shutil.which('g++'), '-std=c++17', '-O2',
                        '-I', str(ROOT/'include'), '-I', str(shared),
                        str(cpp), '-o', str(exe)], check=True)
        # Full control matrix distinguishes pre-existing issues from the new
        # optional inner search; do not compare only already-failing layouts.
        inputs = '\n'.join(' '.join(map(str, [c['direction'], c['speed_cap'], *c['colors']]))
                           for c in cases)+'\n'
        run = subprocess.run([str(exe), '.02'], input=inputs, text=True, capture_output=True)
        (dest/f'dynamic_{name}_output.txt').write_text(run.stdout+run.stderr)
        assert run.returncode in (0, 1), run.stderr
        rows = []
        for line in run.stdout.splitlines():
            if not line.startswith('DYNAMIC '):
                continue
            f = line.split()
            rows.append(dict(case=int(f[1]), passed=bool(int(f[2])),
                elapsed_s=float(f[3]), lateral_mmss=float(f[4]),
                peak_speed_mms=float(f[5]), peak_tracking_ratio=float(f[6]),
                wall_mm=float(f[7]), pillar_mm=float(f[8])))
        assert len(rows) == len(cases)
        results[name] = rows
        failures = [r for r in rows if not r['passed']]
        print(name, 'cases', len(rows), 'failures', len(failures), flush=True)
        for row in failures:
            print(json.dumps(row), flush=True)
    changed = [dict(case=a['case'], current=a, without_new_inner_search=b,
                    direction=cases[a['case']]['direction'], colors=cases[a['case']]['colors'])
               for a, b in zip(results['current'], results['without_new_inner_search'])
               if not a['passed'] or not b['passed']]
    report = dict(interval_s=.02, cases=cases, results=results, failing_comparison=changed,
                  fixture_sha256=hashlib.sha256(fixture.encode()).hexdigest(),
                  source_sha256={p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest()
                     for p in ('src/obstacle_path.cpp', 'src/motor_control.cpp', 'include/config.h')})
    (dest/'dynamic_matrix_review.json').write_text(json.dumps(report, indent=2)+'\n')
    raise SystemExit(1 if any(not r['passed'] for r in results['current']) else 0)


if __name__ == '__main__':
    main()
