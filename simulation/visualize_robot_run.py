#!/usr/bin/env python3
"""Render whole-run SVG/PNG views from archived logs, including recorded pillars."""
from __future__ import annotations

import argparse
import hashlib
import math
import re
from pathlib import Path

from analyze_parking_exit_pose import ParsedLog, parse_log_sessions, triple

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / 'local_workspace/parking-exit-analysis-all'


def seat_position(seat: int, turn: int) -> tuple[float, float]:
    """Match initializeSeats: 500 mm stations, +/-100 mm lateral, CW/CCW."""
    if not 0 <= seat < 24 or turn not in (-1, 1):
        raise ValueError('seat must be 0..23 and turn must be -1 or +1')
    section, within = divmod(seat, 6)
    station, side = divmod(within, 2)
    x = turn * (-500 + station * 500)
    y = -1000 + turn * (-100 if side == 0 else 100)
    angle = turn * section * math.pi / 2
    return (round(x * math.cos(angle) - y * math.sin(angle), 6),
            round(x * math.sin(angle) + y * math.cos(angle), 6))


def recorded_pillars(text: str, turn: int) -> list[tuple[int, str, float, float]]:
    """Only accepted map/route records; rejected camera projections aren't pillars."""
    seats: dict[int, str] = {}
    for line in text.splitlines():
        m = re.search(r'\[MAP\] Confirmed S([0-3]) station=([0-2]) side=(RIGHT|LEFT) color=(RED|GREEN)', line)
        if m:
            section, station = int(m[1]), int(m[2])
            seats[section * 6 + station * 2 + (m[3] == 'LEFT')] = m[4]
        m = re.search(r'(?:Live avoidance injected|Later-lap avoidance) seat=(\d+) color=(RED|GREEN)', line)
        if not m:
            m = re.search(r'Stored behind start seat/color=(\d+)/(RED|GREEN)', line)
        if m:
            seats[int(m[1])] = m[2]
    return [(seat, color, *seat_position(seat, turn))
            for seat, color in sorted(seats.items())]


def session_text(log: ParsedLog) -> str:
    lines = log.path.read_text(encoding='utf-8', errors='replace').splitlines()
    return '\n'.join(lines[log.source_line_start - 1:log.source_line_end])


def connector_plans(text: str) -> list[list[tuple[float, float, float]]]:
    """Never draw a chord between distinct replans or missing indices."""
    versions = []
    previous = None
    for m in re.finditer(r'\[CONNECTOR_POINT\] kind=connector index=(\d+) x=([\d.-]+) y=([\d.-]+) h=([\d.-]+)', text):
        index = int(m[1])
        point = tuple(map(float, m.group(2, 3, 4)))
        if not all(math.isfinite(value) for value in point):
            previous = None
            continue
        if previous is None or index != previous + 1:
            versions.append([])
        versions[-1].append(point)
        previous = index
    return versions


def render(log: ParsedLog, output_dir: Path) -> tuple[Path, Path]:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    text = session_text(log)
    turn = int(log.config['turn'])
    pillars = recorded_pillars(text, turn)
    exit_samples = [s for s in log.samples if s.state.startswith('segment_')]
    loc = [s for s in log.samples if s.state.startswith('localize_')]
    plans = connector_plans(text)
    plan = [point for version in plans for point in version]
    tracking = [tuple(map(float, m)) for m in re.findall(
        r'\[CONNECTOR_TRACK\] t=(\d+) progress=\d+ x=([\d.-]+) y=([\d.-]+) h=([\d.-]+)', text)]
    later: dict[int, list[tuple[int, tuple[float, float, float]]]] = {}
    sparse = []
    for line in text.splitlines():
        m = re.match(r'\[LATER_TRACK\] t=(\d+) lap=(\d+) idx=\d+ pose=([\d.,-]+)', line)
        if m:
            later.setdefault(int(m[2]), []).append((int(m[1]), triple(m[3])))
        if line.startswith(('[RED SEAT]', '[DISCOVERY_TRACE]')):
            m = re.search(r' t=(\d+).*? pose=([\d.,-]+)', line)
            if m:
                sparse.append((int(m[1]), triple(m[2])))
    sparse = sorted(set(sparse))
    fig = plt.figure(figsize=(16, 10), facecolor='white')
    gs = fig.add_gridspec(1, 2, left=.055, right=.96, bottom=.25, top=.82,
                         width_ratios=[1.35, 1], wspace=.18)
    field, zoom = fig.add_subplot(gs[0]), fig.add_subplot(gs[1])
    def line(ax, points, **kwargs):
        if points:
            ax.plot([p[0] for p in points], [p[1] for p in points], **kwargs)
    def draw(ax):
        for samples, color, label in [(exit_samples, '#1769aa', 'Parking exit: logged poses'),
                                      (loc, '#008b80', 'Localization: logged poses')]:
            # Corrections are estimator changes, never connect them as driving.
            parts = [[]]
            for sample in samples:
                if parts[-1] and any(parts[-1][-1].time_ms < int(c['t']) <= sample.time_ms
                                    for c in log.corrections):
                    parts.append([])
                parts[-1].append(sample)
            for i, part in enumerate(parts):
                line(ax, [s.pose for s in part], color=color, lw=2.3,
                     label=label if i == 0 else None)
        for i, correction in enumerate(log.corrections):
            if 'before' in correction and 'after' in correction:
                line(ax, [triple(correction['before']), triple(correction['after'])],
                     color='#a34ab5', ls=':', lw=2, label='Pose correction' if i == 0 else None)
        end = triple(log.corrections[-1]['after']) if log.corrections else (
            loc[-1].pose if loc else exit_samples[-1].pose if exit_samples else None)
        if plan and end:
            line(ax, [end, plan[0]], color='#d98518', ls=':', lw=1.8,
                 label='Scan endpoints (path unavailable)')
        for version, points in enumerate(plans):
            line(ax, points, color='#888', ls='--', lw=1.5,
                 label='Accepted connector plans (separate versions)' if version == 0 else None)
        # Show sparse connector observations as points; do not invent intervening motion.
        if tracking:
            prefix = [p for t, p in sparse if t < tracking[0][0]]
            if prefix:
                ax.scatter([p[0] for p in prefix], [p[1] for p in prefix], s=9,
                           color='#d98518', label='Early scan/connector pose records')
            line(ax, [r[1:] for r in tracking], color='#d98518', lw=2.5,
                 marker='.' if len(tracking) == 1 else None, label='Connector: logged poses')
            ax.scatter(*tracking[-1][1:3], color='#222', marker='D', s=42,
                       zorder=6, label='Last connector sample')
        if exit_samples:
            ax.scatter(*exit_samples[0].pose[:2], color='#149443', s=55,
                       zorder=6, label='Exit start')
        gap_match = re.search(r'Prototype footprint .*? gap_mm=([\d.]+)', text)
        gap = float(gap_match[1]) if gap_match else 247.5
        for x in (480 - gap - 20, 480):
            ax.add_patch(Rectangle((x, -1500), 20, 200, facecolor='#eaa3cf', edgecolor='#b95599'))
        for seat, color, x, y in pillars:
            # Seat centers are nominal firmware geometry, not camera-projected positions.
            ax.scatter(x, y, marker='s', s=85, facecolor='#dc4545' if color == 'RED' else '#38a85c',
                       edgecolor='#333', linewidth=.6, zorder=7)
            if ax.get_xlim()[0] <= x <= ax.get_xlim()[1] and ax.get_ylim()[0] <= y <= ax.get_ylim()[1]:
                ax.annotate(f'{color[0]}{seat}', (x, y), xytext=(7, 7),
                            textcoords='offset points', fontsize=8, zorder=8)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)')
        ax.grid(color='#e5e5e5', lw=.6)
        ax.spines[['top', 'right']].set_visible(False)
    field.set_xlim(-1580, 1580); field.set_ylim(-1580, 1580)
    detail_points = [s.pose for s in exit_samples + loc] + plan + [r[1:] for r in tracking]
    if detail_points:
        zoom.set_xlim(min(p[0] for p in detail_points) - 100, max(p[0] for p in detail_points) + 130)
        zoom.set_ylim(min(p[1] for p in detail_points) - 100, max(p[1] for p in detail_points) + 100)
    else:
        zoom.set_xlim(-600, 600); zoom.set_ylim(-1500, -800)
    draw(field); draw(zoom)
    field.add_patch(Rectangle((-1500, -1500), 3000, 3000, fill=False, edgecolor='#222', lw=2))
    field.add_patch(Rectangle((-500, -500), 1000, 1000, facecolor='#eee', edgecolor='#222', lw=1.5))
    field.text(0, 0, 'Inner field', ha='center', color='#777')
    first_end = min((t for lap, rows in later.items() if lap >= 2 for t, p in rows), default=math.inf)
    connector_end = tracking[-1][0] if tracking else 0
    points = [p for t, p in sparse if connector_end <= t < first_end]
    if points:
        field.scatter([p[0] for p in points], [p[1] for p in points], s=12,
                      color='#999', alpha=.7, label='Lap 1: sparse logged poses')
    colors = ['#2960b2', '#b35197', '#008b80']
    for i, (lap, rows) in enumerate(sorted(later.items())):
        line(field, [p for t, p in rows], color=colors[i % len(colors)], lw=1.7,
             alpha=.85, label=f'Lap {lap}: tracking samples')
    field.set_title('Whole-field recorded run + mapped pillars', loc='left', fontsize=13, pad=12)
    zoom.set_title('Parking exit, scan and connector', loc='left', fontsize=13, pad=12)
    direction = 'CCW' if turn == 1 else 'CW'
    fig.suptitle(f'{log.label} · {direction} · whole recorded run', x=.055, y=.96, ha='left', fontsize=20)
    complete = 'Three-lap finish recorded' if 'Three-lap test complete' in text else (
        'Connector steering rejected' if 'Forward tracking rejected' in text else 'See original log for finish/hold outcome')
    merge = 'Connector merge completed' if re.search(r'\[PARK ENTRY CONNECTOR\] Complete', text) else 'No connector completion recorded'
    fig.text(.055, .91, f'{merge} · {complete} · {len(pillars)} mapped pillars', fontsize=12)
    fig.text(.055, .87, 'Pillars: red/green squares, labels R/G + seat index. Only accepted records from this session; unobserved seats omitted.', fontsize=10)
    handles, labels = field.get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower left', bbox_to_anchor=(.05, .085), ncol=3,
               frameon=False, fontsize=9, columnspacing=1.6)
    fig.text(.055, .065, 'Onboard rear-axle estimates; equal X/Y scale. Sparse records have gaps; sampled lines are not continuous ground truth.', fontsize=9, color='#555')
    fig.text(.055, .04, 'Pillar centers use nominal firmware seats (100 mm lateral offset). Square markers are symbols, not scaled footprints. Map may be incomplete or wrong.', fontsize=9, color='#555')
    fig.text(.055, .018, f'Source SHA-256: {hashlib.sha256(log.path.read_bytes()).hexdigest()}', fontsize=8, color='#777')
    output_dir.mkdir(parents=True, exist_ok=True)
    name = ''.join(c if c.isalnum() else '_' for c in log.label) + '_whole_run'
    svg, png = output_dir / f'{name}.svg', output_dir / f'{name}.png'
    for path in (svg, png):
        fig.savefig(path, dpi=180)
    plt.close(fig)
    return svg, png


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--output-dir', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    for path in args.logs:
        source = path.resolve()
        output = args.output_dir.resolve()
        if output == source.parent or source.parent in output.parents:
            parser.error('output must be outside the source log directory')
        for log in parse_log_sessions(source):
            for artifact in render(log, output):
                print(artifact)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
