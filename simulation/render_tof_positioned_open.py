#!/usr/bin/env python3
"""Position Open Challenge estimates using inner-wall ToF constraints."""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw

from render_open_video_602 import SOURCE, ARCHIVE, ROOT, sequences
from render_open_tof_rays import LEFT, RIGHT, Reading, draw_reading, ray
from render_video_aligned_open import initial_ranges
from render_video_runs import FPS, Pose, View, font, robot

OUTPUT = ROOT / 'local_workspace/video-documentation/sensor-usage/open-challenge-tof-positioned'
INNER = (-500.0, 500.0, -900.0, 900.0)
OUTER = (-1500.0, 1500.0, -1500.0, 1500.0)
PRIOR_RADIUS_MM = 300.0
font = lru_cache(maxsize=20)(font)


@dataclass
class Snapshot:
    index: int
    corner: int
    phase: str
    x: float
    y: float
    heading: float
    left: dict
    right: dict
    placement_source: str

    def pose(self):
        return Pose(0, self.x, self.y, self.heading, 'Course driving', 'ToF-positioned estimate')


def valid_range(value):
    return math.isfinite(value) and 0 < value < 9999


def section_geometry(section):
    """Return coordinate axis, inner wall and outer wall for the straight."""
    return ((1, -900.0, -1500.0), (0, 500.0, 1500.0),
            (1, 900.0, 1500.0), (0, -500.0, -1500.0))[section % 4]


def lateral_coordinate(section, event, fallback):
    axis, inner_wall, outer_wall = section_geometry(section)
    for side, wall in (('left', inner_wall), ('right', outer_wall)):
        distance = event[side]['filtered']
        if valid_range(distance):
            _, endpoint, _ = ray(0, 0, event['heading'], side, distance)
            return wall - endpoint[axis], side
    return fallback, 'prior'


def wall_hit(x, y, heading, side):
    origin, endpoint, _ = ray(x, y, heading, side, 1)
    origin = np.array(origin)
    direction = np.array(endpoint) - origin
    hits = []
    for prefix, bounds in (('inner', INNER), ('outer', OUTER)):
        xmin, xmax, ymin, ymax = bounds
        for axis, value, lower, upper, name in (
            (0, xmin, ymin, ymax, 'west'), (0, xmax, ymin, ymax, 'east'),
            (1, ymin, xmin, xmax, 'south'), (1, ymax, xmin, xmax, 'north'),
        ):
            if abs(direction[axis]) < 1e-10:
                continue
            distance = (value - origin[axis]) / direction[axis]
            point = origin + distance * direction
            if distance > 0 and lower - 1e-7 <= point[1 - axis] <= upper + 1e-7:
                hits.append(dict(distance=float(distance), point=point.tolist(),
                                 surface=f'{prefix}_{name}'))
    if not hits:
        raise ValueError('Sensor ray has no positive field intersection')
    return min(hits, key=lambda hit: hit['distance'])


def chassis_clear(position, heading):
    angle = math.radians(heading)
    forward = np.array([math.cos(angle), math.sin(angle)])
    left = np.array([-math.sin(angle), math.cos(angle)])
    polygon = np.array([position + along * forward + across * left
                        for along, across in ((-40, -67.5), (125, -67.5),
                                              (125, 67.5), (-40, 67.5))])
    if np.max(np.abs(polygon)) >= 1498:
        return False
    inner = np.array([[-500, -900], [500, -900], [500, 900], [-500, 900]])
    for axis in (np.array([1, 0]), np.array([0, 1]), forward, left):
        a = polygon @ axis
        b = inner @ axis
        if a.max() <= b.min() - 2 or b.max() <= a.min() - 2:
            return True
    return False


def arc_position(section, incoming, outgoing, radius, fraction):
    angle = section * math.pi / 2
    forward = np.round([math.cos(angle), math.sin(angle)], 12)
    inward = np.array([-forward[1], forward[0]])
    incoming_axis = section_geometry(section)[0]
    outgoing_axis = section_geometry(section + 1)[0]
    prior_position = np.zeros(2)
    prior_position[incoming_axis] = incoming
    next_position = np.zeros(2)
    next_position[outgoing_axis] = outgoing
    lateral = float(inward @ prior_position)
    finish_forward = float(forward @ next_position)
    turn = fraction * math.pi / 2
    return (forward * (finish_forward - radius + radius * math.sin(turn))
            + inward * (lateral + radius * (1 - math.cos(turn))))


def select_radius(section, incoming, outgoing, entry, exit):
    candidates = sorted(range(180, 501), key=lambda radius: (abs(radius - PRIOR_RADIUS_MM), radius))
    for radius in candidates:
        position = arc_position(section, incoming, outgoing, radius, 1)
        hit = wall_hit(*position, exit['heading'], 'left')
        target = exit['left']['filtered']
        if not hit['surface'].startswith('inner') or abs(hit['distance'] - target) > .01:
            continue
        point = hit['point']
        along_axis = 1 - section_geometry(section + 1)[0]
        lower, upper = (-500, 500) if along_axis == 0 else (-900, 900)
        if not lower + 5 <= point[along_axis] <= upper - 5:
            continue
        clear = True
        for fraction in np.linspace(0, 1, 31):
            position = arc_position(section, incoming, outgoing, radius, float(fraction))
            heading = entry['heading'] + fraction * (exit['heading'] - entry['heading'])
            if not chassis_clear(position, heading):
                clear = False
                break
        if clear:
            return float(radius)
    raise ValueError(f'No feasible ToF-positioned corner for section {section}')


def make_snapshot(index, corner, phase, position, event, source):
    return Snapshot(index, corner, phase, float(position[0]), float(position[1]),
                    event['heading'], event['left'], event['right'], source)


def build_model(text):
    sequence = sequences(text)[0]
    initial = dict(heading=0.0, **initial_ranges(text))
    initial_cross, _ = lateral_coordinate(0, initial, -1050.0)
    previous_cross = initial_cross
    snapshots = []
    arcs = []
    for corner, (entry, exit) in enumerate(zip(sequence['corners'], sequence['turns']), 1):
        section = (corner - 1) % 4
        incoming, entry_source = lateral_coordinate(section, entry, previous_cross)
        outgoing, exit_source = lateral_coordinate((section + 1) % 4, exit, previous_cross)
        radius = select_radius(section, incoming, outgoing, entry, exit)
        start = arc_position(section, incoming, outgoing, radius, 0)
        finish = arc_position(section, incoming, outgoing, radius, 1)
        entry_snapshot = make_snapshot(2 * corner - 1, corner, 'entry', start, entry, entry_source)
        exit_snapshot = make_snapshot(2 * corner, corner, 'exit', finish, exit, exit_source)
        snapshots.extend([entry_snapshot, exit_snapshot])
        arcs.append(dict(section=section, incoming=incoming, outgoing=outgoing,
                         radius=radius, entry=entry_snapshot, exit=exit_snapshot))
        previous_cross = outgoing
    first = snapshots[0]
    lateral_change = first.y - initial_cross
    first_travel = sequence['corners'][0]['distance']
    first_along = math.sqrt(first_travel ** 2 - lateral_change ** 2)
    start = np.array([first.x - first_along, initial_cross])
    initial_snapshot = make_snapshot(0, 0, 'initial', start, initial, 'left')
    stop_event = sequence['stop']
    final_cross, source = lateral_coordinate(0, stop_event, snapshots[-1].y)
    lateral_change = final_cross - snapshots[-1].y
    final_along = math.sqrt(stop_event['distance'] ** 2 - lateral_change ** 2)
    stop = np.array([snapshots[-1].x + final_along, final_cross])
    final_snapshot = make_snapshot(25, 12, 'stop', stop, stop_event, source)
    snapshots = [initial_snapshot] + snapshots + [final_snapshot]
    frames = []

    def hold(snapshot, count):
        for _ in range(count):
            frames.append(dict(pose=snapshot.pose(), snapshot=snapshot, stage='measurement'))

    def straight(a, b):
        for fraction in np.linspace(0, 1, 16)[1:]:
            pose = Pose(0, a.x + fraction * (b.x - a.x), a.y + fraction * (b.y - a.y),
                        a.heading + fraction * (b.heading - a.heading), 'Course driving', 'driving estimate')
            frames.append(dict(pose=pose, snapshot=None, stage='straight'))

    hold(initial_snapshot, 8)
    previous = initial_snapshot
    for arc in arcs:
        straight(previous, arc['entry'])
        hold(arc['entry'], 2)
        for fraction in np.linspace(0, 1, 9)[1:]:
            position = arc_position(arc['section'], arc['incoming'], arc['outgoing'], arc['radius'], float(fraction))
            a, b = arc['entry'], arc['exit']
            pose = Pose(0, float(position[0]), float(position[1]),
                        a.heading + fraction * (b.heading - a.heading), 'Course driving', 'driving estimate')
            frames.append(dict(pose=pose, snapshot=None, stage='turn'))
        hold(arc['exit'], 2)
        previous = arc['exit']
    straight(previous, final_snapshot)
    hold(final_snapshot, 15)
    return snapshots, arcs, frames


def snapshot_report(snapshot):
    result = asdict(snapshot)
    result['sensors'] = {}
    for side in ('left', 'right'):
        hit = wall_hit(snapshot.x, snapshot.y, snapshot.heading, side)
        measured = getattr(snapshot, side)['filtered']
        result['sensors'][side] = dict(logged=measured, modeled=hit['distance'],
                                      residual=hit['distance'] - measured if valid_range(measured) else None,
                                      surface=hit['surface'], wall_point=hit['point'])
    return result


def render(reference_video=None):
    cv2.setNumThreads(1)
    raw = SOURCE.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    assert digest == hashlib.sha256(ARCHIVE.read_bytes()).hexdigest()
    snapshots, arcs, frames = build_model(raw.decode('utf-8'))
    reports = [snapshot_report(s) for s in snapshots]
    for s, report in zip(snapshots, reports):
        if s.placement_source in ('left', 'right'):
            fit = report['sensors'][s.placement_source]
            assert abs(fit['residual']) < .01, (s.index, fit)
            if s.placement_source == 'left':
                assert fit['surface'].startswith('inner')
    assert all(chassis_clear(np.array([f['pose'].x, f['pose'].y]), f['pose'].h) for f in frames)
    assert len(snapshots) == 26
    assert abs(math.hypot(snapshots[1].x-snapshots[0].x, snapshots[1].y-snapshots[0].y)-622) < .01
    assert abs(math.hypot(snapshots[-1].x-snapshots[-2].x, snapshots[-1].y-snapshots[-2].y)-591) < .01
    OUTPUT.mkdir(parents=True, exist_ok=True)
    extent = 1580
    for s in snapshots:
        for side in ('left', 'right'):
            _, end, valid = ray(s.x, s.y, s.heading, side, getattr(s, side)['filtered'])
            if valid:
                extent = max(extent, *(abs(v)+60 for v in end))
    scenes = {}
    for field_only in (False, True):
        size = (1080, 1080) if field_only else (1920, 1080)
        im = Image.new('RGB', size, '#101d30' if not field_only else '#f5f4ee')
        d = ImageDraw.Draw(im)
        v = View((0, 0, 1080, 1080) if field_only else (45, 165, 945, 1065), (-extent, extent, -extent, extent))
        d.rectangle(v.rect, fill='#f5f4ee')
        v.box(d, -1500, -1500, 3000, 3000, outline='#344357', width=4)
        v.box(d, -500, -900, 1000, 1800, fill='#dfe3e6', outline='#536273', width=4)
        if not field_only:
            d.text((45, 30), 'OPEN CHALLENGE / LOG 602 / FIRST RUN', font=font(33, True), fill='white')
            d.text((45, 90), 'ToF-positioned driving estimate / illustrative pacing', font=font(24), fill='#b8c9db')
            notes = ['Orange: LEFT ToF / blue: RIGHT ToF',
                     'Solid ray + cross: logged filtered range',
                     'Dashed ray: out of range; no measured endpoint',
                     'Red segment: outer-wall range residual',
                     'Inner enclosure: 1000 x 1800 mm',
                     'Outer field: 3000 x 3000 mm',
                     'Mounts: 40 mm forward, +/-35 mm lateral',
                     'Along-track position and timing are estimated.',
                     'Brief holds show the logged measurement poses.',
                     'Camera measurements are not logged.']
            for i, line in enumerate(notes):
                d.text((995, 660+i*35), line, font=font(21), fill='#b8c9db')
        scenes[field_only] = (im, v)
    writers, paths = {}, {}
    for mode, (im, _) in scenes.items():
        for speed in (1, 4):
            name = f'run_602_tof_positioned_{"field_only" if mode else "with_panel"}_{"standard" if speed == 1 else "4x"}.mp4'
            path = OUTPUT / name
            pending = path.with_suffix('.rendering.mp4')
            writer = cv2.VideoWriter(str(pending), cv2.VideoWriter_fourcc(*'mp4v'), FPS, im.size)
            assert writer.isOpened()
            writers[mode, speed] = writer
            paths[mode, speed] = (pending, path)
    trails = {mode: Image.new('RGBA', im.size) for mode, (im, _) in scenes.items()}
    previous = None
    try:
        for tick, f in enumerate(frames):
            p, s = f['pose'], f['snapshot']
            for mode, (base, v) in scenes.items():
                if previous:
                    ImageDraw.Draw(trails[mode]).line([v.pt(*previous), v.pt(p.x, p.y)], fill='#10ad9a', width=4)
                im = base.copy()
                im.paste(trails[mode], (0, 0), trails[mode])
                d = ImageDraw.Draw(im)
                if s:
                    reading = Reading(tick, 1, s.corner, s.x, s.y, s.heading, s.left, s.right, s.phase)
                    draw_reading(d, v, reading)
                    report = reports[s.index]
                    right = report['sensors']['right']
                    if right['residual'] is not None and abs(right['residual']) > .5:
                        _, end, _ = ray(s.x, s.y, s.heading, 'right', s.right['filtered'])
                        target = v.pt(*right['wall_point'])
                        d.line([v.pt(*end), target], fill='#e14b60', width=5)
                        d.ellipse((target[0]-4, target[1]-4, target[0]+4, target[1]+4), fill='#e14b60')
                robot(d, v, p, '#10ad9a')
                if not mode:
                    d.text((995, 180), 'INNER-WALL ToF PRIORITY', font=font(29, True), fill='#6de4d3')
                    if s:
                        d.text((995, 240), f'Snapshot {s.index+1}/26 / {s.phase.upper()} / corner {s.corner}', font=font(24, True), fill='white')
                        d.text((995, 295), f'Placement: {s.placement_source.upper()} ToF', font=font(24), fill='white')
                        for i, (side, color) in enumerate((('left', LEFT), ('right', RIGHT))):
                            r = report['sensors'][side]
                            y = 365+i*110
                            value = f"{r['logged']:.0f} mm" if valid_range(r['logged']) else 'out of range'
                            d.text((995, y), f'{side.upper()} logged: {value}', font=font(26, True), fill=color)
                            if r['residual'] is not None:
                                d.text((995, y+40), f"Wall model: {r['modeled']:.1f} mm / residual {r['residual']:+.1f} mm", font=font(23), fill='#d5dfeb')
                    else:
                        d.text((995, 260), 'DRIVING ESTIMATE', font=font(29, True), fill='white')
                        d.text((995, 320), 'Between logged measurement snapshots', font=font(24), fill='#b8c9db')
                        d.text((995, 370), 'Position interpolated continuously', font=font(24), fill='#b8c9db')
                    d.text((995, 595), 'Residual = modeled range - logged range', font=font(21), fill='#e18c99')
                bgr = cv2.cvtColor(np.asarray(im), cv2.COLOR_RGB2BGR)
                for speed in (1, 4):
                    for _ in range(4 if speed == 1 else 1):
                        writers[mode, speed].write(bgr)
                if tick == len(frames)-1:
                    im.save(OUTPUT / f'preview_{"field_only" if mode else "with_panel"}.png')
            previous = (p.x, p.y)
            if tick % 100 == 0:
                print(f'Rendered {tick}/{len(frames)} presentation frames', flush=True)
    finally:
        for writer in writers.values():
            writer.release()
    exports = []
    for (mode, speed), (pending, path) in paths.items():
        cap = cv2.VideoCapture(str(pending))
        count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        assert count == len(frames)*(4 if speed == 1 else 1)
        for index in (0, count//2, count-1):
            cap.set(cv2.CAP_PROP_POS_FRAMES, index)
            ok, frame = cap.read()
            assert ok and frame.shape[:2] == ((1080, 1080) if mode else (1080, 1920))
        cap.release()
        pending.replace(path)
        exports.append(dict(file=path.name, frames=count, seconds=count/FPS, pace_multiplier=speed))
    reference = None
    if reference_video:
        h = hashlib.sha256()
        with reference_video.open('rb') as source:
            for chunk in iter(lambda: source.read(4*1024*1024), b''):
                h.update(chunk)
        reference = dict(file=reference_video.name, sha256=h.hexdigest(), use='Layout and turn-sequence reference; no video-led XY fit')
    manifest = dict(source=SOURCE.relative_to(ROOT).as_posix(), sha256=digest,
                    reference_video=reference, inner_mm=[1000, 1800], outer_mm=[3000, 3000],
                    sequence=1, timing='illustrative; measurement presentation holds',
                    position_policy='Inner LEFT ToF first, outer RIGHT fallback, previous driving estimate otherwise',
                    limitations='Along-track positions estimated; nominal mounts; relative logged headings. Internal straight encoder lengths are not all enforced.',
                    prior_turn_radius_mm=300, fitted_radii_mm=[a['radius'] for a in arcs],
                    snapshots=reports, exports=exports,
                    validation=dict(snapshot_constraints=True, all_rendered_chassis_clear=True,
                                    initial_travel_mm=622, final_travel_mm=591, export_decode=True))
    (OUTPUT/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    (OUTPUT/'README.md').write_text('# Open Challenge: ToF-positioned estimated path\n\n'
        'First complete Open sequence in log 602. Uses the earlier estimated rounded driving path with a fixed 1000 x 1800 mm inner enclosure, checked against the DJI overhead video.\n\n'
        '| View | Standard illustrative pace | 4x illustrative pace |\n| --- | --- | --- |\n'
        '| Field only | [MP4](run_602_tof_positioned_field_only_standard.mp4) | [MP4](run_602_tof_positioned_field_only_4x.mp4) |\n'
        '| Explanatory panel | [MP4](run_602_tof_positioned_with_panel_standard.mp4) | [MP4](run_602_tof_positioned_with_panel_4x.mp4) |\n\n'
        '[Field preview](preview_field_only.png) | [Panel preview](preview_with_panel.png) | [Source and geometry checks](manifest.json)\n\n'
        'Valid inner LEFT ToF sets lateral robot position exactly at each logged snapshot. Valid RIGHT is the fallback where LEFT is unavailable; otherwise the previous lateral estimate continues. The recorded gyro headings and ranges are preserved. The previous 300 mm rounded-turn estimate is retained where feasible and minimally enlarged where necessary for inner rays to hit the finite enclosure face and the chassis to clear the walls. Initial and final logged straight travel (622 and 591 mm) anchor their along-track estimates. Other along-track positions remain estimates; sparse conflicting data cannot establish a unique trajectory.\n\n'
        'At the stop, LEFT 281 mm reaches the inner wall exactly. The modeled RIGHT distance is 250.25 mm versus logged 234 mm, a +16.25 mm residual. Red segments show this outer-wall disagreement; wall geometry and logged values are not stretched to hide it. Nominal mounts are 40 mm forward and +/-35 mm lateral.\n\n'
        'Orange/blue rays show LEFT/RIGHT filtered readings only at their logged poses. Dashed out-of-range rays extend to the outer boundary without a detection endpoint. Robot movement is continuous; brief presentation holds keep measurement rays attached to the correct pose. Camera measurements are not logged. Both pacing choices are illustrative, not recorded time or synchronization to the film. Outputs are silent 30 fps MP4. The second Open sequence remains in the older exports and is not fitted to this first-run film.\n\n'
        'Validation checks all 26 snapshot constraints, chassis clearance for every rendered pose, initial/final travel anchors, original log hash, frame counts and first/middle/final video decoding. Regenerate with `python simulation/render_tof_positioned_open.py --reference-video <video-path>`.\n', encoding='utf-8')
    print(json.dumps(dict(exports=exports, final=reports[-1]['sensors'], reference=reference)), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--reference-video', type=Path)
    render(parser.parse_args().reference_video)
