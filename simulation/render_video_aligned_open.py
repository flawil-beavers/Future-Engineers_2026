#!/usr/bin/env python3
"""Render the filmed Open run using a reviewed video track and rule geometry."""
from __future__ import annotations

import argparse
from bisect import bisect_right
import csv
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import re

import cv2
import numpy as np
from PIL import Image, ImageDraw

from render_open_tof_rays import LEFT, RIGHT, Reading, draw_reading, label, ray
from render_open_video_602 import SOURCE, ARCHIVE, sequences
from render_video_runs import ROOT, FPS, View, Pose, font, robot

OUTPUT = ROOT / 'local_workspace/video-documentation/sensor-usage/open-challenge-video-aligned'
RULES_URL = 'https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf'
font = lru_cache(maxsize=20)(font)


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def sample_pose(track, times, time):
    return Pose(
        round(time * 1000),
        float(np.interp(time, times, [p['x'] for p in track])),
        float(np.interp(time, times, [p['y'] for p in track])),
        float(np.interp(time, times, [p['heading'] for p in track])),
        'Course driving',
        'video track',
    )


def initial_ranges(text):
    chunks = re.split(r'(?=\[STATE CHANGE\] Manual enable -> FOLLOWING)', text)
    first = next(chunk for chunk in chunks if 'Mode OPEN_CHALLENGE completed.' in chunk)
    block = next(block for block in re.split(r'(?=\[STATE CHANGE\])', first)
                 if block.startswith('[STATE CHANGE] Manual enable'))
    result = {}
    for side in ('LEFT', 'RIGHT'):
        match = re.search(rf'{side}:\s+Dist: ([-\d.]+) mm \(Raw: ([-\d.]+)\)', block)
        result[side.lower()] = {'filtered': float(match[1]), 'raw': float(match[2])}
    return result


def background(field_only, extent):
    size = (1080, 1080) if field_only else (1920, 1080)
    image = Image.new('RGB', size, '#f5f4ee' if field_only else '#101c30')
    draw = ImageDraw.Draw(image)
    rectangle = (0, 0, 1080, 1080) if field_only else (55, 170, 940, 1055)
    view = View(rectangle, (-extent, extent, -extent, extent))
    draw.rectangle(rectangle, fill='#f5f4ee')
    view.box(draw, -1500, -1500, 3000, 3000, outline='#344357', width=4)
    view.box(draw, -500, -900, 1000, 1800, fill='#dfe3e6', outline='#536273', width=3)
    if not field_only:
        draw.text((55, 30), 'FUTURE ENGINEERS', font=font(23, True), fill='#6de4d3')
        draw.text((55, 70), 'Open Challenge / filmed trajectory', font=font(36, True), fill='#eff5fc')
        draw.text((55, 120), 'VIDEO-TRACKED POSITION / APPROXIMATE SENSOR SYNCHRONIZATION', font=font(17), fill='#a6b8cf')
        draw.text((1020, 668), 'Inner enclosure 1000 x 1800 mm', font=font(26, True), fill='#eff5fc')
        draw.text((1020, 710), 'Outer field 3000 x 3000 mm', font=font(22), fill='#a6b8cf')
        draw.text((1020, 930), 'Orange LEFT ToF / blue RIGHT ToF', font=font(22, True), fill='#eff5fc')
        draw.text((1020, 970), 'Solid range rays end in crosses. Dashed rays are out of range.', font=font(18), fill='#a6b8cf')
        draw.text((1020, 1004), 'Wheel-based position reconstruction and sensor timing are approximate.', font=font(17), fill='#a6b8cf')
        draw.text((1020, 1036), 'Centre layout follows WRO 2026 pages 11, 27 and 43.', font=font(18), fill='#a6b8cf')
    return image, view


def render(video_path, track_path, output=OUTPUT):
    cv2.setNumThreads(1)
    data = json.loads(track_path.read_text(encoding='utf-8'))
    video_digest = sha256(video_path)
    expected_digest = data.get('source_video_sha256')
    if expected_digest is not None and expected_digest != video_digest:
        raise ValueError('The trajectory belongs to a different source video')
    data['source_video_sha256'] = video_digest
    track = data['track']
    times = np.array([p['t'] for p in track])
    log_text = SOURCE.read_text(encoding='utf-8')
    if sha256(SOURCE) != sha256(ARCHIVE):
        raise ValueError('Log differs from archived original')
    anchors = list(data['sensor_anchors'])
    first = initial_ranges(log_text)
    pose = sample_pose(track, times, 11.2)
    anchors.append(dict(video_seconds=11.2, corner=0, kind='initial', x=pose.x, y=pose.y,
                        heading=0, **first))
    stop = data['video_stop_seconds']
    pose = sample_pose(track, times, stop)
    result = sequences(log_text)[0]['stop']
    anchors.append(dict(video_seconds=stop, corner=12, kind='stop', x=pose.x, y=pose.y,
                        heading=result['heading'], left=result['left'], right=result['right']))
    anchors.sort(key=lambda record: record['video_seconds'])
    anchor_times = [record['video_seconds'] for record in anchors]
    extent = 1580
    for record in anchors:
        for side in ('left', 'right'):
            _, endpoint, valid = ray(record['x'], record['y'], record['heading'],
                                     side, record[side]['filtered'])
            if valid:
                extent = max(extent, max(abs(value) for value in endpoint) + 60)
    print(f'Finite-endpoint view extent {extent:.1f} mm', flush=True)
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise ValueError('Source video could not be opened')
    video_fps = cap.get(cv2.CAP_PROP_FPS)
    source_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = source_frames / video_fps
    output.mkdir(parents=True, exist_ok=True)
    writers = {}
    paths = {}
    temporary_paths = {}
    scenes = {mode: background(mode, extent) for mode in (False, True)}
    for mode in (False, True):
        for speed in (1, 4):
            name = f'open_1318_{"field_only" if mode else "with_video_panel"}_{speed}x.mp4'
            path = output / name
            temporary_path = output / (path.stem + '.rendering.mp4')
            size = (1080, 1080) if mode else (1920, 1080)
            writer = cv2.VideoWriter(str(temporary_path), cv2.VideoWriter_fourcc(*'mp4v'), FPS, size)
            if not writer.isOpened():
                raise RuntimeError('Video encoder unavailable')
            paths[mode, speed] = path
            temporary_paths[mode, speed] = temporary_path
            writers[mode, speed] = writer
    trails = {mode: Image.new('RGBA', scene.size) for mode, (scene, view) in scenes.items()}
    count = math.ceil(duration * FPS)
    previous = None
    source_index = -1
    source_image = None
    previews = set()
    try:
        for frame in range(0, count, 2):
            time = frame / FPS
            pose = sample_pose(track, times, time)
            target_index = min(source_frames - 1, round(time * video_fps))
            while source_index < target_index:
                ok, image = cap.read()
                if not ok:
                    raise RuntimeError(f'Source frame {source_index + 1} could not be decoded')
                source_image = image
                source_index += 1
            index = bisect_right(anchor_times, time) - 1
            record = anchors[index] if index >= 0 else None
            active = record is not None and time - record['video_seconds'] <= 1
            for mode, (scene, view) in scenes.items():
                draw = ImageDraw.Draw(trails[mode])
                point = view.pt(pose.x, pose.y)
                if previous is not None:
                    draw.line([view.pt(*previous), point], fill='#10ad9a', width=4)
                image = scene.copy()
                image.paste(trails[mode], (0, 0), trails[mode])
                draw = ImageDraw.Draw(image)
                if active:
                    reading = Reading(0, 1, record['corner'], record['x'], record['y'],
                                      record['heading'], record['left'], record['right'], record['kind'])
                    draw_reading(draw, view, reading)
                robot(draw, view, pose, '#10ad9a', rear=40, width=135)
                if not mode:
                    inset = Image.fromarray(cv2.cvtColor(source_image, cv2.COLOR_BGR2RGB))
                    inset = inset.resize((840, 473))
                    image.paste(inset, (1010, 180))
                    draw = ImageDraw.Draw(image)
                    draw.text((1020, 755), f'Film time {time:06.2f} s', font=font(24, True), fill='#eff5fc')
                    draw.text((1020, 800), 'LEFT  ' + (label(record['left']['filtered']) if record else 'awaiting snapshot'), font=font(23), fill=LEFT)
                    draw.text((1020, 842), 'RIGHT ' + (label(record['right']['filtered']) if record else 'awaiting snapshot'), font=font(23), fill=RIGHT)
                    draw.text((1020, 884), 'Range snapshot visible' if active else 'Latest snapshot retained in readout', font=font(19), fill='#a6b8cf')
                pixels = np.asarray(image)
                for speed in (1, 4):
                    if speed == 4 and frame % 4:
                        continue
                    rendered = pixels.copy()
                    if not mode:
                        cv2.putText(rendered, f'{speed}x playback', (1650, 110),
                                    cv2.FONT_HERSHEY_SIMPLEX, .8, (166, 184, 207), 2, cv2.LINE_AA)
                    bgr = cv2.cvtColor(rendered, cv2.COLOR_RGB2BGR)
                    repeats = min(2, count - frame) if speed == 1 else 1
                    for _ in range(repeats):
                        writers[mode, speed].write(bgr)
                if active and record['corner'] == 5 and mode not in previews:
                    image.save(output / f'preview_{"field_only" if mode else "with_video_panel"}.png')
                    previews.add(mode)
            previous = (pose.x, pose.y)
            if frame % (FPS * 20) == 0:
                print(f'Video-aligned Open run {time:.0f}/{duration:.2f} s', flush=True)
    finally:
        cap.release()
        for writer in writers.values():
            writer.release()
    exports = []
    for (mode, speed), path in paths.items():
        temporary_path = temporary_paths[mode, speed]
        cap = cv2.VideoCapture(str(temporary_path))
        frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        expected_shape = (1080, 1080) if mode else (1080, 1920)
        for frame in (0, frames // 4, frames // 2, frames - 1):
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame)
            ok, image = cap.read()
            if not ok or image.shape[:2] != expected_shape:
                raise RuntimeError(f'Export verification failed for {path.name}')
        cap.release()
        temporary_path.replace(path)
        exports.append(dict(file=path.name, field_only=mode, speed=speed,
                            frames=frames, seconds=frames / FPS))
    with (output / 'trajectory.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(track[0]))
        writer.writeheader()
        writer.writerows(track)
    (output / 'calibration_and_alignment.json').write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8')
    (output / 'sensor_snapshots.json').write_text(json.dumps(anchors, indent=2) + '\n', encoding='utf-8')
    manifest = dict(video_basename=video_path.name, video_sha256=video_digest,
                    source_log=ARCHIVE.relative_to(ROOT).as_posix(), source_log_sha256=sha256(ARCHIVE),
                    log_sequence=1, association='First starting clearance and first straight match the film',
                    inner_enclosure_mm=[1000, 1800], outer_field_mm=[3000, 3000],
                    field_view_half_extent_mm=extent,
                    rules_url=RULES_URL, rules_pages=[11, 27, 43],
                    rules_pdf_sha256='c23ba063c548e74812ffd9ed580931b2d4264541261b5f8eaff643a1e3201043',
                    pose_reference='Video wheel-contact proxy, nominal wheelbase/track correction to rear axle',
                    timing='Film time for trajectory, approximate event alignment for untimed log records', exports=exports)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    (output / 'README.md').write_text(
        '# Open Challenge filmed trajectory\n\n'
        'The inner enclosure is 1000 x 1800 mm within a 3000 x 3000 mm field. '
        'The layout follows the filmed rectangle and WRO 2026 Open Challenge dimensions. '
        f'[Official rules, pages 11, 27 and 43]({RULES_URL}).\n\n'
        '| View | 1x film time | 4x |\n| --- | --- | --- |\n'
        '| Field only, 1080 square | [1x](open_1318_field_only_1x.mp4) | [4x](open_1318_field_only_4x.mp4) |\n'
        '| With source-film panel, 1920 x 1080 | [1x](open_1318_with_video_panel_1x.mp4) | [4x](open_1318_with_video_panel_4x.mp4) |\n\n'
        '[Field preview](preview_field_only.png) | [Video-panel preview](preview_with_video_panel.png)\n\n'
        'These exports cover the single filmed run in MVI_1318_OPEN_video1.MP4, matched to '
        'the first Open sequence in log 602 by the starting clearances and first straight. '
        'Trajectory time follows the film. Positions are reconstructed from visible wheel contacts '
        'using a perspective and radial fit to printed mat markers, with a nominal wheelbase of '
        '100 mm and wheel track of 123 mm. Heading is smoothed from forward travel and stabilized '
        'on the visible straight sections. The operator occlusion at the start and one isolated '
        'false detection are interpolated. The final stationary pose is held through the film ending.\n\n'
        'The reconstruction provides approximate rear-axle positions. The oblique camera, '
        'manual calibration points, wheel-contact estimate and nominal chassis dimensions limit '
        'metric precision. The 1000 x 1800 mm rectangle is the rule-based layout selected from the film. '
        'Recorded sensor events have no timestamps. Their positions/times are estimated from '
        'the filmed turns and logged straight encoder travel. Sensor endpoints therefore remain '
        'approximate projections. Finite ranges are preserved, including discrepancies at corners.\n\n'
        'Orange is LEFT ToF and blue is RIGHT ToF. Solid rays end at each logged filtered range. '
        'Dashed rays extend to the field boundary for unavailable/out-of-range readings. '
        'The source-film panel provides a frame-matched view for checking the trajectory. '
        'All exports are silent 30 fps MP4 files. The field-only versions contain the field, '
        'robot, trail and active rays.\n\n'
        'The manifest records both source hashes and the rules reference. Calibration, '
        'trajectory CSV and event-alignment files are included alongside the clips.\n',
        encoding='utf-8',
    )
    print(json.dumps(exports), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', required=True, type=Path)
    parser.add_argument('--track', required=True, type=Path)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    args = parser.parse_args()
    render(args.video, args.track, args.output)
