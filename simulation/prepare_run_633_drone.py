"""Cache and track the filmed run spanning DJI0066 and DJI0067."""
from pathlib import Path
import argparse
import json
import math

import cv2
import numpy as np
from scipy.ndimage import gaussian_filter1d

from render_video_runs import ROOT, parse

OUTPUT = ROOT / 'local_workspace/run-633-drone'
BATCH = ROOT / 'local_workspace/video-documentation/stabilized-dji-batch-0066-0068'
SOURCES = ('DJI_20261031175628_0066_D', 'DJI_20261031180153_0067_D')
WORLD_CORNERS = np.float32([[-500, 500], [500, 500], [500, -500], [-500, -500]])
CALIBRATION = {
    66: dict(time=280., corners=[[2042, 519], [2897, 496], [2896, 1343], [2057, 1340]]),
    67: dict(time=0., corners=[[2012, 528], [2869, 505], [2873, 1352], [2025, 1352]]),
}
FIELD_IMAGE = np.array([[.3, 0, 500], [0, -.3, 500], [0, 0, 1.]])


def camera_at(data, index):
    records = data['transforms']
    position = min(index // 2, len(records)-1)
    left = records[position]
    if position == len(records)-1 or index == left['frame']:
        return np.array(left['H'])
    right = records[position+1]
    fraction = (index-left['frame'])/(right['frame']-left['frame'])
    return np.array(left['H'])*(1-fraction)+np.array(right['H'])*fraction


def calibration():
    result = {}
    for number, name in zip((66, 67), SOURCES):
        data = json.loads((BATCH / name / 'camera_transforms.json').read_text())
        spec = CALIBRATION[number]
        camera = camera_at(data, round(spec['time'] * data['fps']))
        points = np.float32(spec['corners']) / 4
        reference_points = cv2.perspectiveTransform(points.reshape(-1, 1, 2), camera)
        world = cv2.getPerspectiveTransform(reference_points.astype(np.float32).reshape(-1, 2), WORLD_CORNERS)
        result[number] = dict(data=data, world=world)
    return result


def cache_frames(video_root):
    OUTPUT.mkdir(parents=True, exist_ok=True)
    cache = OUTPUT / 'frames'
    cache.mkdir(exist_ok=True)
    calibrations = calibration()
    rows = []
    first_duration = calibrations[66]['data']['frames'] / calibrations[66]['data']['fps']
    common_world = calibrations[66]['world']
    for number, name in zip((66, 67), SOURCES):
        info = calibrations[number]
        data = info['data']
        fps = data['fps']
        first = round(275 * fps) if number == 66 else 0
        stop = data['frames'] if number == 66 else round(156 * fps)
        capture = cv2.VideoCapture(str(video_root / (name + '.MP4')))
        assert capture.isOpened()
        capture.set(cv2.CAP_PROP_POS_FRAMES, first)
        stitch = np.linalg.inv(common_world) @ info['world']
        for index in range(first, stop):
            ok = capture.grab()
            assert ok, (name, index)
            if index % 2:
                continue
            ok, image = capture.retrieve()
            assert ok
            small = cv2.resize(image, (960, 540))
            matrix = stitch @ camera_at(data, index)
            stabilized = cv2.warpPerspective(small, matrix, (960, 540))
            filename = f'{number}_{index:06}.jpg'
            cv2.imwrite(str(cache / filename), stabilized, [cv2.IMWRITE_JPEG_QUALITY, 92])
            time = index / fps
            rows.append(dict(video=number, frame=index, source_seconds=time,
                             time=time if number == 66 else first_duration+time,
                             image=f'frames/{filename}'))
            if index % 600 == 0:
                print(f'Caching {number}: {time:.1f}s', flush=True)
        capture.release()
    data = dict(frames=rows, fps=calibrations[66]['data']['fps'],
                seam_seconds=first_duration, calibration=CALIBRATION,
                image_to_world=common_world.tolist(),
                sources=[dict(file=name+'.MP4', sha256=calibrations[number]['data']['sha256'])
                         for number, name in zip((66, 67), SOURCES)])
    (OUTPUT / 'film_frames.json').write_text(json.dumps(data, indent=2))
    return data


def track(data):
    rows = data['frames']
    matrix = FIELD_IMAGE @ np.array(data['image_to_world'])
    samples = []
    for index in np.linspace(0, len(rows)-1, 45, dtype=int):
        image = cv2.imread(str(OUTPUT / rows[index]['image']))
        samples.append(cv2.warpPerspective(image, matrix, (1000, 1000)))
    background = np.median(samples, axis=0).astype(np.uint8)
    cv2.imwrite(str(OUTPUT / 'field_background.jpg'), background)
    yy, xx = np.indices((1000, 1000))
    wx, wy = (xx-500)/.3, (500-yy)/.3
    region = ((np.abs(wx) < 1460) & (np.abs(wy) < 1460)
              & ((np.abs(wx) > 540) | (np.abs(wy) > 540)))
    candidates = []
    positions = []
    times = []
    last = np.array([350., -1360.])
    previous_time = rows[0]['time']
    for index, row in enumerate(rows):
        image = cv2.imread(str(OUTPUT / row['image']))
        field = cv2.warpPerspective(image, matrix, (1000, 1000))
        gray = cv2.cvtColor(field, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(field, cv2.COLOR_BGR2HSV)
        delta = background.astype(np.int16).mean(axis=2) - field.astype(np.int16).mean(axis=2)
        binary = ((gray < 125) & (hsv[:, :, 1] < 165) & (delta > 25) & region).astype(np.uint8)*255
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        count, labels, stats, centers = cv2.connectedComponentsWithStats(binary)
        choices = []
        for component in range(1, count):
            area = int(stats[component, cv2.CC_STAT_AREA])
            width, height = stats[component, 2:4]
            if 65 <= area <= 2500 and max(width, height) < 95 and min(width, height) >= 5:
                pixel = centers[component]
                world = np.array([(pixel[0]-500)/.3, (500-pixel[1])/.3])
                choices.append(dict(x=float(world[0]), y=float(world[1]), area=area))
        candidates.append(dict(t=row['time'], choices=choices))
        if choices:
            available = [value for value in choices if math.hypot(value['x']-last[0], value['y']-last[1])
                         < max(180, (row['time']-previous_time)*650)]
            if available:
                chosen = max(available, key=lambda value: value['area'])
                last = np.array([chosen['x'], chosen['y']])
                previous_time = row['time']
                positions.append(last)
                times.append(row['time'])
        if index % 450 == 0:
            print('Detecting robot', index, '/', len(rows), flush=True)
    assert len(positions) > len(rows)*.65, (len(positions), len(rows))
    uniform = np.array([row['time'] for row in rows])
    xy = np.column_stack([np.interp(uniform, times, np.array(positions)[:, axis]) for axis in (0, 1)])
    xy = gaussian_filter1d(xy, 2, axis=0)
    velocity = np.gradient(xy, uniform, axis=0)
    heading = np.rad2deg(np.arctan2(velocity[:, 1], velocity[:, 0]))
    speed = np.linalg.norm(velocity, axis=1)
    track = [dict(t=float(time), x=float(p[0]), y=float(p[1]), heading=float(h), speed=float(s))
             for time, p, h, s in zip(uniform, xy, heading, speed)]
    result = dict(**{key:value for key,value in data.items() if key != 'frames'},
                  track=track, detected_frames=len(positions), cached_frames=len(rows))
    (OUTPUT / 'candidates.json').write_text(json.dumps(candidates))
    (OUTPUT / 'video_track.json').write_text(json.dumps(result, indent=2))
    print('Detection coverage', len(positions), '/', len(rows), 'max speed', speed.max(), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video-root', type=Path)
    parser.add_argument('--reuse-cache', action='store_true')
    args = parser.parse_args()
    cv2.setNumThreads(2)
    if args.reuse_cache:
        data = json.loads((OUTPUT / 'film_frames.json').read_text())
    else:
        if args.video_root is None:
            parser.error('--video-root is required to extract footage')
        data = cache_frames(args.video_root)
    track(data)
