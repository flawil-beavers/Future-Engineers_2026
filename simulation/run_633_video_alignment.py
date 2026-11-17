"""Align filmed motion to run633 and preserve its logged pose anchors."""
from pathlib import Path
import json
import math

import cv2
import numpy as np
from scipy.ndimage import gaussian_filter1d

from render_video_runs import ROOT, Pose, parse
from prepare_run_633_drone import OUTPUT, FIELD_IMAGE

ARCHIVE = ROOT / 'simulation/evidence/parking_exit_diagnostics/20261101_log_633_ccw.txt'


def wrap(value):
    return (value + 180) % 360 - 180


def heading_from_images(film, track):
    rows = film['frames']
    projection = FIELD_IMAGE @ np.array(film['image_to_world'])
    positions = np.array([[p['x'], p['y']] for p in track])
    reference = cv2.imread(str(OUTPUT / rows[0]['image']))
    reference = cv2.warpPerspective(reference, projection, (1000, 1000))
    first = positions[0]
    center = (float(500 + .3*first[0]), float(500 - .3*first[1]))
    template = cv2.getRectSubPix(reference, (64, 64), center)
    cv2.imwrite(str(OUTPUT / 'robot_reference.jpg'), template)
    yy, xx = np.indices((64, 64))
    radius = np.hypot(xx-31.5, yy-31.5)
    weights = (np.exp(-(radius/26)**8) * (radius<29)).astype(np.float32)[:, :, None]
    angles = np.arange(0, 360, 5)
    templates = []
    def vector(image):
        value = image.astype(np.float32)
        mean = (value*weights).sum(axis=(0,1)) / weights.sum()
        value = ((value-mean)*weights).ravel()
        return value / max(float(np.linalg.norm(value)), 1e-9)
    for angle in angles:
        matrix = cv2.getRotationMatrix2D((31.5,31.5), float(angle), 1)
        rotated = cv2.warpAffine(template, matrix, (64,64), borderValue=(225,225,225))
        templates.append(vector(rotated))
    templates = np.array(templates)
    scores = []
    for index, (row, position) in enumerate(zip(rows, positions)):
        image = cv2.imread(str(OUTPUT / row['image']))
        image = cv2.warpPerspective(image, projection, (1000,1000))
        center = (float(500+.3*position[0]), float(500-.3*position[1]))
        patch = cv2.getRectSubPix(image, (64,64), center)
        scores.append(templates @ vector(patch))
        if index % 600 == 0:
            print('Robot orientation', index, '/', len(rows), flush=True)
    scores = np.array(scores)
    changes = wrap(angles[:,None] - angles[None,:])
    transition = .015 * (changes/12)**2
    transition[np.abs(changes)>20] = 1e6
    previous = 1-scores[0] + .01*(wrap(angles)/30)**2
    back = np.zeros((len(rows), len(angles)), np.int16)
    for index in range(1, len(rows)):
        costs = previous[:,None] + transition
        back[index] = costs.argmin(axis=0)
        previous = costs.min(axis=0) + 1-scores[index]
    selected = np.zeros(len(rows), np.int16)
    selected[-1] = previous.argmin()
    for index in range(len(rows)-1,0,-1):
        selected[index-1] = back[index,selected[index]]
    heading = np.rad2deg(np.unwrap(np.deg2rad(angles[selected])))
    heading = gaussian_filter1d(heading, 1)
    confidence = scores[np.arange(len(rows)),selected]
    for record, angle, score in zip(track, heading, confidence):
        record['body_heading'] = float(angle)
        record['orientation_score'] = float(score)
    return track


def find_offset(track, poses):
    times = np.array([p['t'] for p in track])
    centers = np.array([[p['x'],p['y']] for p in track])
    angles = np.array([p['body_heading'] for p in track])
    rear_axle = centers - 42.5*np.column_stack([np.cos(np.deg2rad(angles)),np.sin(np.deg2rad(angles))])
    selected = [p for p in poses['samples'] if p.source == 'LATER_TRACK'][::3]
    log_times = np.array([(p.t-poses['start'])/1000 for p in selected])
    log_xy = np.array([[p.x,p.y] for p in selected])
    log_h = np.array([p.h for p in selected])
    candidates = []
    for offset in np.arange(275, 290, .025):
        xy = np.column_stack([np.interp(log_times+offset,times,rear_axle[:,a]) for a in (0,1)])
        h = np.interp(log_times+offset,times,angles)
        errors = log_xy-xy
        translation = np.median(errors,axis=0)
        position_error = np.linalg.norm(errors-translation,axis=1)
        angle_error = np.abs(wrap(log_h-h))
        score = np.median(position_error) + 2*np.median(angle_error)
        candidates.append(dict(offset=float(offset), score=float(score),
                               median_position_error_mm=float(np.median(position_error)),
                               median_heading_error_deg=float(np.median(angle_error)),
                               translation_mm=translation.tolist()))
    return sorted(candidates,key=lambda p:p['score'])


def build_alignment(reuse_orientation=False):
    film = json.loads((OUTPUT / 'film_frames.json').read_text())
    path = OUTPUT / 'video_track.json'
    data = json.loads(path.read_text())
    track = data['track']
    if not reuse_orientation or 'body_heading' not in track[0]:
        track = heading_from_images(film, track)
        data['track'] = track
        path.write_text(json.dumps(data,indent=2))
    log = parse(ARCHIVE)
    candidates = find_offset(track,log)
    result = dict(run=633, offset_seconds=candidates[0]['offset'],
                  duration_seconds=(log['finish']-log['start'])/1000,
                  seam_video_seconds=film['seam_seconds'],
                  leading_offset_candidates=candidates[:8],
                  position_policy='Logged poses retained; video-shaped gaps are anchored at their logged endpoints',
                  distance_policy='Displayed ToF lengths projected to scene walls and parking rails at user request')
    (OUTPUT / 'alignment.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2),flush=True)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reuse-orientation',action='store_true')
    arguments = parser.parse_args()
    cv2.setNumThreads(2)
    build_alignment(arguments.reuse_orientation)
