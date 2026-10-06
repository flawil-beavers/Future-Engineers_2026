"""Reproduce start-case geometry estimates; no physical acceptance is implied.

Reports stay in local_workspace. RED candidates are rejected if they pass on
the wrong side even when collision and handoff checks succeed.
"""
import hashlib
import json
import math
from pathlib import Path

from analyze_connector_tracking import fields
from connector_transition_sim import curve, simulate
from parking_entry_scout_sim import clearance
from parking_exit_swept_search import Pose, collision, robot_polygons

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / 'simulation/evidence/parking_exit_diagnostics'


def green_path(clearance_mm):
    # Isolated CW seat4: straight west then R500 corner. 50 mm samples,
    # +/-8 displacement points and radius-1 smoothing, as in firmware.
    points = []
    for distance in range(-700, 1201, 50):
        if distance <= 0:
            x, y = -500-distance, -1000.0
        else:
            angle = distance/500
            x, y = -500-500*math.sin(angle), -500-500*math.cos(angle)
        if abs(distance) <= 400:
            y -= (clearance_mm-100)*(1-abs(distance)/450)
        points.append((x, y))
    original = points[:]
    center = 14
    for index in range(center-9, center+10):
        points[index] = tuple(sum(original[index+j][axis] for j in (-1, 0, 1))/3
                              for axis in (0, 1))
    wall_min = pillar_min = math.inf
    # Evaluate only the straight and its first quarter-circle.
    for index in range(1, len(points)-1):
        if (index-center)*50 > math.pi*250:
            break
        x, y = points[index]
        heading = math.atan2(points[index+1][1]-points[index-1][1],
                             points[index+1][0]-points[index-1][0])
        for rear in (0, 70):
            wall, pillar = clearance((x-rear*math.cos(heading),
                                      y-rear*math.sin(heading),
                                      math.degrees(heading)), (-500, -900))
            wall_min, pillar_min = min(wall_min, wall), min(pillar_min, pillar)
    return dict(clearance_setting_mm=clearance_mm, peak_xy_mm=points[center],
                minimum_sampled_capsule_wall_mm=wall_min,
                minimum_sampled_capsule_pillar_mm=pillar_min)


def pink_replay(number, shift_mm=0):
    path = EVIDENCE / f'20261005_log_{number}_cw.txt'
    samples = []
    for line in path.read_text().splitlines():
        if not line.startswith('[PARK_DIAG]'):
            continue
        values = fields(line)
        if values.get('type') != 'sample' or values.get('state') != 'localize_drive':
            continue
        x, y, heading = map(float, values['pose'].split(','))
        pose = Pose(480-x, y+1500+shift_mm, 180-heading)
        steer = -int(values['steer'])
        outline = robot_polygons(pose, steer)
        samples.append((min(y for polygon in outline for _, y in polygon)-200,
                        any(collision(pose, steer, gap)
                            for gap in (242.5, 247.5, 252.5))))
    return dict(log=path.relative_to(ROOT).as_posix(),
                sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                hypothetical_inward_shift_mm=shift_mm, samples=len(samples),
                minimum_outline_above_pink_end_mm=min(v[0] for v in samples),
                sampled_collision_count=sum(v[1] for v in samples))


def red_counterfactual():
    # Start inferred from rounded log436 preflight forward/lateral values.
    # Modified tangent/travel limit is an OFFLINE rejected candidate only.
    start = dict(x=626.2326, y=-1208.9965, h=153.3)
    end = dict(x=100, y=-960, h=208.07)
    connector = curve(start, end, 24, end_scale=1.0)
    route = [end, dict(x=50, y=-986.6667, h=208.07),
             dict(x=0, y=-1000, h=180), dict(x=-50, y=-1000, h=180)]
    trace = []
    result = simulate(connector, route, L=125, continue_route=True,
                      max_travel_mm=700, pillar_seats=((500, -900),), trace=trace)
    crossing = next(p for p in trace if p['x'] <= 500)
    # Westward travel: right of RED means north (Y > -900).
    correct_side = crossing['y'] > -900
    return dict(kinematic_result=result, first_axle_crossing=crossing,
                correct_red_side=correct_side,
                firmware_candidate_accepted=False,
                reason='Wrong RED side; longer travel alone cannot fix the route')


def main():
    report = dict(green_nominal=[green_path(v) for v in (260, 200)],
                  pink_logged=[pink_replay(v) for v in (439, 440, 441, 442)],
                  pink_441_counterfactual=[pink_replay(441, v) for v in (10, 20)],
                  red_near_rejected=red_counterfactual(),
                  limitations=['Estimated rounded poses, approximate robot geometry',
                               'Sampled outlines do not prove continuous clearance',
                               'GREEN nominal route is not a closed-loop physical replay',
                               'Pink shifts are calculations, not implemented motion'])
    destination = ROOT / 'local_workspace/start-geometry/report.json'
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    assert not report['red_near_rejected']['correct_red_side']


if __name__ == '__main__':
    main()
