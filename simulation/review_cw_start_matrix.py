"""Historical CW evidence/visibility and shorter-start geometry prototype.

For current implemented firmware readiness run check_cw_start_planner.py and
check_cw_start_state.py. This historical study does not execute firmware gates.
"""
import hashlib
import itertools
import json
import math
import re
from pathlib import Path

from parking_entry_scout_sim import camera_geometry, scout_pose
from connector_transition_sim import curve, simulate
from parking_exit_swept_search import Pose, collision, robot_polygons

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / 'simulation/evidence/parking_exit_diagnostics'


def review(number):
    path = EVIDENCE / f'20261005_log_{number}_cw.txt'
    text = path.read_text(encoding='utf-8', errors='replace')
    match = re.search(r'pose_x_y_heading=([\d.-]+)/([\d.-]+)/([\d.-]+)', text)
    report = dict(log=path.relative_to(ROOT).as_posix(),
                  sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  start_confirmations=re.findall(
                      r'\[MAP\] Confirmed S0 station=(\d+) side=(\w+) color=(\w+)', text),
                  connector_completed='[PARK ENTRY CONNECTOR] Complete' in text,
                  lap_completed='[PATH] Completed lap 1' in text)
    if match:
        pose = tuple(map(float, match.groups()))
        report['near_primary_bearing_range'] = camera_geometry(pose, (500, -900))
        report['near_scout_bearing_range'] = camera_geometry(
            scout_pose(*pose, 'CW', 85), (500, -900))
    exit_match = re.search(r'type=sample.*state=localize_drive.*pose=([^ ]+)', text)
    if exit_match:
        start = tuple(map(float, exit_match.group(1).split(',')))
        # Proposed replacement for the long reverse localization, not an
        # addition after it. Retracing the last 130 mm returns to the scan.
        poses = [scout_pose(*start, 'CW', travel) for travel in range(0, 196, 2)]
        primary = scout_pose(*start, 'CW', 65)
        near = scout_pose(*start, 'CW', 195)
        pb, pr = camera_geometry(primary, (0, -900))
        nb, nr = camera_geometry(near, (500, -900))
        pink = any(collision(Pose(480-x, y+1500, 180-h), 50, gap)
                   for x, y, h in poses for gap in (242.5, 252.5))
        maximum_x = max(
            480-min(px for polygon in robot_polygons(
                Pose(480-x, y+1500, 180-h), 50) for px, _ in polygon)
            for x, y, h in poses)
        report['proposed_short_approach'] = dict(
            logged_prelocalization_pose=start, primary_scan_pose=primary,
            middle_bearing_range=[pb, pr], near_bearing_range=[nb, nr],
            sampled_pink_collision=pink, maximum_body_field_x_mm=maximum_x,
            geometry_pass=not pink and maximum_x < 500 and
            abs(pb) <= 26.426 and abs(nb) <= 26.426 and
            230 <= pr <= 600 and 230 <= nr <= 600,
            retains_second_marker_localization=False, implemented=False)
    return report


def shorter_scout():
    cases = []
    for dx, dy, dh in itertools.product((-10, 0, 10), (-10, 0, 10), (-2, 0, 2)):
        start = (300+dx, -1220+dy, 151.1+dh)
        poses = [scout_pose(*start, 'CW', travel) for travel in range(0, 131, 2)]
        bearing, distance = camera_geometry(poses[-1], (500, -900))
        pink = any(collision(Pose(480-x, y+1500, 180-h), 50, gap)
                   for x, y, h in poses for gap in (242.5, 252.5))
        # Same conservative full-lock wheel shapes as the pink check.
        maximum_field_x = max(
            480-min(px for polygon in robot_polygons(
                Pose(480-x, y+1500, 180-h), 50) for px, _ in polygon)
            for x, y, h in poses)
        cases.append(dict(shift=[dx, dy, dh], near_bearing_deg=bearing,
                          near_range_mm=distance, sampled_pink_collision=pink,
                          maximum_body_field_x_mm=maximum_field_x,
                          primary_middle_bearing_range=camera_geometry(start, (0, -900)),
                          passed=not pink and abs(bearing) <= 26.426 and
                          230 <= distance <= 600 and maximum_field_x < 500))
    return dict(nominal_scan_pose=[300, -1220, 151.1], reverse_scout_mm=130,
                cases=cases, pass_count=sum(c['passed'] for c in cases),
                total=len(cases), implemented=False,
                missing='Complete measured localization-to-scan approach and connector rollout')


def known_layout_route(layout):
    # Isolated south straight + first R500 corner. As in firmware, displacement
    # changes XY, but not the saved baseline tangent metadata.
    route = []
    for distance in range(0, 1801, 50):
        if distance <= 1000:
            x, y, heading = 500-distance, -1000, 180
        else:
            angle = (distance-1000)/500
            x, y, heading = -500-500*math.sin(angle), -500-500*math.cos(angle), 180-math.degrees(angle)
        route.append(dict(x=x, y=float(y), h=heading))
    for station, color in layout:
        if station == 0:
            continue  # Stored for later approach, not bypassed initially.
        center = station*10
        displacement = -360 if color == 'RED' else (100 if station == 2 else 160)
        for offset in range(-8, 9):
            route[center+offset]['y'] -= displacement*(1-abs(offset)/9)
        old = [p.copy() for p in route]
        for index in range(center-9, center+10):
            for axis in ('x', 'y'):
                route[index][axis] = sum(old[index+j][axis] for j in (-1, 0, 1))/3
    return route


def short_connector_study(layouts):
    start = dict(zip(('x', 'y', 'h'), scout_pose(243, -1214, 180, 'CW', 65)))
    rows = []
    for layout in layouts:
        route = known_layout_route(layout)
        # Keep the near station collision guard even when it is stored-only.
        pillars = [(500, -900)] + [(500-station*500, -900)
                                   for station, _ in layout if station != 0]
        middle = any(station == 1 for station, _ in layout)
        result = None
        for index in range(3, 11 if not middle else 9):
            end = dict(route[index])
            end['h'] = math.degrees(math.atan2(route[index+1]['y']-end['y'],
                                             route[index+1]['x']-end['x']))
            chord = math.hypot(end['x']-start['x'], end['y']-start['y'])
            count = max(3, min(64, math.ceil(chord/25)+1))
            connector = curve(start, end, count)
            for lookahead in (150, 125, 100, 82.5, 57.5, 32.5):
                rollout = simulate(connector, route[index:], L=lookahead,
                                   continue_route=True, check_pink=True,
                                   pillar_seats=pillars)
                if rollout['status'] == 'pass':
                    result = dict(merge_x_mm=end['x'], lookahead_mm=lookahead,
                                  rollout=rollout)
                    break
            if result:
                break
        rows.append(dict(layout=layout, candidate=result))
    return dict(start_pose=start, cases=rows,
                candidate_count=sum(row['candidate'] is not None for row in rows),
                total=len(rows), implemented=False,
                limitations=['All colours assumed known; recognition/injection timing not replayed',
                             'Endpoint search explores candidates beyond current firmware selection',
                             'Future shorter-connector minimum-forward gate is not implemented',
                             'Nominal connector only, not physical tracking or later-lap validation'])


def main():
    # Six singleton cases and four end-pair colour combinations after Figure
    # 8e relocation. The empty case is an extra diagnostic regression.
    layouts = [(0, 'GREEN'), (0, 'RED'), (1, 'GREEN'), (1, 'RED'),
               (2, 'GREEN'), (2, 'RED')]
    official_layouts = [[item] for item in layouts] + [
        [(0, first), (2, last)]
        for first, last in itertools.product(('GREEN', 'RED'), repeat=2)]
    report = dict(official_inner_layouts=official_layouts,
                  evidence=[review(n) for n in range(426, 446) if n != 429],
                  shorter_scout_design=shorter_scout(),
                  shorter_connector_design=short_connector_study(official_layouts),
                  rules={'passing_side': '9.19 and Appendix A.5 apply on a full directional crossing',
                         'behind_start': 'No extra approach needed; observe/store for the later encounter',
                         'no_blanket_exception': 'The PDF does not exempt a new full crossing created by reversing'},
                  limitations=['No raw frames in the failed driving logs',
                               'Candidate isolated scout uses ideal sampled kinematics',
                               'This historical prototype does not execute the current implemented firmware planner'])
    destination = ROOT / 'local_workspace/cw-start-matrix/report.json'
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2)+'\n')
    print('Archived driving logs reviewed:', len(report['evidence']))
    print('Unique inner-row layouts:', len(official_layouts))
    scout = report['shorter_scout_design']
    print('Isolated shorter scout:', scout['pass_count'], '/', scout['total'])
    approaches = [v['proposed_short_approach'] for v in report['evidence']
                  if 'proposed_short_approach' in v]
    print('Short approach from logged exit poses:',
          sum(v['geometry_pass'] for v in approaches), '/', len(approaches))
    connectors = report['shorter_connector_design']
    print('Nominal shorter connector candidates:', connectors['candidate_count'], '/', connectors['total'])
    print('Historical prototype only; current implementation checks: check_cw_start_planner.py / check_cw_start_state.py')


if __name__ == '__main__':
    main()
