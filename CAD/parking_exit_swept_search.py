"""Offline conservative parking-exit search for the current prototype.

Coordinates:
  x: official driving direction, rear marker inner face at x=0
  y: away from the outer black wall, parking opening at y=200
  pose: rear-axle midpoint and chassis heading

The search uses an inflated multi-part robot footprint and constant-curvature
Ackermann motion primitives. It is a feasibility tool, not firmware. See
PARKING_EXIT_PATH_SIMULATION.md for its assumptions and validation workflow.
"""

from __future__ import annotations

import heapq
import math
from pathlib import Path
from dataclasses import dataclass


ROBOT_FRONT_MM = 125.0
ROBOT_REAR_MM = 40.0
ROBOT_HALF_WIDTH_MM = 67.5
TRACK_HALF_WIDTH_MM = 50.0
WHEEL_DIAMETER_MM = 43.2
WHEEL_WIDTH_MM = 25.0
PHYSICAL_WHEELBASE_MM = 100.0
PARKING_DEPTH_MM = 200.0
MARKER_THICKNESS_MM = 20.0
NOMINAL_GAP_MM = 247.5
MIN_GAP_MM = 242.5
PLACEMENT_ERROR_MM = 5.0
HEADING_ERROR_DEG = 1.0
WALL_SAFETY_MARGIN_MM = 5.0
MARKER_FACE_SAFETY_MM = 5.0
# The magenta pieces are physically 200 mm long. Do not extend their open ends
# by the general 5 mm face margin. Its modeled open end remains at the measured
# 200 mm length.
MARKER_END_SAFETY_MM = 0.0

# Approximate rear-axle turn radius at full steering from the repository's
# Ackermann LUT/model. Positive steering turns toward -Y (right); negative
# steering turns toward +Y (left).
TURN_RADIUS_MM = 109.0
STEP_MM = 5.0
MIN_CONTROL_SEGMENT_STEPS = 4  # 20 mm before changing gear or steering


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    heading_deg: float


def transform_polygon(pose: Pose, local):
    angle = math.radians(pose.heading_deg)
    c = math.cos(angle)
    s = math.sin(angle)
    return tuple((pose.x + c * x - s * y, pose.y + s * x + c * y)
                 for x, y in local)


def local_rotated_rectangle(cx, cy, half_length, half_width, angle_deg):
    angle = math.radians(angle_deg)
    c = math.cos(angle)
    s = math.sin(angle)
    corners = ((half_length, half_width), (half_length, -half_width),
               (-half_length, -half_width), (-half_length, half_width))
    return tuple((cx + c * x - s * y, cy + s * x + c * y)
                 for x, y in corners)


def front_wheel_angles(steering):
    if steering < 0:
        return -37.545, -62.455
    if steering > 0:
        return 62.455, 37.545
    return 0.0, 0.0


def robot_polygons(pose: Pose, steering: int, margin: float = 0.0):
    # The photographs show a roughly 100 mm-wide chassis between the wheels,
    # plus narrow centreline protrusions that establish the 125/40 mm length.
    # Wheels are modeled separately so empty bounding-box corners remain free.
    chassis = ((110.0 + margin, 50.0 + margin),
               (110.0 + margin, -50.0 - margin),
               (-35.0 - margin, -50.0 - margin),
               (-35.0 - margin, 50.0 + margin))
    centre_strip = ((ROBOT_FRONT_MM + margin, 25.0 + margin),
                    (ROBOT_FRONT_MM + margin, -25.0 - margin),
                    (-ROBOT_REAR_MM - margin, -25.0 - margin),
                    (-ROBOT_REAR_MM - margin, 25.0 + margin))
    half_wheel_length = WHEEL_DIAMETER_MM * 0.5 + margin
    half_wheel_width = WHEEL_WIDTH_MM * 0.5 + margin
    left_angle, right_angle = front_wheel_angles(steering)
    local = [
        chassis,
        centre_strip,
        local_rotated_rectangle(0.0, TRACK_HALF_WIDTH_MM,
                                half_wheel_length, half_wheel_width, 0.0),
        local_rotated_rectangle(0.0, -TRACK_HALF_WIDTH_MM,
                                half_wheel_length, half_wheel_width, 0.0),
        local_rotated_rectangle(PHYSICAL_WHEELBASE_MM, TRACK_HALF_WIDTH_MM,
                                half_wheel_length, half_wheel_width, left_angle),
        local_rotated_rectangle(PHYSICAL_WHEELBASE_MM, -TRACK_HALF_WIDTH_MM,
                                half_wheel_length, half_wheel_width, right_angle),
    ]
    return tuple(transform_polygon(pose, poly) for poly in local)


def robot_points(pose: Pose, steering: int):
    return tuple(point for poly in robot_polygons(pose, steering) for point in poly)


def projection(poly, axis):
    values = [x * axis[0] + y * axis[1] for x, y in poly]
    return min(values), max(values)


def polygons_intersect(a, b):
    for poly in (a, b):
        for i in range(len(poly)):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % len(poly)]
            axis = (-(y2 - y1), x2 - x1)
            a_min, a_max = projection(a, axis)
            b_min, b_max = projection(b, axis)
            if a_max < b_min or b_max < a_min:
                return False
    return True


def rectangle(x_min, x_max, y_min, y_max):
    return ((x_min, y_min), (x_max, y_min),
            (x_max, y_max), (x_min, y_max))


def collision(pose: Pose, steering: int, gap_mm: float = MIN_GAP_MM):
    # Keep the robot outline physical and put margins on the relevant obstacle
    # dimensions. This allows a smaller margin at the measured open end of a
    # parking piece without reducing clearance to its broad faces.
    bodies = robot_polygons(pose, steering, 0.0)
    if min(y for body in bodies for _, y in body) <= WALL_SAFETY_MARGIN_MM:
        return True
    rear_marker = rectangle(
        -MARKER_THICKNESS_MM - MARKER_FACE_SAFETY_MM,
        MARKER_FACE_SAFETY_MM,
        0.0,
        PARKING_DEPTH_MM + MARKER_END_SAFETY_MM)
    front_marker = rectangle(
        gap_mm - MARKER_FACE_SAFETY_MM,
        gap_mm + MARKER_THICKNESS_MM + MARKER_FACE_SAFETY_MM,
        0.0,
        PARKING_DEPTH_MM + MARKER_END_SAFETY_MM)
    return any(polygons_intersect(body, rear_marker) or
               polygons_intersect(body, front_marker) for body in bodies)


def advance(pose: Pose, direction: int, steering: int, distance_mm=STEP_MM):
    signed_distance = direction * distance_mm
    heading = math.radians(pose.heading_deg)
    if steering == 0:
        return Pose(pose.x + signed_distance * math.cos(heading),
                    pose.y + signed_distance * math.sin(heading),
                    pose.heading_deg)

    # Firmware sign: negative is left (+Y), positive is right (-Y).
    curvature = -math.copysign(1.0 / TURN_RADIUS_MM, steering)
    delta = signed_distance * curvature
    radius_signed = 1.0 / curvature
    x = pose.x + radius_signed * (math.sin(heading + delta) - math.sin(heading))
    y = pose.y - radius_signed * (math.cos(heading + delta) - math.cos(heading))
    return Pose(x, y, math.degrees(heading + delta))


def swept_valid(start: Pose, end: Pose, direction: int, steering: int):
    pose = start
    for _ in range(5):
        pose = advance(pose, direction, steering, STEP_MM / 5.0)
        if collision(pose, steering):
            return False
    return (abs(pose.x - end.x) < 0.1 and abs(pose.y - end.y) < 0.1 and
            abs(pose.heading_deg - end.heading_deg) < 0.1)


def quantize(pose: Pose, direction: int, steering: int, run_steps: int):
    return (round(pose.x / 5.0), round(pose.y / 5.0),
            round(pose.heading_deg / 3.0), direction, steering,
            min(run_steps, MIN_CONTROL_SEGMENT_STEPS))


def goal(pose: Pose, steering: int):
    return (min(y for _, y in robot_points(pose, steering)) >=
            PARKING_DEPTH_MM + MARKER_END_SAFETY_MM and
            abs(pose.heading_deg) <= 6.0 and steering == 0)


def heuristic(pose: Pose, steering: int):
    min_y = min(y for _, y in robot_points(pose, steering))
    lateral = max(0.0, PARKING_DEPTH_MM + SAFETY_MARGIN_MM - min_y)
    return lateral + abs(pose.heading_deg) * 1.5


def reconstruct(nodes, key):
    steps = []
    while nodes[key][1] is not None:
        pose, parent, direction, steering, _ = nodes[key]
        steps.append((pose, direction, steering))
        key = parent
    steps.reverse()
    segments = []
    for pose, direction, steering in steps:
        if segments and tuple(segments[-1][0:2]) == (direction, steering):
            segments[-1][2] += STEP_MM
            segments[-1][3] = pose
        else:
            segments.append([direction, steering, STEP_MM, pose])
    return segments


def search(rear_clearance_mm: float):
    start = Pose(ROBOT_REAR_MM + rear_clearance_mm,
                 PARKING_DEPTH_MM - 62.5,
                 0.0)
    if collision(start, 0):
        return None, "initial_collision", 0
    start_key = quantize(start, 0, 0, 0)
    nodes = {start_key: (start, None, 0, 0, 0)}
    costs = {start_key: 0.0}
    queue = [(heuristic(start, 0), 0.0, 0, start_key)]
    sequence = 0
    while queue and sequence < 600000:
        _, cost, _, key = heapq.heappop(queue)
        if cost != costs.get(key):
            continue
        pose, _, last_direction, last_steering, run_steps = nodes[key]
        if goal(pose, last_steering):
            return (start, reconstruct(nodes, key), sequence), "goal", sequence
        if not (-120.0 <= pose.x <= 380.0 and 40.0 <= pose.y <= 520.0 and
                -100.0 <= pose.heading_deg <= 100.0):
            continue
        for direction in (1, -1):
            for steering in (-50, 0, 50):
                same_control = (direction == last_direction and
                                steering == last_steering)
                if (last_direction and not same_control and
                        run_steps < MIN_CONTROL_SEGMENT_STEPS):
                    continue
                nxt = advance(pose, direction, steering)
                # Steering moves at the stopped/current pose before motion.
                if collision(pose, steering):
                    continue
                if not swept_valid(pose, nxt, direction, steering):
                    continue
                next_run_steps = run_steps + 1 if same_control else 1
                nxt_key = quantize(nxt, direction, steering, next_run_steps)
                change_cost = 0.0
                if last_direction and direction != last_direction:
                    change_cost += 25.0
                if steering != last_steering:
                    change_cost += 5.0
                new_cost = cost + STEP_MM + change_cost
                if new_cost >= costs.get(nxt_key, float("inf")):
                    continue
                costs[nxt_key] = new_cost
                nodes[nxt_key] = (nxt, key, direction, steering,
                                  next_run_steps)
                sequence += 1
                heapq.heappush(queue, (new_cost + heuristic(nxt, steering), new_cost,
                                       sequence, nxt_key))
    reason = "limit" if sequence >= 600000 else "exhausted"
    return None, reason, sequence


def validate_segments(start, segments):
    scenarios = []
    # Check translations explicitly rather than extending the measured 200 mm
    # parking-piece length by the general safety margin.
    for gap in (MIN_GAP_MM, NOMINAL_GAP_MM + PLACEMENT_ERROR_MM):
        for dx in (-PLACEMENT_ERROR_MM, PLACEMENT_ERROR_MM):
            for dy in (-PLACEMENT_ERROR_MM, PLACEMENT_ERROR_MM):
                for dh in (-HEADING_ERROR_DEG, HEADING_ERROR_DEG):
                    pose = Pose(start.x + dx, start.y + dy,
                                start.heading_deg + dh)
                    valid = not collision(pose, 0, gap)
                    for direction, steering, distance, _ in segments:
                        if collision(pose, steering, gap):
                            valid = False
                            break
                        steps = int(round(distance))
                        for _ in range(steps):
                            pose = advance(pose, direction, steering, 1.0)
                            if collision(pose, steering, gap):
                                valid = False
                                break
                        if not valid:
                            break
                    scenarios.append(valid)
    return sum(scenarios), len(scenarios)


def validate_rear_tof_positioning():
    """Check the supported middle +/-20 mm starts and the complete exit.

    The rear sensor measures longitudinal position, then firmware moves
    straight to 50 mm rear body clearance before running the existing path.
    Include the existing gap, lateral-placement, and +/-1 degree heading cases.
    """
    scenarios = []
    for gap in (MIN_GAP_MM, NOMINAL_GAP_MM + PLACEMENT_ERROR_MM):
        middle_clearance = (gap - ROBOT_FRONT_MM - ROBOT_REAR_MM) * 0.5
        for offset in (-20.0, 0.0, 20.0):
            initial_clearance = middle_clearance + offset
            for y_error in (-PLACEMENT_ERROR_MM, PLACEMENT_ERROR_MM):
                for heading_error in (-HEADING_ERROR_DEG, HEADING_ERROR_DEG):
                    pose = Pose(
                        ROBOT_REAR_MM + initial_clearance,
                        PARKING_DEPTH_MM - 62.5 + y_error,
                        heading_error)
                    valid = not collision(pose, 0, gap)
                    correction = 50.0 - initial_clearance
                    direction = 1 if correction >= 0.0 else -1
                    for _ in range(int(math.ceil(abs(correction)))):
                        step = min(1.0, abs(correction))
                        pose = advance(pose, direction, 0, step)
                        correction -= direction * step
                        if collision(pose, 0, gap):
                            valid = False
                            break
                    if valid:
                        for segment_direction, steering, distance in SELECTED_CONTROLS:
                            for _ in range(int(round(distance))):
                                pose = advance(
                                    pose, segment_direction, steering, 1.0)
                                if collision(pose, steering, gap):
                                    valid = False
                                    break
                            if not valid:
                                break
                    scenarios.append(valid)
    return sum(scenarios), len(scenarios)


SELECTED_CONTROLS = (
    (-1, +50, 20.0),
    (+1, -50, 25.0),
    (-1, +50, 20.0),
    (+1, -50, 75.0),
    # Model distance to return from about 73.6 degrees to parallel. Firmware
    # terminates this final arc from gyro alignment within bounded distance.
    (+1, +50, 140.0),
)

# Test-only straight reverse after the five exit segments. Firmware may stop
# sooner when two wall frames confirm that the side-ToF cone has crossed the
# opposite magenta-piece edge.
SELECTED_WITH_REVERSE_LOCALIZATION = SELECTED_CONTROLS + (
    (-1, 0, 60.0),
)

# Direction-specific test-only discovery envelopes. The firmware stops the
# first straight segment when the corrected field x reaches 60 mm (CCW) or
# 520 mm (CW); these nominal controls reproduce those endpoints from the
# selected prototype start. The simulation's +y always points away from the
# outer wall, so its CW local frame is reflected; the physical firmware mirrors
# the steering even though the local swept-envelope control retains +50.
CCW_ENTRY_DISCOVERY_CONTROLS = SELECTED_CONTROLS + (
    (-1, 0, 400.0),
    (-1, +50, 40.0),
)
CW_ENTRY_DISCOVERY_CONTROLS = SELECTED_CONTROLS + (
    (-1, 0, 260.0),
    (-1, +50, 40.0),
)


def build_segments(start, controls=SELECTED_CONTROLS):
    pose = start
    segments = []
    for direction, steering, distance in controls:
        steps = max(1, int(round(distance)))
        step_mm = distance / steps
        for _ in range(steps):
            pose = advance(pose, direction, steering, step_mm)
        segments.append([direction, steering, distance, pose])
    return segments


def sampled_path(start, controls=SELECTED_CONTROLS):
    pose = start
    points = [pose]
    endpoints = []
    for direction, steering, distance in controls:
        steps = max(1, int(math.ceil(distance)))
        step_mm = distance / steps
        for _ in range(steps):
            pose = advance(pose, direction, steering, step_mm)
            points.append(pose)
        endpoints.append(pose)
    return points, endpoints


def svg_polygon(points, tx, ty, **attrs):
    attributes = " ".join(f'{key}="{value}"' for key, value in attrs.items())
    coords = " ".join(f"{tx(x):.1f},{ty(y):.1f}" for x, y in points)
    return f'<polygon points="{coords}" {attributes}/>'


def write_svg(start, output_path):
    path, endpoints = sampled_path(start, SELECTED_WITH_REVERSE_LOCALIZATION)
    x_min, x_max = -35.0, 370.0
    y_min, y_max = -10.0, 370.0
    width, height = 850.0, 800.0
    pad = 45.0
    scale = min((width - 2 * pad) / (x_max - x_min),
                (height - 2 * pad) / (y_max - y_min))
    tx = lambda x: pad + (x - x_min) * scale
    ty = lambda y: height - pad - (y - y_min) * scale
    out = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 850 800" '
        'role="img" aria-labelledby="title desc">',
        '<title id="title">Parking exit and reverse localization path</title>',
        '<desc id="desc">Top-down geometry of the two magenta parking pieces, '
        'outer wall, rear-axle path, and robot footprint at the start, after '
        'segment four, parallel outside the parking lot, and after the bounded '
        'reverse localization movement.</desc>',
        '<rect width="850" height="800" fill="#ffffff"/>',
        f'<line x1="{tx(x_min):.1f}" y1="{ty(0):.1f}" '
        f'x2="{tx(x_max):.1f}" y2="{ty(0):.1f}" stroke="#111827" '
        'stroke-width="8"/>',
        f'<line x1="{tx(0):.1f}" y1="{ty(PARKING_DEPTH_MM):.1f}" '
        f'x2="{tx(NOMINAL_GAP_MM):.1f}" y2="{ty(PARKING_DEPTH_MM):.1f}" '
        'stroke="#64748b" stroke-width="2" stroke-dasharray="8 6"/>',
        svg_polygon(rectangle(-MARKER_THICKNESS_MM, 0.0, 0.0,
                              PARKING_DEPTH_MM), tx, ty,
                    fill="#ec4899", stroke="#9d174d", **{"stroke-width": "2"}),
        svg_polygon(rectangle(NOMINAL_GAP_MM,
                              NOMINAL_GAP_MM + MARKER_THICKNESS_MM,
                              0.0, PARKING_DEPTH_MM), tx, ty,
                    fill="#ec4899", stroke="#9d174d", **{"stroke-width": "2"}),
        '<text x="55" y="770" font-family="sans-serif" font-size="15" '
        'fill="#111827">Outer black wall</text>',
        f'<text x="{tx(105):.1f}" y="{ty(PARKING_DEPTH_MM) - 10:.1f}" '
        'font-family="sans-serif" font-size="14" fill="#475569">200 mm '
        'parking opening</text>',
    ]
    path_points = " ".join(f"{tx(p.x):.1f},{ty(p.y):.1f}" for p in path)
    out.append(f'<polyline points="{path_points}" fill="none" stroke="#2563eb" '
               'stroke-width="4"/>')

    shown = (("Start", start, 0),
             ("After segment 4", endpoints[3], -50),
             ("Parallel outside", endpoints[4], +50),
             ("Reverse localization limit", endpoints[5], 0))
    for label, pose, steering in shown:
        for poly in robot_polygons(pose, steering, 0.0):
            out.append(svg_polygon(poly, tx, ty, fill="#22c55e",
                                   stroke="#166534", opacity="0.20",
                                   **{"stroke-width": "1.5"}))
        out.append(f'<circle cx="{tx(pose.x):.1f}" cy="{ty(pose.y):.1f}" '
                   'r="5" fill="#166534"/>')
        out.append(f'<text x="{tx(pose.x) + 8:.1f}" y="{ty(pose.y) - 8:.1f}" '
                   'font-family="sans-serif" font-size="14" fill="#111827">'
                   f'{label}</text>')

    for index, pose in enumerate(endpoints, 1):
        out.append(f'<circle cx="{tx(pose.x):.1f}" cy="{ty(pose.y):.1f}" '
                   'r="8" fill="#ffffff" stroke="#2563eb" stroke-width="3"/>')
        out.append(f'<text x="{tx(pose.x):.1f}" y="{ty(pose.y) + 5:.1f}" '
                   'text-anchor="middle" font-family="sans-serif" font-size="13" '
                   f'font-weight="bold" fill="#1e3a8a">{index}</text>')

    out.extend([
        '<line x1="565" y1="42" x2="610" y2="42" stroke="#2563eb" '
        'stroke-width="4"/><text x="620" y="47" font-family="sans-serif" '
        'font-size="14" fill="#111827">Rear-axle path</text>',
        '<rect x="565" y="60" width="45" height="18" fill="#22c55e" '
        'fill-opacity="0.20" stroke="#166534"/><text x="620" y="74" '
        'font-family="sans-serif" font-size="14" fill="#111827">Robot footprint</text>',
        '<rect x="565" y="90" width="45" height="18" fill="#ec4899" '
        'stroke="#9d174d"/><text x="620" y="104" font-family="sans-serif" '
        'font-size="14" fill="#111827">Parking pieces</text>',
        '</svg>'
    ])
    Path(output_path).write_text("\n".join(out), encoding="utf-8")


def report_selected():
    start = Pose(ROBOT_REAR_MM + 50.0, PARKING_DEPTH_MM - 62.5, 0.0)
    segments = build_segments(start)
    print("selected five-segment candidate")
    print(f"  start rear_axle=({start.x:.1f},{start.y:.1f})")
    for direction, steering, distance, pose in segments:
        verb = "F" if direction > 0 else "R"
        print(f"  {verb} steer={steering:+d} distance={distance:.0f} -> "
              f"({pose.x:.1f},{pose.y:.1f},{pose.heading_deg:.1f}deg)")
    passed, total = validate_segments(start, segments)
    print(f"  tolerance_scenarios={passed}/{total}")
    localized_segments = build_segments(
        start, SELECTED_WITH_REVERSE_LOCALIZATION)
    localized_pose = localized_segments[-1][3]
    reverse_passed, reverse_total = validate_segments(
        start, localized_segments)
    print("  reverse localization limit -> "
          f"({localized_pose.x:.1f},{localized_pose.y:.1f},"
          f"{localized_pose.heading_deg:.1f}deg)")
    print("  exit_plus_reverse_tolerance_scenarios="
          f"{reverse_passed}/{reverse_total}")
    ccw_segments = build_segments(start, CCW_ENTRY_DISCOVERY_CONTROLS)
    ccw_passed, ccw_total = validate_segments(start, ccw_segments)
    cw_segments = build_segments(start, CW_ENTRY_DISCOVERY_CONTROLS)
    cw_passed, cw_total = validate_segments(start, cw_segments)
    print(f"  ccw_entry_discovery_tolerance_scenarios="
          f"{ccw_passed}/{ccw_total}")
    print(f"  cw_entry_discovery_tolerance_scenarios="
          f"{cw_passed}/{cw_total}")
    rear_passed, rear_total = validate_rear_tof_positioning()
    print(f"  rear_tof_middle_plus_minus_20_exit_scenarios="
          f"{rear_passed}/{rear_total}")
    output = Path(__file__).with_name("parking_exit_path.svg")
    write_svg(start, output)
    print(f"  diagram={output}")
    return (passed == total and reverse_passed == reverse_total and
            ccw_passed == ccw_total and cw_passed == cw_total and
            rear_passed == rear_total)


def search_alternatives():
    for rear_clearance in range(5, 76, 5):
        result, reason, expanded = search(float(rear_clearance))
        if result is None:
            print(f"rear_clearance={rear_clearance}: no path "
                  f"reason={reason} generated={expanded}")
            continue
        start, segments, expanded = result
        print(f"rear_clearance={rear_clearance}: path expanded={expanded}")
        print(f"  start rear_axle=({start.x:.1f},{start.y:.1f})")
        for direction, steering, distance, pose in segments:
            verb = "F" if direction > 0 else "R"
            print(f"  {verb} steer={steering:+d} distance={distance:.0f} -> "
                  f"({pose.x:.1f},{pose.y:.1f},{pose.heading_deg:.1f}deg)")
        passed, total = validate_segments(start, segments)
        print(f"  tolerance_scenarios={passed}/{total}")
        if passed == total:
            break


def main():
    report_selected()


if __name__ == "__main__":
    main()
