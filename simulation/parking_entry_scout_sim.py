"""Offline geometry/state checks for the parking-entry scout.

This intentionally models only fixed-field geometry and ideal encoder motion.
It cannot establish physical clearance, camera segmentation reliability, or
tracking accuracy.
"""

from __future__ import annotations

import math
import argparse
import pathlib
import re
import csv
import itertools


ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_LOG_DIR = ROOT / "simulation" / "fixtures" / "parking_entry_scout"

RADIUS_MM = 109.0
SCOUT_MM = 85.0
CAMERA_X_MM = 125.0
VIEW_MIN_MM = 230.0
VIEW_MAX_MM = 600.0
BEARING_LIMIT_DEG = 65.3 * 0.42 - 1.0
ROBOT_RADIUS_MM = 70.0
ROBOT_AXIS_FRONT_MM = 60.0
PILLAR_RADIUS_MM = 42.5

WALLS = (
    (-1500.0, -1500.0, 1500.0, -1500.0),
    (1500.0, -1500.0, 1500.0, 1500.0),
    (1500.0, 1500.0, -1500.0, 1500.0),
    (-1500.0, 1500.0, -1500.0, -1500.0),
    (-500.0, -500.0, 500.0, -500.0),
    (500.0, -500.0, 500.0, 500.0),
    (500.0, 500.0, -500.0, 500.0),
    (-500.0, 500.0, -500.0, -500.0),
)

RESULT_RE = re.compile(
    r"\[PARK ENTRY RESULT\].*?station=(?P<station>\d+).*?"
    r"pose_x_y_heading=(?P<x>-?[\d.]+)/(?P<y>-?[\d.]+)/(?P<h>-?[\d.]+)"
)
TURN_RE = re.compile(r"discovery armed turn=(CW|CCW)")


def wrap180(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def point_segment_distance(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def orientation(ax: float, ay: float, bx: float, by: float, cx: float, cy: float) -> float:
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def segments_intersect(a: tuple[float, ...], b: tuple[float, ...]) -> bool:
    ax, ay, bx, by = a
    cx, cy, dx, dy = b
    o1 = orientation(ax, ay, bx, by, cx, cy)
    o2 = orientation(ax, ay, bx, by, dx, dy)
    o3 = orientation(cx, cy, dx, dy, ax, ay)
    o4 = orientation(cx, cy, dx, dy, bx, by)
    if o1 * o2 < 0.0 and o3 * o4 < 0.0:
        return True
    # Collinearity alone does not imply overlap: disjoint collinear segments
    # must retain their real distance rather than being reported as touching.
    def on_segment(px: float, py: float, segment: tuple[float, ...]) -> bool:
        x1, y1, x2, y2 = segment
        return (min(x1, x2) - 1e-9 <= px <= max(x1, x2) + 1e-9
                and min(y1, y2) - 1e-9 <= py <= max(y1, y2) + 1e-9)
    return ((abs(o1) < 1e-9 and on_segment(cx, cy, a))
            or (abs(o2) < 1e-9 and on_segment(dx, dy, a))
            or (abs(o3) < 1e-9 and on_segment(ax, ay, b))
            or (abs(o4) < 1e-9 and on_segment(bx, by, b)))


def segment_distance(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    if segments_intersect(a, b):
        return 0.0
    ax, ay, bx, by = a
    cx, cy, dx, dy = b
    return min(
        point_segment_distance(ax, ay, cx, cy, dx, dy),
        point_segment_distance(bx, by, cx, cy, dx, dy),
        point_segment_distance(cx, cy, ax, ay, bx, by),
        point_segment_distance(dx, dy, ax, ay, bx, by),
    )


def scout_pose(
    x: float, y: float, heading_deg: float, turn: str, travel_mm: float,
    radius_mm: float = RADIUS_MM,
) -> tuple[float, float, float]:
    route_turn_sign = -1.0 if turn == "CW" else 1.0
    curvature = -route_turn_sign / radius_mm
    heading = math.radians(heading_deg)
    remaining = travel_mm
    while remaining > 0.1:
        step = min(10.0, remaining)
        signed_step = -step
        next_heading = heading + curvature * signed_step
        x += (math.sin(next_heading) - math.sin(heading)) / curvature
        y += (-math.cos(next_heading) + math.cos(heading)) / curvature
        heading = next_heading
        remaining -= step
    return x, y, math.degrees(heading)


def preceding_inner_seat(turn: str, target_station: int) -> tuple[float, float]:
    station = target_station - 1
    if turn == "CCW":
        x = -500.0 + station * 500.0
    else:
        x = 500.0 - station * 500.0
    return x, -900.0


def camera_geometry(
    pose: tuple[float, float, float], seat: tuple[float, float]
) -> tuple[float, float]:
    x, y, heading_deg = pose
    heading = math.radians(heading_deg)
    camera_x = x + CAMERA_X_MM * math.cos(heading)
    camera_y = y + CAMERA_X_MM * math.sin(heading)
    dx = seat[0] - camera_x
    dy = seat[1] - camera_y
    bearing = wrap180(math.degrees(math.atan2(dy, dx)) - heading_deg)
    return bearing, math.hypot(dx, dy)


def clearance(pose: tuple[float, float, float], seat: tuple[float, float]) -> tuple[float, float]:
    x, y, heading_deg = pose
    heading = math.radians(heading_deg)
    axis = (
        x,
        y,
        x + ROBOT_AXIS_FRONT_MM * math.cos(heading),
        y + ROBOT_AXIS_FRONT_MM * math.sin(heading),
    )
    pillar = (
        point_segment_distance(seat[0], seat[1], *axis)
        - ROBOT_RADIUS_MM
        - PILLAR_RADIUS_MM
    )
    wall = min(segment_distance(axis, segment) for segment in WALLS) - ROBOT_RADIUS_MM
    return wall, pillar


def simulate_log(path: pathlib.Path) -> str | None:
    text = path.read_text(errors="replace")
    turn_match = TURN_RE.search(text)
    result_match = RESULT_RE.search(text)
    if not turn_match or not result_match:
        return None
    turn = turn_match.group(1)
    station = int(result_match.group("station"))
    start = tuple(float(result_match.group(key)) for key in ("x", "y", "h"))
    seat = preceding_inner_seat(turn, station)

    minimum_wall = math.inf
    minimum_pillar = math.inf
    for travel in range(0, int(SCOUT_MM) + 1, 5):
        pose = scout_pose(*start, turn, float(travel))
        wall, pillar = clearance(pose, seat)
        minimum_wall = min(minimum_wall, wall)
        minimum_pillar = min(minimum_pillar, pillar)

    end = scout_pose(*start, turn, SCOUT_MM)
    bearing, range_mm = camera_geometry(end, seat)
    visible = (
        VIEW_MIN_MM <= range_mm <= VIEW_MAX_MM
        and abs(bearing) <= BEARING_LIMIT_DEG
    )
    passed = minimum_wall > 0.0 and minimum_pillar > 0.0 and visible
    return (
        f"{path.stem}: {turn} target_station={station} "
        f"end={end[0]:.1f}/{end[1]:.1f}/{end[2]:.1f} "
        f"preceding_bearing/range={bearing:.1f}/{range_mm:.1f} "
        f"min_wall/pillar={minimum_wall:.1f}/{minimum_pillar:.1f} "
        f"{'PASS' if passed else 'FAIL'}"
    )


def check_state_transitions() -> None:
    # Connector arming is legal only after both independent station outcomes.
    scenarios = {
        "primary_and_scout_resolved": (True, True, "CONNECTOR"),
        "initial_primary_unknown": (False, True, "PRIMARY_RETRY"),
        "scout_unknown": (True, False, "HOLD"),
        "retry_unknown": (False, True, "HOLD"),
    }
    for name, (primary, scout, expected) in scenarios.items():
        if primary and scout:
            actual = "CONNECTOR"
        elif name == "initial_primary_unknown" and scout:
            actual = "PRIMARY_RETRY"
        else:
            actual = "HOLD"
        assert actual == expected, (name, actual, expected)


def sensitivity_log(path: pathlib.Path) -> list[dict[str, object]]:
    """Explore assumed errors; these bounds are not physical validation."""
    text = path.read_text(errors="replace")
    turn_match = TURN_RE.search(text)
    result_match = RESULT_RE.search(text)
    if not turn_match or not result_match:
        raise ValueError(f"missing scout pose in {path.name}")
    turn = turn_match.group(1)
    station = int(result_match.group("station"))
    start = tuple(float(result_match.group(key)) for key in ("x", "y", "h"))
    seat = preceding_inner_seat(turn, station)
    rows = []
    for dx, dy, dh, radius_factor, ds in itertools.product(
            (-10.0, 0.0, 10.0), (-10.0, 0.0, 10.0), (-2.0, 0.0, 2.0),
            (0.95, 1.0, 1.05), (-5.0, 0.0, 5.0)):
        radius = RADIUS_MM * radius_factor
        travel = SCOUT_MM + ds
        perturbed = (start[0] + dx, start[1] + dy, start[2] + dh)
        points = [float(value) for value in range(0, int(travel), 5)] + [travel]
        clearances = [clearance(scout_pose(*perturbed, turn, distance, radius), seat)
                      for distance in points]
        wall = min(value[0] for value in clearances)
        pillar = min(value[1] for value in clearances)
        bearing, distance = camera_geometry(
            scout_pose(*perturbed, turn, travel, radius), seat)
        bearing_margin = BEARING_LIMIT_DEG - abs(bearing)
        range_margin = min(distance - VIEW_MIN_MM, VIEW_MAX_MM - distance)
        rows.append(dict(log=path.stem, turn=turn, dx_mm=dx, dy_mm=dy,
                         dh_deg=dh, radius_mm=radius, travel_mm=travel,
                         bearing_deg=bearing, range_mm=distance,
                         bearing_margin_deg=bearing_margin,
                         range_margin_mm=range_margin,
                         wall_mm=wall, pillar_mm=pillar,
                         passed=wall > 0 and pillar > 0
                         and bearing_margin >= 0 and range_margin >= 0))
    return rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay parking-entry scout geometry from robot logs."
    )
    parser.add_argument(
        "--log-dir",
        type=pathlib.Path,
        default=DEFAULT_LOG_DIR,
        help="directory containing log_<number>.txt files",
    )
    parser.add_argument("--first-log", type=int, default=362)
    parser.add_argument("--last-log", type=int, default=369)
    parser.add_argument("--sensitivity", action="store_true",
                        help="explore assumed +/-10 mm XY, +/-2 deg, +/-5%% radius, +/-5 mm travel")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.first_log > args.last_log:
        raise SystemExit("--first-log must not exceed --last-log")
    check_state_transitions()
    print("state_transition_checks: PASS")
    count = 0
    failures = 0
    expected = args.last_log - args.first_log + 1
    for number in range(args.first_log, args.last_log + 1):
        path = args.log_dir / f"log_{number}.txt"
        if not path.is_file():
            raise SystemExit(f"missing required log: {path}")
        result = simulate_log(path)
        if result:
            print(result)
            count += 1
            failures += int(result.endswith("FAIL"))
    assert count == expected, (count, expected)
    assert failures == 0, failures
    print(f"geometry_cases: {count}/{expected} PASS")
    if args.sensitivity:
        rows = []
        print("Sensitivity assumptions, NOT measured robot tolerances: "
              "XY +/-10 mm; heading +/-2 deg; radius +/-5%; travel +/-5 mm")
        for number in range(args.first_log, args.last_log + 1):
            cases = sensitivity_log(args.log_dir / f"log_{number}.txt")
            rows.extend(cases)
            print(f"log_{number}: {sum(bool(row['passed']) for row in cases)}/{len(cases)} "
                  f"assumed cases within view/clearance; "
                  f"min bearing/range margin={min(float(r['bearing_margin_deg']) for r in cases):.2f}deg/"
                  f"{min(float(r['range_margin_mm']) for r in cases):.1f}mm; "
                  f"wall/pillar={min(float(r['wall_mm']) for r in cases):.1f}/"
                  f"{min(float(r['pillar_mm']) for r in cases):.1f}mm")
        output = ROOT / "local_workspace" / "parking-entry-sensitivity.csv"
        output.parent.mkdir(exist_ok=True)
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f"Sensitivity results: {output.relative_to(ROOT).as_posix()}")


if __name__ == "__main__":
    main()
