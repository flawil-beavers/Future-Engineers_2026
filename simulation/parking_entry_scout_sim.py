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
    return (
        orientation(ax, ay, bx, by, cx, cy)
        * orientation(ax, ay, bx, by, dx, dy)
        <= 0.0
        and orientation(cx, cy, dx, dy, ax, ay)
        * orientation(cx, cy, dx, dy, bx, by)
        <= 0.0
    )


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
    x: float, y: float, heading_deg: float, turn: str, travel_mm: float
) -> tuple[float, float, float]:
    route_turn_sign = -1.0 if turn == "CW" else 1.0
    curvature = -route_turn_sign / RADIUS_MM
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


if __name__ == "__main__":
    main()
