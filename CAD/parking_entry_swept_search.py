"""Search and validate a dedicated final-parking trajectory.

The collision model and measured 165 mm robot geometry come from
``parking_exit_swept_search.py``.  The search starts at the centred,
fully-contained scoring pose and drives out of the bay.  Reversing the order
and drive direction of that result gives the firmware entry controls.
"""

from __future__ import annotations

import heapq
import math
import sys

sys.dont_write_bytecode = True

import parking_exit_swept_search as model


TARGET = model.Pose(81.25, 100.0, 0.0)
GOAL_OPEN_MARGIN_MM = 10.0
MAX_NODES = 1_200_000
SEARCH_MARKER_FACE_SAFETY_MM = 12.0
SEARCH_WALL_SAFETY_MARGIN_MM = 10.0


def footprint_min_y(pose, steering):
    return min(y for _, y in model.robot_points(pose, steering))


def is_capture_pose(pose, steering, run_steps):
    return (footprint_min_y(pose, steering) >=
            model.PARKING_DEPTH_MM + GOAL_OPEN_MARGIN_MM and
            abs(pose.heading_deg) <= 1.0 and steering == 0 and
            run_steps >= model.MIN_CONTROL_SEGMENT_STEPS)


def heuristic(pose, steering):
    lateral = max(0.0, model.PARKING_DEPTH_MM + GOAL_OPEN_MARGIN_MM -
                  footprint_min_y(pose, steering))
    return lateral + abs(pose.heading_deg) * 1.5


def search():
    if model.collision(TARGET, 0):
        return None, "target_collision", 0
    start_key = model.quantize(TARGET, 0, 0, 0)
    nodes = {start_key: (TARGET, None, 0, 0, 0)}
    costs = {start_key: 0.0}
    queue = [(heuristic(TARGET, 0), 0.0, 0, start_key)]
    sequence = 0

    while queue and sequence < MAX_NODES:
        _, cost, _, key = heapq.heappop(queue)
        if cost != costs.get(key):
            continue
        pose, _, last_direction, last_steering, run_steps = nodes[key]
        if is_capture_pose(pose, last_steering, run_steps):
            return (TARGET, model.reconstruct(nodes, key), sequence), "goal", sequence
        if not (-120.0 <= pose.x <= 380.0 and
                30.0 <= pose.y <= 520.0 and
                -120.0 <= pose.heading_deg <= 120.0):
            continue

        for direction in (1, -1):
            for steering in (-50, 0, 50):
                same_control = (direction == last_direction and
                                steering == last_steering)
                if (last_direction and not same_control and
                        run_steps < model.MIN_CONTROL_SEGMENT_STEPS):
                    continue
                nxt = model.advance(pose, direction, steering)
                if model.collision(pose, steering):
                    continue
                if not model.swept_valid(pose, nxt, direction, steering):
                    continue
                next_run_steps = run_steps + 1 if same_control else 1
                nxt_key = model.quantize(
                    nxt, direction, steering, next_run_steps)
                change_cost = 0.0
                if last_direction and direction != last_direction:
                    change_cost += 25.0
                if steering != last_steering:
                    change_cost += 5.0
                new_cost = cost + model.STEP_MM + change_cost
                if new_cost >= costs.get(nxt_key, float("inf")):
                    continue
                costs[nxt_key] = new_cost
                nodes[nxt_key] = (
                    nxt, key, direction, steering, next_run_steps)
                sequence += 1
                heapq.heappush(
                    queue,
                    (new_cost + heuristic(nxt, steering),
                     new_cost, sequence, nxt_key))

    return None, "limit" if sequence >= MAX_NODES else "exhausted", sequence


def reverse_controls(exit_segments):
    return tuple(
        (-direction, steering, distance)
        for direction, steering, distance, _ in reversed(exit_segments))


def drive_controls(start, controls, gap_mm):
    pose = start
    if model.collision(pose, 0, gap_mm):
        return pose, False
    for direction, steering, distance in controls:
        if model.collision(pose, steering, gap_mm):
            return pose, False
        steps = max(1, int(math.ceil(distance)))
        step_mm = distance / steps
        for _ in range(steps):
            pose = model.advance(pose, direction, steering, step_mm)
            if model.collision(pose, steering, gap_mm):
                return pose, False
    return pose, True


def contained(pose, gap_mm):
    points = model.robot_points(pose, 0)
    return (min(x for x, _ in points) > 0.0 and
            max(x for x, _ in points) < gap_mm and
            min(y for _, y in points) > 0.0 and
            max(y for _, y in points) < model.PARKING_DEPTH_MM)


def validate_entry(capture, controls):
    passed = 0
    total = 0
    worst = None
    for gap in (model.MIN_GAP_MM,
                model.NOMINAL_GAP_MM + model.PLACEMENT_ERROR_MM):
        for dx in (-model.PLACEMENT_ERROR_MM, model.PLACEMENT_ERROR_MM):
            for dy in (-model.PLACEMENT_ERROR_MM, model.PLACEMENT_ERROR_MM):
                for dh in (-model.HEADING_ERROR_DEG, model.HEADING_ERROR_DEG):
                    total += 1
                    initial = model.Pose(
                        capture.x + dx, capture.y + dy,
                        capture.heading_deg + dh)
                    final, valid = drive_controls(initial, controls, gap)
                    valid = valid and contained(final, gap)
                    if valid:
                        passed += 1
                    elif worst is None:
                        worst = (gap, dx, dy, dh, final)
    return passed, total, worst


def main():
    normal_marker_margin = model.MARKER_FACE_SAFETY_MM
    normal_wall_margin = model.WALL_SAFETY_MARGIN_MM
    model.MARKER_FACE_SAFETY_MM = SEARCH_MARKER_FACE_SAFETY_MM
    model.WALL_SAFETY_MARGIN_MM = SEARCH_WALL_SAFETY_MARGIN_MM
    result, reason, visited = search()
    print(f"search={reason} visited={visited}")
    if result is None:
        raise SystemExit(1)
    target, exit_segments, _ = result
    capture = exit_segments[-1][3]
    entry = reverse_controls(exit_segments)
    model.MARKER_FACE_SAFETY_MM = normal_marker_margin
    model.WALL_SAFETY_MARGIN_MM = normal_wall_margin
    print(f"target=({target.x:.2f},{target.y:.2f},{target.heading_deg:.2f})")
    print(f"capture=({capture.x:.2f},{capture.y:.2f},{capture.heading_deg:.2f})")
    print("exit controls:")
    for direction, steering, distance, endpoint in exit_segments:
        print(f"  ({direction:+d}, {steering:+d}, {distance:.1f}) -> "
              f"({endpoint.x:.1f},{endpoint.y:.1f},{endpoint.heading_deg:.1f})")
    print("entry controls:")
    for direction, steering, distance in entry:
        print(f"  ({direction:+d}, {steering:+d}, {distance:.1f})")
    passed, total, worst = validate_entry(capture, entry)
    print(f"entry tolerance={passed}/{total}")
    if worst:
        print(f"first failure={worst}")
    if passed != total:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
