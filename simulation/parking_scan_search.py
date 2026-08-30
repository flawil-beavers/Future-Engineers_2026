"""Historical bounded search for a parking-entry camera scan pose."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import simulation.parking_exit_swept_search as model


start = model.Pose(
    model.ROBOT_REAR_MM + 50.0,
    model.PARKING_DEPTH_MM - 62.5,
    0.0,
)
target_x = 267.5
target_y = 600.0
def camera_geometry(pose):
    heading = math.radians(pose.heading_deg)
    camera_x = pose.x + 125.0 * math.cos(heading)
    camera_y = pose.y + 125.0 * math.sin(heading)
    bearing = (
        math.degrees(math.atan2(target_y - camera_y, target_x - camera_x))
        - pose.heading_deg
        + 180.0
    ) % 360.0 - 180.0
    distance = math.hypot(target_x - camera_x, target_y - camera_y)
    return bearing, distance


candidates = []
control_pairs = ((-1, -50), (-1, +50), (+1, -50), (+1, +50))
for first in control_pairs:
    for second in control_pairs:
        for first_mm in range(20, 201, 20):
            for second_mm in range(20, 201, 20):
                controls = model.SELECTED_WITH_REVERSE_LOCALIZATION + (
                    (first[0], first[1], float(first_mm)),
                    (second[0], second[1], float(second_mm)),
                )
                segments = model.build_segments(start, controls)
                pose = segments[-1][3]
                bearing, distance = camera_geometry(pose)
                if abs(bearing) <= 25.0 and 230.0 <= distance <= 600.0:
                    candidates.append((first_mm + second_mm, abs(bearing),
                                       first, second, first_mm, second_mm,
                                       bearing, distance, controls, pose))

print(f"nominal_candidates={len(candidates)}", flush=True)
passed = []
def quick_validate(segments):
    valid_count = 0
    for gap in (model.MIN_GAP_MM,
                model.NOMINAL_GAP_MM + model.PLACEMENT_ERROR_MM):
        for dx in (-model.PLACEMENT_ERROR_MM, model.PLACEMENT_ERROR_MM):
            for dy in (-model.PLACEMENT_ERROR_MM, model.PLACEMENT_ERROR_MM):
                for dh in (-model.HEADING_ERROR_DEG, model.HEADING_ERROR_DEG):
                    pose = model.Pose(start.x + dx, start.y + dy,
                                      start.heading_deg + dh)
                    valid = not model.collision(pose, 0, gap)
                    for direction, steering, distance, _ in segments:
                        steps = max(1, int(math.ceil(distance / 5.0)))
                        step = distance / steps
                        for _ in range(steps):
                            pose = model.advance(
                                pose, direction, steering, step)
                            if model.collision(pose, steering, gap):
                                valid = False
                                break
                        if not valid:
                            break
                    if valid:
                        valid_count += 1
    return valid_count == 16


for candidate in sorted(candidates):
    segments = model.build_segments(start, candidate[8])
    if not quick_validate(segments):
        continue
    valid, total = model.validate_segments(start, segments)
    if valid == total:
        passed.append(candidate)
        print(
            "PASS "
            f"first={candidate[2]}x{candidate[4]} "
            f"second={candidate[3]}x{candidate[5]} "
            f"bearing={candidate[6]:.1f} range={candidate[7]:.1f} "
            f"pose={candidate[9]}",
            flush=True,
        )
        if len(passed) >= 10:
            break

print(f"passed={len(passed)}", flush=True)
