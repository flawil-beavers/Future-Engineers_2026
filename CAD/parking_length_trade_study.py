"""Compare permanent length extensions against the checked parking-exit path.

This is a sensitivity study, not a path optimizer. It deliberately reuses the
footprint, collision checks, motion model, tolerance cases, five exit controls,
and bounded reverse-localization movement from parking_exit_swept_search.py.
Any result below 16/16 means the existing path is invalid for that geometry.
Even 16/16 would still require a newly contained final-parking target and
physical validation.
"""

from __future__ import annotations

import parking_exit_swept_search as model


VARIANTS = (
    ("current", 125.0, 40.0),
    ("rear +15", 125.0, 55.0),
    ("rear +35", 125.0, 75.0),
    ("rear +55", 125.0, 95.0),
    ("front +15", 140.0, 40.0),
    ("front +35", 160.0, 40.0),
)


def validate_variant(name: str, front_mm: float, rear_mm: float):
    length_mm = front_mm + rear_mm
    gap_mm = 1.5 * length_mm

    model.ROBOT_FRONT_MM = front_mm
    model.ROBOT_REAR_MM = rear_mm
    model.NOMINAL_GAP_MM = gap_mm
    model.MIN_GAP_MM = gap_mm - model.PLACEMENT_ERROR_MM

    start = model.Pose(
        rear_mm + 50.0,
        model.PARKING_DEPTH_MM - 62.5,
        0.0,
    )
    segments = model.build_segments(
        start,
        model.SELECTED_WITH_REVERSE_LOCALIZATION,
    )
    passed, total = model.validate_segments(start, segments)
    return {
        "name": name,
        "front_mm": front_mm,
        "rear_mm": rear_mm,
        "length_mm": length_mm,
        "gap_mm": gap_mm,
        "slack_mm": 0.5 * length_mm,
        "centred_clearance_mm": 0.25 * length_mm,
        "passed": passed,
        "total": total,
    }


def main():
    print(
        "variant,front_mm,rear_mm,length_mm,gap_mm,total_slack_mm,"
        "centred_end_clearance_mm,existing_path_cases"
    )
    for variant in VARIANTS:
        result = validate_variant(*variant)
        print(
            f"{result['name']},{result['front_mm']:.1f},"
            f"{result['rear_mm']:.1f},{result['length_mm']:.1f},"
            f"{result['gap_mm']:.1f},{result['slack_mm']:.1f},"
            f"{result['centred_clearance_mm']:.2f},"
            f"{result['passed']}/{result['total']}"
        )


if __name__ == "__main__":
    main()
