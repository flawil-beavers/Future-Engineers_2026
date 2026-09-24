#!/usr/bin/env python3
"""Analyze versioned parking-exit diagnostic records using only stdlib."""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
import statistics
from dataclasses import dataclass
from pathlib import Path

PREFIX = "[PARK_DIAG] "
CONFIG_PREFIX = "[PARK_DIAG_CONFIG] "
SUPPORTED_SCHEMA = 2


def fields(text: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for token in text.strip().split():
        if "=" in token:
            key, value = token.split("=", 1)
            result[key] = value
    return result


def triple(value: str) -> tuple[float, float, float]:
    parts = value.split(",")
    if len(parts) != 3:
        raise ValueError(f"expected three comma-separated values: {value}")
    return tuple(float(item) for item in parts)  # type: ignore[return-value]


def wrap180(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


@dataclass
class Sample:
    time_ms: int
    state: str
    segment: int
    turn: int
    encoder_mm: float
    command: int
    speed: float
    steering: float
    dc_state: int
    gyro_deg: float
    pose: tuple[float, float, float]
    nominal: tuple[float, float, float]
    reference: str
    raw: dict[str, str]

    @property
    def position_error_mm(self) -> float:
        return math.hypot(self.pose[0] - self.nominal[0],
                          self.pose[1] - self.nominal[1])

    @property
    def heading_error_deg(self) -> float:
        return wrap180(self.pose[2] - self.nominal[2])


@dataclass
class ParsedLog:
    path: Path
    config: dict[str, str]
    samples: list[Sample]
    events: list[dict[str, str]]
    corrections: list[dict[str, str]]
    overflow: bool
    truncated: bool
    duplicate_tof_sequences: int
    ordering_errors: int


def tof_value(sample: Sample, sensor: str) -> tuple[float, float, float] | None:
    """Return filtered range, sigma and age for a valid, accepted observation."""
    value = sample.raw.get(sensor, "none")
    if value in {"none", "same"}:
        return None
    parts = value.split(",")
    if len(parts) != 8 or parts[6:] != ["1", "1"]:
        return None
    return float(parts[3]), float(parts[5]), float(parts[1])


def resolved_tof(samples: list[Sample], index: int, sensor: str,
                 max_age_ms: float = 300.0) -> tuple[float, float, float] | None:
    """Resolve a fresh value, including a prior value represented by `same`."""
    for prior_index in range(index, -1, -1):
        value = tof_value(samples[prior_index], sensor)
        if value:
            age = value[2] + samples[index].time_ms - samples[prior_index].time_ms
            return (value[0], value[1], age) if age <= max_age_ms else None
        if samples[prior_index].raw.get(sensor, "none") == "none":
            return None
    return None


def parse_log(path: Path) -> ParsedLog:
    config: dict[str, str] = {}
    samples: list[Sample] = []
    events: list[dict[str, str]] = []
    corrections: list[dict[str, str]] = []
    overflow = False
    truncated = False
    last_sequences: dict[str, int] = {}
    duplicates = 0
    ordering_errors = 0
    last_time = -1

    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "LOG BUFFER OVERFLOW" in raw_line:
            overflow = True
        if raw_line.startswith(CONFIG_PREFIX):
            config = fields(raw_line[len(CONFIG_PREFIX):])
            continue
        if not raw_line.startswith(PREFIX):
            continue
        record = fields(raw_line[len(PREFIX):])
        schema = int(record.get("v", "0"))
        if schema != SUPPORTED_SCHEMA:
            raise ValueError(f"{path}: unsupported diagnostic schema {schema}")
        kind = record.get("type", "")
        record_time = int(record.get("t", "0"))
        if record_time < last_time:
            ordering_errors += 1
        last_time = max(last_time, record_time)
        truncated |= kind == "truncated" or "truncated" in record.get("detail", "")
        if kind == "correction":
            corrections.append(record)
        elif kind != "sample":
            events.append(record)
        else:
            if not config:
                raise ValueError(f"{path}: sample appears before PARK_DIAG_CONFIG")
            sample = Sample(
                time_ms=int(record["t"]), state=record["state"],
                segment=int(record["seg"]), turn=int(record["turn"]),
                encoder_mm=float(record["emm"]), command=int(record["cmd"]),
                speed=float(record["speed"]), steering=float(record["steer"]),
                dc_state=int(record["dc"]), gyro_deg=float(record["gyro"]),
                pose=triple(record["pose"]), nominal=triple(record["nominal"]),
                reference=record.get("ref", "none"), raw=record)
            samples.append(sample)
            for sensor in ("s0", "s1", "s2"):
                value = record.get(sensor, "none")
                if value in {"none", "same"}:
                    continue
                sequence = int(value.split(",", 1)[0])
                if last_sequences.get(sensor) == sequence:
                    duplicates += 1
                last_sequences[sensor] = sequence

    if not config:
        raise ValueError(f"{path}: missing PARK_DIAG_CONFIG record")
    if int(config.get("v", "0")) != SUPPORTED_SCHEMA:
        raise ValueError(f"{path}: unsupported config schema")
    if not samples:
        raise ValueError(f"{path}: no diagnostic samples")
    return ParsedLog(path, config, samples, events, corrections,
                     overflow, truncated, duplicates, ordering_errors)


def neutral_points(samples: list[Sample]) -> list[tuple[float, float, str, int, str]]:
    points: list[tuple[float, float, str, int, str]] = []
    last_approach = "unknown"
    for previous, current in zip(samples, samples[1:]):
        steering_delta = current.steering - previous.steering
        if steering_delta > 0:
            last_approach = "increasing"
        elif steering_delta < 0:
            last_approach = "decreasing"
        if current.state not in {"rear_drive", "localize_drive"}:
            continue
        distance = current.encoder_mm - previous.encoder_mm
        if abs(distance) < 1.0 or abs(current.speed) < 20.0:
            continue
        if not all(math.isfinite(value) for value in
                   (distance, current.speed, current.steering,
                    current.gyro_deg, previous.gyro_deg)):
            continue
        if current.steering != previous.steering or current.command == 0:
            continue
        curvature = wrap180(current.gyro_deg - previous.gyro_deg) / distance
        direction = "forward" if distance > 0 else "reverse"
        points.append((current.steering, curvature, direction,
                       current.turn, last_approach))
    return points


def fit_neutral(points: list[tuple[float, float, str, int, str]]) -> dict[str, float] | None:
    xy = [(point[0], point[1]) for point in points]
    if len(xy) < 3:
        return None
    mean_x = statistics.fmean(x for x, _ in xy)
    mean_y = statistics.fmean(y for _, y in xy)
    variance = sum((x - mean_x) ** 2 for x, _ in xy)
    if variance < 1e-6:
        return None
    slope = sum((x - mean_x) * (y - mean_y) for x, y in xy) / variance
    if abs(slope) < 1e-8:
        return None
    intercept = mean_y - slope * mean_x
    logical_zero = -intercept / slope
    residuals = [y - (slope * x + intercept) for x, y in xy]
    return {
        "samples": float(len(xy)), "slope": slope, "intercept": intercept,
        "logical_zero": logical_zero,
        "residual_rms": math.sqrt(statistics.fmean(r * r for r in residuals)),
    }


def linear_neutral(samples: list[Sample]) -> dict[str, float] | None:
    return fit_neutral(neutral_points(samples))


def grouped_neutral_points(points: list[tuple[float, float, str, int, str]]) -> list[tuple[str, dict[str, float]]]:
    groups: list[tuple[str, dict[str, float]]] = []
    selectors = {
        "forward": lambda p: p[2] == "forward",
        "reverse": lambda p: p[2] == "reverse",
        "clockwise-exit": lambda p: p[3] > 0,
        "counterclockwise-exit": lambda p: p[3] < 0,
        "increasing-approach": lambda p: p[4] == "increasing",
        "decreasing-approach": lambda p: p[4] == "decreasing",
    }
    for name, selector in selectors.items():
        result = fit_neutral([point for point in points if selector(point)])
        if result:
            groups.append((name, result))
    return groups


def grouped_neutral(samples: list[Sample]) -> list[tuple[str, dict[str, float]]]:
    return grouped_neutral_points(neutral_points(samples))


def reversal_rows(parsed: ParsedLog) -> list[dict[str, object]]:
    """Summarize natural command reversals; only rear-marker geometry is resolved."""
    rows: list[dict[str, object]] = []
    prior_nonzero: Sample | None = None
    prior_index: int | None = None
    for index, sample in enumerate(parsed.samples):
        if sample.command == 0:
            continue
        if prior_nonzero and prior_index is not None and sample.command * prior_nonzero.command < 0:
            row: dict[str, object] = {
                "file": parsed.path.name,
                "time_ms": sample.time_ms,
                "from_command": prior_nonzero.command,
                "to_command": sample.command,
                "encoder_delta_mm": sample.encoder_mm - prior_nonzero.encoder_mm,
                "reference": sample.reference,
                "observed_motion_mm": "unobservable",
                "effective_lost_motion_mm": "unobservable",
                "uncertainty_mm": "unobservable",
            }
            if (sample.reference == prior_nonzero.reference == "rear_marker"):
                before = resolved_tof(parsed.samples, prior_index, "s2")
                after = resolved_tof(parsed.samples, index, "s2")
                if before and after:
                    observed = -(after[0] - before[0])
                    encoder = sample.encoder_mm - prior_nonzero.encoder_mm
                    row["observed_motion_mm"] = observed
                    row["effective_lost_motion_mm"] = abs(encoder) - abs(observed)
                    row["uncertainty_mm"] = math.hypot(before[1], after[1])
            rows.append(row)
        prior_nonzero = sample
        prior_index = index
    return rows


def segment_rows(parsed: ParsedLog) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for segment in sorted({sample.segment for sample in parsed.samples}):
        group = [sample for sample in parsed.samples if sample.segment == segment]
        if not group:
            continue
        brake = [sample for sample in group if "brake" in sample.state]
        rows.append({
            "file": parsed.path.name,
            "segment": segment,
            "samples": len(group),
            "encoder_delta_mm": group[-1].encoder_mm - group[0].encoder_mm,
            "brake_travel_mm": (brake[-1].encoder_mm - brake[0].encoder_mm)
            if len(brake) > 1 else 0.0,
            "heading_change_deg": wrap180(group[-1].gyro_deg - group[0].gyro_deg),
            "max_position_error_mm": max(s.position_error_mm for s in group),
            "settled_position_error_mm": group[-1].position_error_mm,
            "settled_heading_error_deg": group[-1].heading_error_deg,
        })
    return rows


def write_pose_svg(log: ParsedLog, path: Path) -> None:
    """Write a dependency-free actual-versus-nominal top-down trace."""
    all_points = [(s.pose[0], s.pose[1]) for s in log.samples]
    all_points += [(s.nominal[0], s.nominal[1]) for s in log.samples]
    min_x = min(x for x, _ in all_points)
    max_x = max(x for x, _ in all_points)
    min_y = min(y for _, y in all_points)
    max_y = max(y for _, y in all_points)
    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)
    def points(values: list[tuple[float, float]]) -> str:
        return " ".join(
            f"{30 + (x - min_x) * 540 / span_x:.1f},"
            f"{370 - (y - min_y) * 340 / span_y:.1f}" for x, y in values)
    actual = points([(s.pose[0], s.pose[1]) for s in log.samples])
    nominal = points([(s.nominal[0], s.nominal[1]) for s in log.samples])
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="600" height="400" viewBox="0 0 600 400">
<rect width="600" height="400" fill="white"/>
<polyline points="{nominal}" fill="none" stroke="#777" stroke-width="3" stroke-dasharray="7 5"/>
<polyline points="{actual}" fill="none" stroke="#1769aa" stroke-width="3"/>
<text x="30" y="22" font-family="sans-serif" font-size="14">blue: estimated pose; dashed: nominal</text>
</svg>"""
    path.write_text(svg, encoding="utf-8")


def analyze(paths: list[Path]) -> tuple[list[ParsedLog], list[dict[str, object]]]:
    parsed = [parse_log(path) for path in paths]
    return parsed, [row for log in parsed for row in segment_rows(log)]


def write_report(parsed: list[ParsedLog], rows: list[dict[str, object]], output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    csv_path = output / "parking_exit_segments.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else ["file"])
        writer.writeheader()
        writer.writerows(rows)

    reversal_data = [row for log in parsed for row in reversal_rows(log)]
    reversal_path = output / "parking_exit_reversals.csv"
    reversal_fields = list(reversal_data[0]) if reversal_data else ["file"]
    with reversal_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=reversal_fields)
        writer.writeheader()
        writer.writerows(reversal_data)

    lines = ["# Parking-exit diagnostic analysis", ""]
    combined_points = [point for log in parsed for point in neutral_points(log.samples)]
    combined = fit_neutral(combined_points)
    if combined:
        centers = {float(log.config.get("center", "nan")) for log in parsed}
        center_text = (f"; candidate raw centre {next(iter(centers)) + combined['logical_zero']:.2f} degrees"
                       if len(centers) == 1 else "")
        lines.extend([
            f"Combined neutral offset: {combined['logical_zero']:+.2f} degrees"
            f"{center_text} (n={int(combined['samples'])}).",
            "",
        ])
    else:
        lines.extend(["Combined neutral estimate: insufficient stable steering variation.", ""])
    combined_groups = grouped_neutral_points(combined_points)
    if len(combined_groups) >= 2:
        offsets = [value["logical_zero"] for _, value in combined_groups]
        uncertainties = [value["residual_rms"] / abs(value["slope"])
                         for _, value in combined_groups]
        spread = max(offsets) - min(offsets)
        if spread > 2.0 * max(uncertainties):
            lines.extend([
                f"Neutral subgroups disagree: offset range {min(offsets):+.2f} to "
                f"{max(offsets):+.2f} degrees. Treat this as hysteresis/spread, "
                "not one optimum.", "",
            ])
    for log in parsed:
        digest = hashlib.sha256(log.path.read_bytes()).hexdigest()
        neutral = linear_neutral(log.samples)
        finish = [event for event in log.events if event.get("type") == "finish"]
        configured = float(log.config.get("center", "nan"))
        lines.extend([
            f"## {log.path.name}", "",
            f"- SHA-256: `{digest}`",
            f"- Schema: {log.config.get('v')}; samples: {len(log.samples)}; "
            f"events: {len(log.events)}; corrections: {len(log.corrections)}",
            f"- Overflow: {'yes' if log.overflow else 'no'}; diagnostic truncation: "
            f"{'yes' if log.truncated else 'no'}; duplicate ToF snapshots: "
            f"{log.duplicate_tof_sequences}; ordering errors: {log.ordering_errors}",
            f"- Completion record: {finish[-1].get('detail', 'yes') if finish else 'missing'}; "
            f"natural reversals: {len(reversal_rows(log))}",
        ])
        if neutral:
            candidate = configured + neutral["logical_zero"]
            lines.append(
                f"- Servo-neutral candidate: {candidate:.2f} raw servo degrees "
                f"(logical offset {neutral['logical_zero']:+.2f}, "
                f"n={int(neutral['samples'])}, residual RMS "
                f"{neutral['residual_rms']:.5f} deg/mm).")
        else:
            lines.append("- Servo-neutral candidate: unavailable (insufficient stable steering variation).")
        groups = grouped_neutral(log.samples)
        if groups:
            descriptions = [
                f"{name} {configured + value['logical_zero']:.2f} degrees "
                f"(n={int(value['samples'])})" for name, value in groups]
            lines.append("- Neutral groups: " + "; ".join(descriptions) + ".")
        lines.extend(["", "The candidate is diagnostic only and must not update `SERVO_CENTER` automatically.", ""])
        safe_name = "".join(character if character.isalnum() else "_"
                            for character in log.path.stem)
        write_pose_svg(log, output / f"{safe_name}_pose.svg")

    repeatability: list[dict[str, object]] = []
    for segment in sorted({int(row["segment"]) for row in rows}):
        group = [row for row in rows if row["segment"] == segment]
        repeatability.append({
            "segment": segment,
            "runs": len(group),
            "position_error_mean_mm": statistics.fmean(
                float(row["settled_position_error_mm"]) for row in group),
            "position_error_spread_mm": statistics.pstdev(
                float(row["settled_position_error_mm"]) for row in group),
            "heading_error_mean_deg": statistics.fmean(
                float(row["settled_heading_error_deg"]) for row in group),
            "heading_error_spread_deg": statistics.pstdev(
                float(row["settled_heading_error_deg"]) for row in group),
        })
    repeatability_path = output / "parking_exit_repeatability.csv"
    repeatability_fields = list(repeatability[0]) if repeatability else ["segment"]
    with repeatability_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=repeatability_fields)
        writer.writeheader()
        writer.writerows(repeatability)
    (output / "parking_exit_analysis.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path,
                        default=Path("local_workspace/parking-exit-analysis"))
    args = parser.parse_args()
    parsed, rows = analyze(args.logs)
    write_report(parsed, rows, args.output_dir)
    print(f"Analyzed {len(parsed)} log(s); wrote {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
