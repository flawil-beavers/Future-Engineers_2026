#!/usr/bin/env python3
"""Analyze versioned parking-exit diagnostic records using only stdlib."""

from __future__ import annotations

import argparse
import csv
import hashlib
import html
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
    session_index: int = 1
    session_count: int = 1
    source_line_start: int = 1
    source_line_end: int = 0
    source_sha256: str = ""

    @property
    def label(self) -> str:
        if self.session_count == 1:
            return self.path.name
        return f"{self.path.name}#session-{self.session_index}"

    @property
    def exit_complete(self) -> bool:
        return any(event.get("type") == "finish" and
                   event.get("detail", "").startswith("unparking_complete")
                   for event in self.events)


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
        if samples[prior_index].raw.get(sensor) != "same":
            # A newer invalid measurement supersedes an older valid one.
            # Only an explicit unchanged sequence may reuse prior evidence.
            return None
    return None


def _parse_lines(path: Path, raw_lines: list[str], session_index: int = 1,
                 session_count: int = 1, source_line_start: int = 1,
                 source_line_end: int = 0,
                 source_sha256: str = "") -> ParsedLog:
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
    diagnostic_bytes = 0
    sample_bytes = 0
    event_bytes = 0

    for raw_line in raw_lines:
        if "LOG BUFFER OVERFLOW" in raw_line:
            overflow = True
        if raw_line.startswith(CONFIG_PREFIX):
            line_bytes = len(raw_line.encode("utf-8")) + 2
            diagnostic_bytes += line_bytes
            event_bytes += line_bytes
            config = fields(raw_line[len(CONFIG_PREFIX):])
            continue
        if not raw_line.startswith(PREFIX):
            continue
        line_bytes = len(raw_line.encode("utf-8")) + 2
        diagnostic_bytes += line_bytes
        record = fields(raw_line[len(PREFIX):])
        schema = int(record.get("v", "0"))
        if schema != SUPPORTED_SCHEMA:
            raise ValueError(f"{path}: unsupported diagnostic schema {schema}")
        kind = record.get("type", "")
        if kind == "sample":
            sample_bytes += line_bytes
        else:
            event_bytes += line_bytes
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
                if sensor not in record:
                    raise ValueError(f"{path}: incomplete sample at {record_time}: missing {sensor}")
                value = record[sensor]
                if value in {"none", "same"}:
                    continue
                if len(value.split(",")) != 8:
                    raise ValueError(f"{path}: incomplete {sensor} at {record_time}")
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
    if len(samples) > int(config.get("sample_limit", "150")):
        raise ValueError(f"{path}: sample count exceeds configured limit")
    if diagnostic_bytes > int(config.get("byte_limit", "65536")):
        raise ValueError(f"{path}: diagnostic bytes exceed configured limit")
    if "sample_bytes" in config and sample_bytes > int(config["sample_bytes"]):
        raise ValueError(f"{path}: sample bytes exceed configured limit")
    if "event_bytes" in config and event_bytes > int(config["event_bytes"]):
        raise ValueError(f"{path}: event bytes exceed configured limit")
    return ParsedLog(path, config, samples, events, corrections,
                     overflow, truncated, duplicates, ordering_errors,
                     session_index, session_count, source_line_start,
                     source_line_end or len(raw_lines), source_sha256)


def parse_log(path: Path) -> ParsedLog:
    raw = path.read_bytes()
    lines = raw.decode("utf-8", errors="replace").splitlines()
    return _parse_lines(path, lines, source_line_end=len(lines),
                        source_sha256=hashlib.sha256(raw).hexdigest())


def parse_log_sessions(path: Path) -> list[ParsedLog]:
    """Expand a source containing multiple diagnostic headers into run sessions."""
    raw = path.read_bytes()
    lines = raw.decode("utf-8", errors="replace").splitlines()
    starts = [index for index, line in enumerate(lines)
              if line.startswith(CONFIG_PREFIX)]
    if len(starts) <= 1:
        return [_parse_lines(path, lines, source_line_end=len(lines),
                             source_sha256=hashlib.sha256(raw).hexdigest())]
    digest = hashlib.sha256(raw).hexdigest()
    sessions: list[ParsedLog] = []
    for number, start in enumerate(starts, 1):
        end = starts[number] if number < len(starts) else len(lines)
        sessions.append(_parse_lines(
            path, lines[start:end], number, len(starts), start + 1, end, digest))
    return sessions


def neutral_points(samples: list[Sample]) -> list[tuple[float, float, str, int, str]]:
    points: list[tuple[float, float, str, int, str]] = []
    last_approach = "unknown"
    for previous, current in zip(samples, samples[1:]):
        steering_delta = current.steering - previous.steering
        if steering_delta > 0:
            last_approach = "increasing"
        elif steering_delta < 0:
            last_approach = "decreasing"
        if (current.state not in {"rear_drive", "localize_drive"}
                or previous.state != current.state):
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
    # A nearly flat fit can extrapolate to arbitrary servo angles. Such a
    # zero outside the observed commands is not an identified neutral point.
    if not math.isfinite(logical_zero) or not min(x for x, _ in xy) <= logical_zero <= max(x for x, _ in xy):
        return None
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
        "clockwise-exit": lambda p: p[3] < 0,
        "counterclockwise-exit": lambda p: p[3] > 0,
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
                "file": parsed.label,
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
                    observed = after[0] - before[0]
                    encoder = sample.encoder_mm - prior_nonzero.encoder_mm
                    row["observed_motion_mm"] = observed
                    row["effective_lost_motion_mm"] = abs(encoder) - abs(observed)
                    row["uncertainty_mm"] = math.hypot(before[1], after[1])
            rows.append(row)
        prior_nonzero = sample
        prior_index = index
    return rows


def rear_motion_rows(parsed: ParsedLog) -> list[dict[str, object]]:
    """Compare straight rear-positioning encoder travel with a fixed rear marker."""
    rows: list[dict[str, object]] = []
    samples = parsed.samples
    index = 0
    move = 0
    while index < len(samples):
        if (samples[index].state != "rear_drive" or
                (index and samples[index - 1].state == "rear_drive")):
            index += 1
            continue
        start = index
        cursor = start + 1
        saw_settle = False
        while cursor < len(samples):
            state = samples[cursor].state
            if state == "rear_settle":
                saw_settle = True
            if saw_settle and state == "rear_drive":
                break
            if state not in {"rear_drive", "rear_brake", "rear_settle"}:
                break
            cursor += 1
        block = samples[start:cursor]
        settled_indices = [start + offset for offset, sample in enumerate(block)
                           if sample.state == "rear_settle"]
        command = next((sample.command for sample in block if sample.command), 0)
        move += 1
        row: dict[str, object] = {
            "file": parsed.label,
            "move": move,
            "direction": "forward" if command > 0 else "reverse" if command < 0 else "unknown",
            "encoder_motion_mm": "unobservable",
            "tof_motion_mm": "unobservable",
            "encoder_minus_tof_mm": "unobservable",
            "uncertainty_mm": "unobservable",
            "start_range_mm": "unobservable",
            "end_range_mm": "unobservable",
        }
        if settled_indices and command:
            end = settled_indices[-1]
            before = resolved_tof(samples, start, "s2")
            after = resolved_tof(samples, end, "s2")
            if before and after:
                encoder_motion = samples[end].encoder_mm - samples[start].encoder_mm
                tof_motion = after[0] - before[0]
                # Include sensor sigma, integer-range quantization and residual
                # settling visible between the final two fresh settle samples.
                recent = []
                for settle_index in settled_indices[-3:]:
                    value = tof_value(samples[settle_index], "s2")
                    if value:
                        recent.append(value[0])
                settling_span = max(recent) - min(recent) if len(recent) > 1 else 0.0
                uncertainty = math.hypot(before[1], after[1]) + 1.0 + settling_span
                if encoder_motion * command > 0 and tof_motion * command > 0:
                    row.update({
                        "encoder_motion_mm": encoder_motion,
                        "tof_motion_mm": tof_motion,
                        "encoder_minus_tof_mm": encoder_motion - tof_motion,
                        "uncertainty_mm": uncertainty,
                        "start_range_mm": before[0],
                        "end_range_mm": after[0],
                    })
        rows.append(row)
        index = max(cursor, index + 1)
    return rows


def correction_rows(parsed: ParsedLog) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    direction = "cw" if parsed.samples[0].turn < 0 else "ccw"
    for correction in parsed.corrections:
        delta = triple(correction["delta"])
        rows.append({
            "file": parsed.label,
            "direction": direction,
            "time_ms": int(correction["t"]),
            "source": correction["source"],
            "dx_mm": delta[0],
            "dy_mm": delta[1],
            "heading_deg": delta[2],
            "xy_magnitude_mm": math.hypot(delta[0], delta[1]),
        })
    return rows


def segment_rows(parsed: ParsedLog) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    def key(sample: Sample) -> tuple[str, int]:
        if sample.state.startswith("rear_"):
            return "rear_positioning", 0
        if sample.state.startswith("localize_"):
            return "localization", sample.segment
        return "exit", sample.segment

    # The firmware reuses seg=1 for rear positioning and the first exit arc.
    # Keep phases separate. A brake may be shorter than one periodic sample,
    # so use state/finish event boundaries instead of spanning all brake samples.
    brake_motion: dict[tuple[str, int], float] = {}
    boundaries = [event for event in parsed.events
                  if event.get("type") in {"state", "finish"}]
    for event, following in zip(boundaries, boundaries[1:]):
        if "brake" not in event.get("detail", ""):
            continue
        candidates = [sample for sample in parsed.samples
                      if sample.time_ms == int(event["t"])]
        if candidates and "emm" in following:
            phase_key = key(candidates[0])
            brake_motion[phase_key] = brake_motion.get(phase_key, 0.0) + (
                float(following["emm"]) - float(event["emm"]))

    for phase, segment in sorted({key(sample) for sample in parsed.samples},
                                 key=lambda value: (value[1], value[0])):
        group = [sample for sample in parsed.samples if key(sample) == (phase, segment)]
        endpoint = group[-1]
        if phase == "exit" and endpoint.state == "segment_brake":
            later = [sample for sample in parsed.samples if sample.time_ms > endpoint.time_ms]
            if later and later[0].state in {"segment_settle", "localize_settle", "localize_drive"}:
                endpoint = later[0]
        brake = brake_motion.get((phase, segment))
        # Synthetic/older fixtures may lack state events. Recover each brake
        # episode separately from its first sample to the next state or finish.
        if brake is None:
            episodes = []
            for index, sample in enumerate(parsed.samples):
                if key(sample) != (phase, segment) or "brake" not in sample.state:
                    continue
                if index and parsed.samples[index - 1].state == sample.state:
                    continue
                next_states = [s for s in parsed.samples[index + 1:] if s.state != sample.state]
                if next_states:
                    episodes.append(next_states[0].encoder_mm - sample.encoder_mm)
                else:
                    finishes = [e for e in parsed.events if e.get("type") == "finish"
                                and int(e.get("t", "0")) >= sample.time_ms and "emm" in e]
                    if finishes:
                        episodes.append(float(finishes[0]["emm"]) - sample.encoder_mm)
            brake = sum(episodes) if episodes else "unobservable"
        rows.append({
            "file": parsed.label,
            "direction": "cw" if group[0].turn < 0 else "ccw",
            "phase": phase,
            "segment": segment,
            "samples": len(group),
            "encoder_delta_mm": endpoint.encoder_mm - group[0].encoder_mm,
            "brake_travel_mm": brake,
            "heading_change_deg": wrap180(endpoint.gyro_deg - group[0].gyro_deg),
            "max_position_error_mm": max(s.position_error_mm for s in group),
            "last_position_error_mm": endpoint.position_error_mm,
            "last_heading_error_deg": endpoint.heading_error_deg,
        })
    return rows


def write_pose_svg(log: ParsedLog, path: Path, exit_only: bool = False) -> None:
    """Draw one pose frame at a true, shared X/Y scale in millimetres."""
    field_samples = [sample for sample in log.samples
                     if sample.state.startswith(("segment_", "localize_"))]
    frame = "field" if field_samples else "rear-local"
    selected = field_samples if field_samples else [sample for sample in log.samples
                                                    if sample.state.startswith("rear_")]
    if exit_only:
        selected = [sample for sample in field_samples
                    if sample.state.startswith("segment_")]
        if not selected:
            return
    if not selected:
        return
    corrections = [] if exit_only or frame != "field" else [
        (triple(event["before"]), triple(event["after"]))
        for event in log.corrections if "before" in event and "after" in event]
    all_points = [(value[0], value[1]) for sample in selected
                  for value in (sample.pose, sample.nominal)]
    all_points += [(value[0], value[1]) for before, after in corrections
                   for value in (before, after)]
    min_x = min(x for x, _ in all_points)
    max_x = max(x for x, _ in all_points)
    min_y = min(y for _, y in all_points)
    max_y = max(y for _, y in all_points)
    scale = min(600.0 / max(max_x - min_x + 20.0, 20.0),
                365.0 / max(max_y - min_y + 20.0, 20.0))
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0

    def xy(value: tuple[float, float, float]) -> tuple[float, float]:
        return (360.0 + (value[0] - center_x) * scale,
                265.0 - (value[1] - center_y) * scale)

    def point(value: tuple[float, float, float]) -> str:
        x, y = xy(value)
        return f"{x:.1f},{y:.1f}"

    title = ("Five-segment exit" if exit_only else
             "Exit and edge localization" if frame == "field" else
             "Rear positioning before field-pose reset")
    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="720" height="530" '
        f'viewBox="0 0 720 530" data-frame="{frame}" '
        f'data-scale-px-per-mm="{scale:.8f}">',
        '<rect width="720" height="530" fill="white"/>',
        f'<text x="35" y="30" font-family="sans-serif" font-size="17">'
        f'{html.escape(title)}: {html.escape(log.label)}</text>',
        '<rect x="40" y="65" width="640" height="400" fill="none" stroke="#bbb"/>',
    ]
    phases = ("segment_", "localize_") if frame == "field" and not exit_only else (
        ("segment_",) if exit_only else ("rear_",))
    correction_times = sorted(int(event["t"]) for event in log.corrections)
    for prefix in phases:
        group = [sample for sample in selected if sample.state.startswith(prefix)]
        if not group:
            continue
        phase_name = "exit" if prefix == "segment_" else (
            "localization" if prefix == "localize_" else "rear-positioning")
        nominal_points = " ".join(point(sample.nominal) for sample in group)
        elements.append(
            f'<polyline data-series="nominal" data-phase="{phase_name}" '
            f'points="{nominal_points}" fill="none" stroke="#777" '
            f'stroke-width="2" stroke-dasharray="6 5"/>')
        # A sensor correction changes the pose estimate without chassis motion.
        # Keep that jump separate from the driven trajectory.
        parts: list[list[Sample]] = [[group[0]]]
        for previous, current in zip(group, group[1:]):
            if any(previous.time_ms < time <= current.time_ms
                   for time in correction_times):
                parts.append([])
            parts[-1].append(current)
        color = "#1769aa" if phase_name == "exit" else (
            "#008b80" if phase_name == "localization" else "#1769aa")
        for part in parts:
            actual_points = " ".join(point(sample.pose) for sample in part)
            elements.append(
                f'<polyline data-series="estimated" data-phase="{phase_name}" '
                f'points="{actual_points}" fill="none" stroke="{color}" '
                f'stroke-width="3"/>')
    for before, after in corrections:
        elements.append(
            f'<line data-series="correction" x1="{xy(before)[0]:.1f}" '
            f'y1="{xy(before)[1]:.1f}" x2="{xy(after)[0]:.1f}" '
            f'y2="{xy(after)[1]:.1f}" stroke="#a34ab5" '
            f'stroke-width="2" stroke-dasharray="3 3"/>')
    end_pose = selected[-1].pose
    if corrections and int(log.corrections[-1]["t"]) >= selected[-1].time_ms:
        end_pose = corrections[-1][1]
    for role, pose, color in (("start", selected[0].pose, "#149443"),
                              ("end", end_pose, "#222")):
        x, y = xy(pose)
        elements.append(f'<circle data-role="{role}" cx="{x:.1f}" '
                        f'cy="{y:.1f}" r="5" fill="{color}"/>')
    span_mm = max(max_x - min_x, max_y - min_y)
    bar_mm = 100 if span_mm > 220 else 50 if span_mm > 80 else 10
    bar_px = bar_mm * scale
    elements.extend([
        f'<line x1="50" y1="490" x2="{50 + bar_px:.1f}" y2="490" '
        f'stroke="#222" stroke-width="3"/>',
        f'<text x="50" y="510" font-family="sans-serif" font-size="12">'
        f'{bar_mm} mm; X right, Y up; same scale on both axes</text>',
        '<text x="35" y="54" font-family="sans-serif" font-size="12">'
        'blue: exit; teal: localization; grey: nominal; purple: pose correction</text>',
        '</svg>',
    ])
    path.write_text("\n".join(elements) + "\n", encoding="utf-8")


def analyze(paths: list[Path]) -> tuple[list[ParsedLog], list[dict[str, object]]]:
    parsed = [session for path in paths for session in parse_log_sessions(path)]
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

    rear_data = [row for log in parsed for row in rear_motion_rows(log)]
    rear_path = output / "parking_exit_rear_motion.csv"
    rear_fields = list(rear_data[0]) if rear_data else ["file"]
    with rear_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rear_fields)
        writer.writeheader()
        writer.writerows(rear_data)

    correction_data = [row for log in parsed for row in correction_rows(log)]
    correction_path = output / "parking_exit_corrections.csv"
    correction_fields = list(correction_data[0]) if correction_data else ["file"]
    with correction_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=correction_fields)
        writer.writeheader()
        writer.writerows(correction_data)

    lines = ["# Parking-exit diagnostic analysis", ""]
    lines.extend([
        "Pose errors compare the onboard estimate with the nominal model; they are not independent ground truth.",
        "Brake travel is signed encoder movement between brake-start and the following state/finish event.",
        "Neutral fits use unchanged sampled steering, but do not establish physical servo settling or remove feedback/sensor lag. Treat candidates as exploratory.",
        "Last endpoint errors are not necessarily settled measurements, especially during localization corrections.", "",
        "Pose SVGs use equal X/Y millimetre scales. The exit-only view omits rear positioning (a different pose frame) and the later straight reverse localization; the full view shows exit and localization with sensor corrections marked separately. These are rear-axle traces, not robot footprints or parking boundaries.",
        "",
    ])
    complete_logs = [log for log in parsed if log.exit_complete and
                     not log.overflow and not log.truncated]
    complete_labels = {log.label for log in complete_logs}
    lines.extend([
        f"Combined summaries use {len(complete_logs)} completed, untruncated exit sessions "
        f"of {len(parsed)} parsed sessions. Other sessions remain in the per-session report and CSVs.",
        "Physical setup and outcome still require review in the evidence README; completion alone does not prove a valid run.",
        "",
    ])
    combined_points = [point for log in complete_logs for point in neutral_points(log.samples)]
    combined = fit_neutral(combined_points)
    if combined:
        centers = {float(log.config.get("center", "nan")) for log in complete_logs}
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
    if combined_groups:
        centers = {float(log.config.get("center", "nan")) for log in complete_logs}
        if len(centers) == 1:
            configured = next(iter(centers))
            descriptions = [
                f"{name} {configured + value['logical_zero']:.2f} raw degrees "
                f"(n={int(value['samples'])})"
                for name, value in combined_groups]
        else:
            configured = None
            descriptions = [
                f"{name} logical offset {value['logical_zero']:+.2f} degrees "
                f"(n={int(value['samples'])})"
                for name, value in combined_groups]
        lines.extend(["Combined neutral groups: " + "; ".join(descriptions) + ".", ""])
        approaches = {name: value for name, value in combined_groups
                      if name in {"increasing-approach", "decreasing-approach"}}
        if configured is not None and len(approaches) == 2:
            candidates = [configured + value["logical_zero"]
                          for value in approaches.values()]
            lines.extend([
                f"Steering-approach candidate span: {min(candidates):.2f} to "
                f"{max(candidates):.2f} degrees; midpoint {statistics.fmean(candidates):.2f} degrees. "
                "This is an exploratory hysteresis/lag result, not an automatic calibration.",
                "",
            ])
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
    observable_rear = [row for row in rear_data
                       if row["file"] in complete_labels and
                       row["encoder_minus_tof_mm"] != "unobservable"]
    if observable_rear:
        differences = [float(row["encoder_minus_tof_mm"])
                       for row in observable_rear]
        within_uncertainty = sum(
            abs(float(row["encoder_minus_tof_mm"])) <= float(row["uncertainty_mm"])
            for row in observable_rear)
        reverse_count = sum(row["direction"] == "reverse" for row in observable_rear)
        lines.extend([
            f"Rear-marker motion agreement: {len(observable_rear)} moves "
            f"({reverse_count} reverse); median encoder-minus-ToF "
            f"{statistics.median(differences):+.2f} mm, spread "
            f"{statistics.pstdev(differences):.2f} mm, maximum absolute "
            f"{max(abs(value) for value in differences):.2f} mm; "
            f"{within_uncertainty}/{len(observable_rear)} within the conservative sensor/settling uncertainty.",
            "This checks total straight motion against the rear marker; it cannot isolate gearbox backlash from tire slip, compliance or ToF error.",
            "",
        ])
    summary_corrections = [row for row in correction_data
                           if row["file"] in complete_labels]
    for source, direction in sorted({(str(row["source"]), str(row["direction"]))
                                     for row in summary_corrections}):
        group = [row for row in summary_corrections
                 if row["source"] == source and row["direction"] == direction]
        dx_values = [float(row["dx_mm"]) for row in group]
        dy_values = [float(row["dy_mm"]) for row in group]
        lines.append(
            f"{source} corrections ({direction}, n={len(group)}): mean dx "
            f"{statistics.fmean(dx_values):+.1f} mm (spread {statistics.pstdev(dx_values):.1f}), "
            f"mean dy {statistics.fmean(dy_values):+.1f} mm "
            f"(spread {statistics.pstdev(dy_values):.1f}), "
            f"XY magnitude range {min(float(row['xy_magnitude_mm']) for row in group):.1f} to "
            f"{max(float(row['xy_magnitude_mm']) for row in group):.1f} mm.")
    if summary_corrections:
        lines.extend([
            "Corrections are existing sensor-based estimator updates, not independent measurements of absolute pose.",
            "",
        ])
    for log in parsed:
        digest = log.source_sha256 or hashlib.sha256(log.path.read_bytes()).hexdigest()
        neutral = linear_neutral(log.samples)
        finish = [event for event in log.events if event.get("type") == "finish"]
        configured = float(log.config.get("center", "nan"))
        lines.extend([
            f"## {log.label}", "",
            f"- Source SHA-256: `{digest}`",
            f"- Schema: {log.config.get('v')}; samples: {len(log.samples)}; "
            f"events: {len(log.events)}; corrections: {len(log.corrections)}",
            f"- Overflow: {'yes' if log.overflow else 'no'}; diagnostic truncation: "
            f"{'yes' if log.truncated else 'no'}; duplicate ToF snapshots: "
            f"{log.duplicate_tof_sequences}; ordering errors: {log.ordering_errors}",
            f"- Completion record: {finish[-1].get('detail', 'yes') if finish else 'missing'}; "
            f"natural reversals: {len(reversal_rows(log))}",
        ])
        if log.session_count > 1:
            lines.append(
                f"- Source lines: {log.source_line_start}-{log.source_line_end}.")
        safe_name = "".join(character if character.isalnum() else "_"
                            for character in log.label)
        has_exit = any(sample.state.startswith("segment_") for sample in log.samples)
        if has_exit:
            lines.append(
                f"- Pose plots: [{safe_name}_exit_pose.svg]({safe_name}_exit_pose.svg) "
                f"(five-segment exit); [{safe_name}_pose.svg]({safe_name}_pose.svg) "
                "(exit plus localization).")
        else:
            lines.append(
                f"- Pose plot: [{safe_name}_pose.svg]({safe_name}_pose.svg) "
                "(rear positioning, before field-pose reset).")
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
        write_pose_svg(log, output / f"{safe_name}_pose.svg")
        if has_exit:
            write_pose_svg(log, output / f"{safe_name}_exit_pose.svg",
                           exit_only=True)

    repeatability: list[dict[str, object]] = []
    summary_rows = [row for row in rows if row["file"] in complete_labels]
    groups = {(str(row["direction"]), str(row["phase"]), int(row["segment"]))
              for row in summary_rows}
    for direction, phase, segment in sorted(groups):
        group = [row for row in summary_rows if row["direction"] == direction
                 and row["phase"] == phase and row["segment"] == segment]
        brakes = [float(row["brake_travel_mm"]) for row in group
                  if row["brake_travel_mm"] != "unobservable"]
        repeatability.append({
            "direction": direction,
            "phase": phase,
            "segment": segment,
            "runs": len(group),
            "encoder_motion_mean_mm": statistics.fmean(
                float(row["encoder_delta_mm"]) for row in group),
            "encoder_motion_spread_mm": statistics.pstdev(
                float(row["encoder_delta_mm"]) for row in group),
            "brake_motion_median_mm": statistics.median(brakes) if brakes else "unobservable",
            "brake_motion_max_abs_mm": max(map(abs, brakes)) if brakes else "unobservable",
            "heading_change_mean_deg": statistics.fmean(
                float(row["heading_change_deg"]) for row in group),
            "heading_change_spread_deg": statistics.pstdev(
                float(row["heading_change_deg"]) for row in group),
            "position_error_mean_mm": statistics.fmean(
                float(row["last_position_error_mm"]) for row in group),
            "position_error_spread_mm": statistics.pstdev(
                float(row["last_position_error_mm"]) for row in group),
            "heading_error_mean_deg": statistics.fmean(
                float(row["last_heading_error_deg"]) for row in group),
            "heading_error_spread_deg": statistics.pstdev(
                float(row["last_heading_error_deg"]) for row in group),
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
