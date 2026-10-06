"""Offline angle/target assessment of cached parking-exit ToF observations.

No sensor acquisition or firmware control. Geometry is a nominal model, not
ground truth; residuals include odometry error and acquisition timing error.
"""
from __future__ import annotations

import ast
import csv
import hashlib
import html
import math
import re
import statistics
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FANS = (22, 25)
MAX_AGE_MS = 50


def constants(path: Path) -> dict[str, float]:
    """Resolve only arithmetic numeric constexpr expressions, without eval."""
    text = re.sub(r"//[^\n]*|/\*.*?\*/", "", path.read_text(), flags=re.S)
    expressions = dict(re.findall(r"constexpr\s+auto\s+(\w+)\s*=\s*([^;]+);", text))
    values: dict[str, float] = {}

    def resolve(node):
        if isinstance(node, ast.Constant) and type(node.value) in (int, float):
            return float(node.value)
        if isinstance(node, ast.Name):
            return values[node.id]
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
            return resolve(node.operand) * (-1 if isinstance(node.op, ast.USub) else 1)
        if isinstance(node, ast.BinOp):
            a, b = resolve(node.left), resolve(node.right)
            if isinstance(node.op, ast.Add): return a + b
            if isinstance(node.op, ast.Sub): return a - b
            if isinstance(node.op, ast.Mult): return a * b
            if isinstance(node.op, ast.Div): return a / b
        raise ValueError("not a numeric geometry expression")

    for _ in range(len(expressions)):
        before = len(values)
        for name, expression in expressions.items():
            if name in values:
                continue
            expression = re.sub(r"(?<=\d)[fF]\b", "", expression.strip())
            try:
                values[name] = resolve(ast.parse("(" + expression + ")", mode="eval").body)
            except (SyntaxError, KeyError, ValueError, ZeroDivisionError):
                pass
        if before == len(values):
            break
    return values


@dataclass(frozen=True)
class Geometry:
    south: float
    fixed_x: float
    gap: float
    thickness: float
    length: float
    left: tuple[float, float]
    right: tuple[float, float]
    rear: tuple[float, float]
    tolerance: float = 5.0

    @classmethod
    def load(cls):
        c = constants(ROOT / "include/config.h")
        get = lambda name: c["OBSTACLE_" + name]
        return cls(get("SOUTH_OUTER_WALL_Y_MM"), get("PARKING_FIXED_INNER_FACE_X_MM"),
                   get("PARKING_EXIT_PROTOTYPE_GAP_MM"), get("PARKING_LIMIT_THICKNESS_MM"),
                   get("PARKING_WIDTH_MM"),
                   (get("TOF_LEFT_LOCAL_X_MM"), get("TOF_LEFT_LOCAL_Y_MM")),
                   (get("TOF_RIGHT_LOCAL_X_MM"), get("TOF_RIGHT_LOCAL_Y_MM")),
                   (-get("REAR_TOF_BEHIND_AXLE_MM"), 0))

    def surfaces(self, shift=0.0):
        # Only the known south parking section is modeled. Other distant walls
        # are unidentified, rather than guessed from an infinite field model.
        result = [("black_south", (-1500., self.south), (1500., self.south))]
        for name, xmin, xmax in (
                ("pink_fixed", self.fixed_x, self.fixed_x + self.thickness),
                ("pink_moving", self.fixed_x - self.gap - self.thickness + shift,
                 self.fixed_x - self.gap + shift)):
            p = [(xmin, self.south), (xmax, self.south),
                 (xmax, self.south + self.length), (xmin, self.south + self.length)]
            result.extend((f"{name}_{i}", p[i], p[(i + 1) % 4]) for i in range(4))
        return result


def ray_hit(origin, angle, surfaces):
    dx, dy = math.cos(angle), math.sin(angle)
    hits = []
    for name, (ax, ay), (bx, by) in surfaces:
        ex, ey = bx - ax, by - ay
        determinant = dx * ey - dy * ex
        if abs(determinant) < 1e-9:
            continue
        rx, ry = ax - origin[0], ay - origin[1]
        distance = (rx * ey - ry * ex) / determinant
        fraction = (rx * dy - ry * dx) / determinant
        if distance > 1e-6 and -1e-9 <= fraction <= 1 + 1e-9:
            normal_dot = abs((-ey * dx + ex * dy) / math.hypot(ex, ey))
            incidence = math.degrees(math.acos(min(1., normal_dot)))
            edge_distance = min(fraction, 1 - fraction) * math.hypot(ex, ey)
            hits.append((distance, name, incidence, edge_distance))
    return min(hits, default=None)


def classify(origin, angle, geometry, fov):
    center = ray_hit(origin, angle, geometry.surfaces())
    if center is None:
        return "unidentified", None
    # Half-degree fan steps and movable-rail placement extremes. Discrete 2D
    # rays approximate a sensing footprint, not its optical return weighting.
    names = set()
    missing = False
    edge = center[3] <= geometry.tolerance
    for shift in (-geometry.tolerance, 0, geometry.tolerance):
        for step in range(2 * fov + 1):
            hit = ray_hit(origin, angle + math.radians(-fov / 2 + step / 2),
                          geometry.surfaces(shift))
            if hit is None:
                missing = True
            else:
                names.add(hit[1])
                edge |= hit[3] <= geometry.tolerance
    if len(names) > 1:
        return "mixed-surface", center
    if missing or edge:
        return "edge-sensitive", center
    return "single-surface", center


def observation_rows(session, procedure, geometry):
    seen = [set(), set(), set()]
    rebases = [int(e["t"]) for e in session.events
               if e.get("type") == "rebase" and e.get("detail") == "field_start"]
    corrections = sorted(int(e["t"]) for e in session.corrections)
    previous = {}
    rows = []
    for sample in session.samples:
        for sensor in range(3):
            text = sample.raw.get(f"s{sensor}", "none")
            if text in ("same", "none"):
                continue
            parts = text.split(",")
            if len(parts) != 8:
                continue
            sequence, age, raw, filtered, signal, sigma, valid, accepted = map(float, parts)
            if sequence in seen[sensor]:
                continue
            seen[sensor].add(sequence)
            phase = sample.state.split("_", 1)[0]
            field_frame = bool(rebases and sample.time_ms > rebases[0] and phase != "rear")
            row = dict(source=session.path.name, sha256=session.source_sha256,
                       session=session.session_index, build=session.config.get("build", "unknown"),
                       configuration=hashlib.sha256(repr(sorted(session.config.items())).encode()).hexdigest()[:12],
                       procedure=procedure, direction="cw" if sample.turn < 0 else "ccw",
                       exit_complete=getattr(session, "exit_complete", False),
                       overflow=getattr(session, "overflow", False),
                       diagnostic_truncated=getattr(session, "truncated", False),
                       phase=phase, state=sample.state, segment=sample.segment,
                       time_ms=sample.time_ms, sensor=("left", "right", "rear")[sensor],
                       sequence=int(sequence), age_ms=age, raw_mm=raw, filtered_mm=filtered,
                       signal_mcps=signal, sigma_mm=sigma, valid=int(valid), accepted=int(accepted),
                       pose_x_mm=sample.pose[0], pose_y_mm=sample.pose[1], heading_deg=sample.pose[2],
                       pose_epoch=sum(t <= sample.time_ms for t in corrections),
                       raw_filtered_delta_mm=filtered - raw if 0 < filtered < 9999 and raw > 0 else "",
                       raw_step_mm="", target_change_candidate=False,
                       expected_mm="", incidence_deg="", target="", distance_group="unknown",
                       raw_residual_mm="", filtered_residual_mm="", reason="", usable=False)
            theta = math.radians(sample.pose[2])
            local = (geometry.left, geometry.right, geometry.rear)[sensor]
            origin = (sample.pose[0] + local[0] * math.cos(theta) - local[1] * math.sin(theta),
                      sample.pose[1] + local[0] * math.sin(theta) + local[1] * math.cos(theta))
            angle = theta + (math.pi / 2, -math.pi / 2, math.pi)[sensor]
            center = None
            for fov in FANS:
                category, hit = classify(origin, angle, geometry, fov) if field_frame else ("unidentified", None)
                row[f"classification_{fov}"] = category
                center = hit
            if center:
                row.update(expected_mm=center[0], target=center[1], incidence_deg=center[2],
                           distance_group="0-110" if center[0] <= 110 else "110-300" if center[0] <= 300
                           else "300-600" if center[0] <= 600 else "over-600")
            reasons = []
            if not field_frame: reasons.append("pose-seeding/local-frame-or-missing-field-rebase")
            elif sample.time_ms - age <= rebases[0]: reasons.append("acquisition-crosses-field-seed")
            if not valid or not accepted: reasons.append("invalid-or-rejected")
            if age > MAX_AGE_MS: reasons.append("stale")
            if not 0 < raw <= 600: reasons.append("raw-out-of-range")
            if center is None: reasons.append("unidentified-target")
            elif center[0] > 600: reasons.append("predicted-out-of-range")
            if any(row[f"classification_{f}"] != "single-surface" for f in FANS):
                reasons.append("ambiguous-footprint")
            # A cached range predating a pose correction cannot be compared in
            # the corrected frame at its logged heading/position.
            if any(sample.time_ms - age <= t <= sample.time_ms for t in corrections):
                reasons.append("acquisition-crosses-pose-correction")
            prior = previous.get(sensor)
            if prior and raw > 0 and prior["raw_mm"] > 0:
                row["raw_step_mm"] = raw - prior["raw_mm"]
                if (row["target"] and prior["target"] and row["pose_epoch"] == prior["pose_epoch"]):
                    unexplained = row["raw_step_mm"] - (row["expected_mm"] - prior["expected_mm"])
                    threshold = max(20., 3 * math.hypot(sigma, prior["sigma_mm"]))
                    row["target_change_candidate"] = (row["target"] != prior["target"] or
                                                       abs(unexplained) > threshold)
            row["usable"] = not reasons
            row["reason"] = ";".join(reasons)
            if row["usable"]:
                row["raw_residual_mm"] = raw - center[0]
                if 0 < filtered <= 600:
                    row["filtered_residual_mm"] = filtered - center[0]
            previous[sensor] = row
            rows.append(row)
    return rows


def scatter(rows, output, title):
    """SVG panels retain continuous angle and show invalid frames separately."""
    metrics = (("raw_residual_mm", "Raw residual (mm)"),
               ("filtered_residual_mm", "Filtered residual (mm)"),
               ("signal_mcps", "Signal (Mcps)"), ("sigma_mm", "Sigma (mm)"),
               ("invalid", "Invalid/rejected (1=yes)"),
               ("raw_filtered_delta_mm", "Filtered minus raw (mm)"))
    width, panel = 780, 180
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{55 + panel * len(metrics)}">',
           '<rect width="100%" height="100%" fill="white"/>',
           f'<text x="20" y="22" font-size="13">{html.escape(title)}</text>',
           '<text x="20" y="42" font-size="12">Blue: usable; orange: excluded. Angle 0 = perpendicular. Nominal model.</text>']
    for index, (key, label) in enumerate(metrics):
        top = 65 + index * panel
        points = []
        for row in rows:
            value = int(not row["valid"] or not row["accepted"]) if key == "invalid" else row[key]
            if row["incidence_deg"] != "" and value != "" and math.isfinite(float(value)):
                points.append((row, float(value)))
        values = [v for _, v in points]
        low, high = min(values, default=0.), max(values, default=1.)
        if high == low: low, high = low - .5, high + .5
        svg.append(f'<text x="20" y="{top}" font-size="12">{label} [{low:.2f}, {high:.2f}], n={len(points)}</text>')
        svg.append(f'<path d="M60 {top+12} V{top+125} H740" fill="none" stroke="black"/>')
        for tick in (0, 15, 30, 45, 60, 75, 90):
            x = 60 + tick / 90 * 680
            svg.append(f'<text x="{x}" y="{top+143}" font-size="10">{tick}°</text>')
        for row, value in points:
            x = 60 + float(row["incidence_deg"]) / 90 * 680
            y = top + 125 - (value - low) / (high - low) * 110
            color = "#2166ac" if row["usable"] else "#d97706"
            tip = html.escape(f'{row["source"]} s{row["session"]} t={row["time_ms"]} {row["reason"]}')
            svg.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="2" fill="{color}" opacity=".6"><title>{tip}</title></circle>')
    svg.append('</svg>')
    output.write_text("\n".join(svg), encoding="utf-8")


def write_assessment(sessions, output, procedure_for):
    geometry = Geometry.load()
    rows = [row for session in sessions for row in observation_rows(
        session, procedure_for(session), geometry)]
    output.mkdir(parents=True, exist_ok=True)
    columns = list(rows[0]) if rows else ["source", "reason"]
    with (output / "parking_exit_tof_observations.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)
    groups = defaultdict(list)
    for row in rows:
        groups[(row["build"], row["configuration"], row["procedure"], row["direction"], row["sensor"],
                row["phase"], row["distance_group"])].append(row)
    plots = output / "tof-plots"
    plots.mkdir(exist_ok=True)
    text = ["# Offline parking-exit ToF assessment", "",
            "Existing route, braking, reversal and correction reports remain unchanged. "
            "This analysis adds no robot program, sensor sweep, movement or pause.", "",
            "[Observation CSV](parking_exit_tof_observations.csv). Each sequence counts once per sensor/session. "
            "Residuals compare raw/filtered readings with the centre-ray prediction; "
            "they are not external pose error or guaranteed clearance.", "",
            f"Model: south wall y={geometry.south:g} mm; pink rails {geometry.length:g} x "
            f"{geometry.thickness:g} mm; gap {geometry.gap:g} +/-{geometry.tolerance:g} mm. "
            "Current include/config.h geometry is assumed for historical runs. "
            "Horizontal fans 22°/25°, sampled every 0.5°; finite fans do not model optical weighting "
            "or vertical rail visibility. Other walls/obstacles are unidentified. "
            "Freshness limit 50 ms. Logged pose is used without acquisition-time interpolation; "
            "movement within sensor age and unknown mounting bias remain errors.", "",
            "Geometry source SHA-256 (include/config.h): `" + hashlib.sha256(
                (ROOT / "include/config.h").read_bytes()).hexdigest() + "`. "
            "CSV carries original source hashes and completion/overflow/truncation flags; "
            "usable means geometric/measurement eligibility, not a validated successful run.", "",
            "Rear/local-frame and pose-seeding observations are retained but excluded. "
            "No field_start rebase means no field-geometry residual. Cached observations crossing "
            "a logged pose correction are excluded. Selected object switches are only candidates: "
            "the diagnostic tuple does not retain every object or continuous frame history. "
            "Filtered-minus-raw differences show clipping/lag candidates, not proof of delay.", "",
            f"Unique observations: {len(rows)}; usable under both fan models: "
            f"{sum(r['usable'] for r in rows)}. Classification changes between fans: "
            f"{sum(r['classification_22'] != r['classification_25'] for r in rows)}.", "",
            "## Coverage and continuous-angle plots", "",
            "Each plot separates firmware header/procedure, direction, sensor, phase and predicted distance. "
            "Headers can survive firmware changes; consult archived metadata. "
            "Invalid counts cover logged tuples, not all sensor frames. 'none'/'same' carry no new "
            "quality observation. Standard deviations describe pooled residuals, not independent accuracy.", "",
            "| Build / config / procedure / direction / sensor / phase / distance mm | Logged | Usable | Runs | "
            "Usable angle range | Invalid/rejected | Raw median / SD mm | Plot |",
            "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    for key, group in sorted(groups.items()):
        usable = [r for r in group if r["usable"]]
        residuals = [r["raw_residual_mm"] for r in usable]
        angles = [r["incidence_deg"] for r in usable]
        stats = f"{statistics.median(residuals):+.2f} / {statistics.pstdev(residuals):.2f}" if residuals else "unobservable"
        bounds = f"{min(angles):.1f}–{max(angles):.1f}°" if angles else "none"
        digest = hashlib.sha256(repr(key).encode()).hexdigest()[:16]
        name = f"tof-plots/{digest}.svg"
        title = " / ".join(key)
        scatter(group, output / name, title)
        text.append(f"| {title} | {len(group)} | {len(usable)} | "
                    f"{len({(r['source'], r['session']) for r in usable})} | {bounds} | "
                    f"{sum(not r['valid'] or not r['accepted'] for r in group)} | {stats} | [SVG]({name}) |")
    text += ["", "## Exclusions", ""]
    counts = Counter(reason for r in rows for reason in r["reason"].split(";") if reason)
    text += [f"- {reason}: {count}" for reason, count in sorted(counts.items())]
    text += ["", "## Recommendation and logging coverage", "",
             "Retain diagnostic-only ToF use and encoder/gyro integration. These logs do not "
             "establish absolute accuracy or that ToF uncertainty is smaller than pose uncertainty. "
             "No automatic pose/servo correction threshold follows from agreement alone. "
             "Oblique angle coverage must be inspected per group; missing coverage is not sensor success.", "",
             "Firmware diagnostics cover rear positioning, five exit arcs and their drive/brake/settle "
             "states, and edge localization when performed. Official CW finishes before its short scan; "
             "CCW covers the shortened edge localization and finishes before its scan. "
             "Subsequent scans/connectors lack this periodic nominal/estimated route stream; normal "
             "text logs are retained but are not equivalent coverage. Initial pose seeding and final "
             "completion need not coincide with a periodic sample. Nominal pose is command-curvature "
             "integration over measured encoder travel, not an independently timed planned trajectory.", "",
             "Next: receive unchanged normal-run logs for current CW/CCW firmware; record physical "
             "outcomes and compare angle coverage and route discrepancies separately by procedure. "
             "No dedicated ToF sweep/test is requested."]
    (output / "parking_exit_tof_assessment.md").write_text("\n".join(text) + "\n", encoding="utf-8")
    return rows
