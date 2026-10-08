#!/usr/bin/env python3
"""Regenerate parking-exit analysis from archived complete robot logs."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
from pathlib import Path

import analyze_parking_exit_pose as analyzer
import analyze_parking_exit_tof as tof_analyzer

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCES = ROOT / "simulation/evidence/parking_exit_diagnostics"
DEFAULT_OUTPUT = ROOT / "local_workspace/parking-exit-analysis-all"
COMPLETE_NAME = re.compile(r"\d{8}_log_\d{3,}_(?:cw|ccw)\.txt")
MANIFEST_FIELDS = (
    "source", "sha256", "session", "source_lines", "build", "direction",
    "exit_complete", "overflow", "diagnostic_truncated", "pose_svg",
    "exit_pose_svg", "build_report", "observed_procedure",
)


def observed_procedure(session: analyzer.ParsedLog) -> str:
    lines = session.path.read_text(encoding="utf-8", errors="replace").splitlines()
    text = "\n".join(lines[session.source_line_start - 1:session.source_line_end])
    if "[CCW START] First-edge reference accepted;" in text:
        return "ccw_first_edge_start"
    if "[CW START] Short scan from initial ToF seed + exit odometry" in text:
        return "cw_short_start"
    if "[CCW START] Second-edge reference reached; no extra reverse" in text:
        return "ccw_short_start"
    return "legacy_or_unidentified"


def config_signature(session: analyzer.ParsedLog) -> tuple[tuple[str, str], ...]:
    items = list(session.config.items())
    procedure = observed_procedure(session)
    if procedure != "legacy_or_unidentified":
        items.append(("observed_procedure", procedure))
    return tuple(sorted(items))


def discover_sources(directory: Path) -> tuple[list[Path], list[tuple[str, str]]]:
    """Select complete originals once each; never analyze excerpt filenames."""
    if not directory.is_dir():
        raise ValueError(f"evidence directory does not exist: {directory}")
    selected: list[Path] = []
    duplicates: list[tuple[str, str]] = []
    seen: dict[str, str] = {}
    for path in sorted(directory.iterdir()):
        if not path.is_file() or not COMPLETE_NAME.fullmatch(path.name):
            continue
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest in seen:
            duplicates.append((path.name, seen[digest]))
            continue
        seen[digest] = path.name
        selected.append(path)
    if not selected:
        raise ValueError(f"no complete parking-exit logs found in {directory}")
    return selected, duplicates


def generate(source_dir: Path, output_dir: Path,
             whole_run_images: bool = False) -> tuple[int, int, int]:
    source_dir = source_dir.resolve()
    output_dir = output_dir.resolve()
    if output_dir == source_dir or source_dir in output_dir.parents:
        raise ValueError("output directory must be outside the evidence directory")
    paths, duplicates = discover_sources(source_dir)
    sessions, rows = analyzer.analyze(paths)
    analyzer.write_report(sessions, rows, output_dir)
    if whole_run_images:
        from visualize_robot_run import render
        for session in sessions:
            render(session, output_dir)
    tof_analyzer.write_assessment(sessions, output_dir, observed_procedure)
    groups: dict[tuple[tuple[str, str], ...], list[analyzer.ParsedLog]] = {}
    for session in sessions:
        groups.setdefault(config_signature(session), []).append(session)
    build_reports = {}
    for signature, group in sorted(groups.items()):
        build = group[0].config.get("build", "unknown")
        safe_build = re.sub(r"[^A-Za-z0-9_-]", "_", build)
        digest = hashlib.sha256(json.dumps(signature).encode("utf-8")).hexdigest()[:12]
        directory = Path("by-build") / f"{safe_build}_{digest}"
        labels = {session.label for session in group}
        analyzer.write_report(group, [row for row in rows if row["file"] in labels],
                              output_dir / directory)
        tof_analyzer.write_assessment(group, output_dir / directory, observed_procedure)
        build_reports[signature] = directory / "parking_exit_analysis.md"
    with (output_dir / "parking_exit_sources.csv").open(
            "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=MANIFEST_FIELDS)
        writer.writeheader()
        for session in sessions:
            safe_name = "".join(character if character.isalnum() else "_"
                                for character in session.label)
            has_exit = any(sample.state.startswith("segment_")
                           for sample in session.samples)
            writer.writerow({
                "source": session.path.name,
                "sha256": session.source_sha256,
                "session": session.session_index,
                "source_lines": f"{session.source_line_start}-{session.source_line_end}",
                "build": session.config.get("build", ""),
                "direction": session.path.stem.rsplit("_", 1)[-1],
                "exit_complete": session.exit_complete,
                "overflow": session.overflow,
                "diagnostic_truncated": session.truncated,
                "pose_svg": f"{safe_name}_pose.svg",
                "exit_pose_svg": f"{safe_name}_exit_pose.svg" if has_exit else "",
                "build_report": build_reports[config_signature(session)].as_posix(),
                "observed_procedure": observed_procedure(session),
            })
    with (output_dir / "parking_exit_batch.md").open("w", encoding="utf-8") as handle:
        handle.write("# Parking-exit batch analysis\n\n")
        handle.write(f"Analyzed {len(paths)} complete source files containing "
                     f"{len(sessions)} sessions.\n\n")
        handle.write("See [the analysis](parking_exit_analysis.md) and "
                     "[source manifest](parking_exit_sources.csv). "
                     "Plots are onboard rear-axle pose estimates, not measured "
                     "clearance or a proposed robot path. Check physical outcomes "
                     "in the evidence README.\n")
        if whole_run_images:
            handle.write("\n## Whole-run images with recorded pillars\n\n")
            for session in sessions:
                safe_name = ''.join(c if c.isalnum() else '_' for c in session.label)
                handle.write(f"- `{session.label}`: [SVG]({safe_name}_whole_run.svg) "
                             f"/ [PNG]({safe_name}_whole_run.png)\n")
        handle.write("\nAdded [offline ToF assessment](parking_exit_tof_assessment.md) "
                     "and [unique observations](parking_exit_tof_observations.csv). "
                     "Existing route/braking/reversal outputs remain available. "
                     "Sensing fans are laptop geometry only, not a robot test.\n")
        handle.write("\n## Results by diagnostic build, configuration and observed procedure\n\n"
                     "Use these reports to compare revisions. The top-level report "
                     "pools historical configurations. A diagnostic object's build "
                     "timestamp may remain unchanged after other firmware changes; "
                     "verify commanded segment targets and the evidence README "
                     "before claiming matching motion or installed firmware.\n\n"
                     "Explicit CW/CCW short-start log markers form separate groups. "
                     "Without those markers, the procedure is legacy or unidentified.\n\n"
                     "| Diagnostic build | Observed procedure | Sessions | Completed, untruncated | Report |\n"
                     "| --- | --- | --- | --- | --- |\n")
        for signature, group in sorted(groups.items()):
            completed = sum(session.exit_complete and not session.overflow and
                            not session.truncated for session in group)
            report = build_reports[signature].as_posix()
            handle.write(f"| `{group[0].config.get('build', 'unknown')}` | "
                         f"{observed_procedure(group[0])} | "
                         f"{len(group)} | {completed} | [Analysis]({report}) |\n")
        if duplicates:
            handle.write("\nIdentical complete source files skipped:\n\n")
            for duplicate, original in duplicates:
                handle.write(f"- `{duplicate}` duplicates `{original}`.\n")
    return len(paths), len(sessions), len(duplicates)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, default=DEFAULT_SOURCES)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--whole-run-images", action="store_true",
                        help="Also create SVG/PNG full-run views with mapped pillars (requires matplotlib)")
    args = parser.parse_args()
    try:
        files, sessions, duplicates = generate(args.source_dir, args.output_dir,
                                               args.whole_run_images)
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, f"Parking-exit batch analysis failed: {error}\n")
    print(f"Analyzed {files} complete files ({sessions} sessions); "
          f"skipped {duplicates} identical files.")
    print(f"Report: {args.output_dir / 'parking_exit_batch.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
