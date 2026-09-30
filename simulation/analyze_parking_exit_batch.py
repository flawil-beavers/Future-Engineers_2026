#!/usr/bin/env python3
"""Regenerate parking-exit analysis from archived complete robot logs."""

from __future__ import annotations

import argparse
import csv
import hashlib
import re
from pathlib import Path

import analyze_parking_exit_pose as analyzer

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCES = ROOT / "simulation/evidence/parking_exit_diagnostics"
DEFAULT_OUTPUT = ROOT / "local_workspace/parking-exit-analysis-all"
COMPLETE_NAME = re.compile(r"\d{8}_log_\d{3,}_(?:cw|ccw)\.txt")
MANIFEST_FIELDS = (
    "source", "sha256", "session", "source_lines", "build", "direction",
    "exit_complete", "overflow", "diagnostic_truncated", "pose_svg",
    "exit_pose_svg",
)


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


def generate(source_dir: Path, output_dir: Path) -> tuple[int, int, int]:
    source_dir = source_dir.resolve()
    output_dir = output_dir.resolve()
    if output_dir == source_dir or source_dir in output_dir.parents:
        raise ValueError("output directory must be outside the evidence directory")
    paths, duplicates = discover_sources(source_dir)
    sessions, rows = analyzer.analyze(paths)
    analyzer.write_report(sessions, rows, output_dir)
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
        if duplicates:
            handle.write("\nIdentical complete source files skipped:\n\n")
            for duplicate, original in duplicates:
                handle.write(f"- `{duplicate}` duplicates `{original}`.\n")
    return len(paths), len(sessions), len(duplicates)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, default=DEFAULT_SOURCES)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    try:
        files, sessions, duplicates = generate(args.source_dir, args.output_dir)
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, f"Parking-exit batch analysis failed: {error}\n")
    print(f"Analyzed {files} complete files ({sessions} sessions); "
          f"skipped {duplicates} identical files.")
    print(f"Report: {args.output_dir / 'parking_exit_batch.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
