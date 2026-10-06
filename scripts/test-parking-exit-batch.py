import csv
import hashlib
import pathlib
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "simulation"))
import analyze_parking_exit_pose as analyzer

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "simulation/analyze_parking_exit_batch.py"
EVIDENCE = ROOT / "simulation/evidence/parking_exit_diagnostics"
SCRATCH = ROOT / "local_workspace"


class ParkingExitBatchTests(unittest.TestCase):
    def setUp(self):
        SCRATCH.mkdir(exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(dir=SCRATCH)
        self.addCleanup(self.temporary.cleanup)
        self.root = pathlib.Path(self.temporary.name)
        self.sources = self.root / "sources"
        self.sources.mkdir()
        self.output = self.root / "output"

    def run_batch(self, output=None):
        return subprocess.run(
            [sys.executable, str(SCRIPT), "--source-dir", str(self.sources),
             "--output-dir", str(output or self.output)],
            cwd=ROOT, text=True, capture_output=True, check=False)

    def test_generates_manifest_and_plots_without_excerpt_or_duplicate(self):
        original = EVIDENCE / "20260926_log_384_cw.txt"
        selected = self.sources / original.name
        shutil.copyfile(original, selected)
        shutil.copyfile(original, self.sources / "20260927_log_999_cw.txt")
        (self.sources / "20260927_log_000_cw_excerpt.txt").write_text(
            "incomplete", encoding="utf-8")
        result = self.run_batch()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("skipped 1 identical files", result.stdout)
        with (self.output / "parking_exit_sources.csv").open(
                encoding="utf-8", newline="") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["source"], original.name)
        self.assertEqual(rows[0]["sha256"],
                         hashlib.sha256(original.read_bytes()).hexdigest())
        self.assertEqual(rows[0]["exit_complete"], "True")
        self.assertTrue((self.output / rows[0]["pose_svg"]).exists())
        self.assertTrue((self.output / rows[0]["exit_pose_svg"]).exists())
        self.assertIn("duplicates", (self.output / "parking_exit_batch.md").read_text())

    def test_multi_session_original_stays_one_source(self):
        original = EVIDENCE / "20260927_log_398_cw.txt"
        shutil.copyfile(original, self.sources / original.name)
        result = self.run_batch()
        self.assertEqual(result.returncode, 0, result.stderr)
        with (self.output / "parking_exit_sources.csv").open(
                encoding="utf-8", newline="") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual([row["session"] for row in rows], ["1", "2"])
        self.assertEqual({row["source"] for row in rows}, {original.name})
        self.assertNotEqual(rows[0]["pose_svg"], rows[1]["pose_svg"])

    def test_tof_addition_preserves_all_existing_route_outputs(self):
        original = EVIDENCE / "20261005_log_446_cw.txt"
        selected = self.sources / original.name
        shutil.copyfile(original, selected)
        baseline = self.root / "baseline"
        sessions, rows = analyzer.analyze([selected])
        analyzer.write_report(sessions, rows, baseline)
        self.assertEqual(self.run_batch().returncode, 0)
        for path in baseline.iterdir():
            self.assertEqual(path.read_bytes(), (self.output / path.name).read_bytes(), path.name)
        self.assertEqual(original.read_bytes(), selected.read_bytes())
        self.assertTrue((self.output / "parking_exit_tof_assessment.md").exists())
        self.assertTrue((self.output / "parking_exit_tof_observations.csv").exists())

    def test_rejects_output_inside_evidence(self):
        result = self.run_batch(self.sources / "plots")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("outside the evidence directory", result.stderr)

    def test_reports_changed_exit_revision_separately(self):
        for name in ("20260926_log_384_cw.txt", "20261005_log_446_cw.txt"):
            shutil.copyfile(EVIDENCE / name, self.sources / name)
        result = self.run_batch()
        self.assertEqual(result.returncode, 0, result.stderr)
        with (self.output / "parking_exit_sources.csv").open(
                encoding="utf-8", newline="") as handle:
            rows = list(csv.DictReader(handle))
        reports = [self.output / row["build_report"] for row in rows]
        self.assertNotEqual(reports[0], reports[1])
        for index, report in enumerate(reports):
            text = report.read_text(encoding="utf-8")
            self.assertIn(rows[index]["source"], text)
            self.assertNotIn(rows[1 - index]["source"], text)
            self.assertIn("1 completed, untruncated exit sessions of 1", text)

    def test_same_build_with_changed_configuration_is_separate(self):
        original = EVIDENCE / "20260926_log_384_cw.txt"
        shutil.copyfile(original, self.sources / original.name)
        changed = original.read_text(encoding="utf-8").replace("center=80", "center=81")
        (self.sources / "20260926_log_999_cw.txt").write_text(changed, encoding="utf-8")
        result = self.run_batch()
        self.assertEqual(result.returncode, 0, result.stderr)
        with (self.output / "parking_exit_sources.csv").open(
                encoding="utf-8", newline="") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(rows[0]["build"], rows[1]["build"])
        self.assertNotEqual(rows[0]["build_report"], rows[1]["build_report"])

    def test_short_start_markers_separate_unchanged_diagnostic_headers(self):
        original = EVIDENCE / "20260926_log_384_cw.txt"
        shutil.copyfile(original, self.sources / original.name)
        text = original.read_text(encoding="utf-8")
        for number, marker in (
                (998, "[CW START] Short scan from initial ToF seed + exit odometry; no second-edge reverse"),
                (999, "[CCW START] Second-edge reference reached; no extra reverse or behind-place scout")):
            (self.sources / f"20260926_log_{number}_cw.txt").write_text(
                text + "\n" + marker + "\n", encoding="utf-8")
        result = self.run_batch()
        self.assertEqual(result.returncode, 0, result.stderr)
        with (self.output / "parking_exit_sources.csv").open(
                encoding="utf-8", newline="") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(len({row["build"] for row in rows}), 1)
        self.assertEqual(len({row["build_report"] for row in rows}), 3)
        self.assertEqual({row["observed_procedure"] for row in rows},
                         {"legacy_or_unidentified", "cw_short_start", "ccw_short_start"})


if __name__ == "__main__":
    unittest.main()
