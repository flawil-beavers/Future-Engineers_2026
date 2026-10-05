import csv
import hashlib
import pathlib
import shutil
import subprocess
import sys
import tempfile
import unittest

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

    def test_rejects_output_inside_evidence(self):
        result = self.run_batch(self.sources / "plots")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("outside the evidence directory", result.stderr)


if __name__ == "__main__":
    unittest.main()
