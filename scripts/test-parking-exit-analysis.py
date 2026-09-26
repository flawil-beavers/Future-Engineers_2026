import importlib.util
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "simulation" / "analyze_parking_exit_pose.py"
SPEC = importlib.util.spec_from_file_location("parking_analysis", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class ParkingExitAnalysisTests(unittest.TestCase):
    def setUp(self):
        self.fixture = ROOT / "simulation" / "fixtures" / "parking_exit_diagnostics" / "sample_log.txt"

    def test_parses_schema_and_braking(self):
        parsed = MODULE.parse_log(self.fixture)
        self.assertEqual(parsed.config["v"], "2")
        self.assertEqual(len(parsed.samples), 7)
        rows = MODULE.segment_rows(parsed)
        localize = next(row for row in rows if row["segment"] == 6)
        self.assertGreaterEqual(localize["brake_travel_mm"], 0.0)
        self.assertFalse(parsed.overflow)
        self.assertEqual(parsed.ordering_errors, 0)

    def test_estimates_known_neutral(self):
        result = MODULE.linear_neutral(MODULE.parse_log(self.fixture).samples)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result["logical_zero"], 0.0, delta=0.75)

    def test_neutral_extrapolation_is_unidentified(self):
        points = [(x, 0.1 + x * 0.0001, "reverse", 1, "increasing")
                  for x in (-2, 0, 2)]
        self.assertIsNone(MODULE.fit_neutral(points))

    def test_real_batch_separates_rear_positioning_and_short_brakes(self):
        path = ROOT / "simulation/evidence/parking_exit_diagnostics/20260926_log_384_cw.txt"
        parsed = MODULE.parse_log(path)
        rows = MODULE.segment_rows(parsed)
        phases = {(row["phase"], row["segment"]): row for row in rows}
        self.assertEqual(set(phases), {("rear_positioning", 0), ("localization", 6),
                                      *(("exit", number) for number in range(1, 6))})
        self.assertAlmostEqual(phases["rear_positioning", 0]["brake_travel_mm"], 0.23, places=2)
        self.assertAlmostEqual(phases["exit", 1]["brake_travel_mm"], -0.93, places=2)
        self.assertAlmostEqual(phases["exit", 2]["brake_travel_mm"], 0.11, places=2)
        self.assertAlmostEqual(phases["localization", 6]["brake_travel_mm"], 0.23, places=2)
        groups = dict(MODULE.grouped_neutral(parsed.samples))
        self.assertNotIn("counterclockwise-exit", groups)

    def test_report_with_real_batch(self):
        path = ROOT / "simulation/evidence/parking_exit_diagnostics/20260926_log_384_cw.txt"
        parsed, rows = MODULE.analyze([path])
        output = ROOT / "local_workspace/parking-analysis-regression"
        MODULE.write_report(parsed, rows, output)
        self.assertIn("rear_positioning", (output / "parking_exit_repeatability.csv").read_text())

    def test_rejects_missing_config(self):
        scratch = ROOT / "local_workspace" / "parking-analysis-test-missing.txt"
        scratch.parent.mkdir(exist_ok=True)
        scratch.write_text("[PARK_DIAG] v=2 type=sample\n", encoding="utf-8")
        try:
            with self.assertRaises(ValueError):
                MODULE.parse_log(scratch)
        finally:
            scratch.unlink(missing_ok=True)

    def test_flags_abort_overflow_truncation_and_duplicate_tof(self):
        source = self.fixture.read_text(encoding="utf-8").splitlines()
        source[2] = source[2].replace("s0=2,", "s0=1,")
        source = [line.replace("unparking_complete", "heading_abort")
                  for line in source]
        source.extend([
            "[PARK_DIAG] v=2 type=truncated t=1500 reason=byte_budget",
            "[LOGGER] LOG BUFFER OVERFLOW",
        ])
        scratch = ROOT / "local_workspace" / "parking-analysis-test-flags.txt"
        scratch.parent.mkdir(exist_ok=True)
        scratch.write_text("\n".join(source), encoding="utf-8")
        try:
            parsed = MODULE.parse_log(scratch)
            self.assertTrue(parsed.overflow)
            self.assertTrue(parsed.truncated)
            self.assertGreater(parsed.duplicate_tof_sequences, 0)
            self.assertTrue(any("heading_abort" in event.get("detail", "")
                                for event in parsed.events))
        finally:
            scratch.unlink(missing_ok=True)

    def test_reversal_without_resolved_geometry_is_unobservable(self):
        parsed = MODULE.parse_log(self.fixture)
        rows = MODULE.reversal_rows(parsed)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["effective_lost_motion_mm"], "unobservable")

    def test_invalid_new_tof_does_not_reuse_older_valid_reference(self):
        samples = MODULE.parse_log(self.fixture).samples
        samples[1].raw["s2"] = "2,1,63,63,1,1,0,0"
        samples[2].raw["s2"] = "same"
        self.assertIsNone(MODULE.resolved_tof(samples, 1, "s2"))
        self.assertIsNone(MODULE.resolved_tof(samples, 2, "s2"))
        samples[1].raw["s2"] = "same"
        self.assertIsNotNone(MODULE.resolved_tof(samples, 1, "s2"))

    def test_rejects_silently_cut_sensor_record(self):
        source = self.fixture.read_text(encoding="utf-8")
        scratch = ROOT / "local_workspace" / "parking-analysis-test-cut.txt"
        scratch.parent.mkdir(exist_ok=True)
        for replacement in ("", "s2=1,1,65"):
            with self.subTest(replacement=replacement):
                scratch.write_text(source.replace("s2=1,1,65,65,1,1,1,1", replacement),
                                   encoding="utf-8")
                try:
                    with self.assertRaises(ValueError):
                        MODULE.parse_log(scratch)
                finally:
                    scratch.unlink(missing_ok=True)

    def test_rejects_exceeded_declared_budgets(self):
        source = self.fixture.read_text(encoding="utf-8")
        scratch = ROOT / "local_workspace" / "parking-analysis-test-budget.txt"
        scratch.parent.mkdir(exist_ok=True)
        for original, replacement in (("sample_limit=150", "sample_limit=6"),
                                      ("byte_limit=65536", "byte_limit=100")):
            with self.subTest(replacement=replacement):
                scratch.write_text(source.replace(original, replacement), encoding="utf-8")
                try:
                    with self.assertRaises(ValueError):
                        MODULE.parse_log(scratch)
                finally:
                    scratch.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
