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


if __name__ == "__main__":
    unittest.main()
