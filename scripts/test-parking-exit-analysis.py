import csv
import importlib.util
import math
import pathlib
import sys
import unittest
import xml.etree.ElementTree as ET

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
        repeatability = (output / "parking_exit_repeatability.csv").read_text()
        self.assertIn("rear_positioning", repeatability)
        self.assertIn("direction", repeatability.splitlines()[0])
        self.assertTrue((output / "parking_exit_rear_motion.csv").exists())
        self.assertTrue((output / "parking_exit_corrections.csv").exists())

    def test_pose_svg_keeps_one_field_frame_and_equal_axis_scale(self):
        path = ROOT / "simulation/evidence/parking_exit_diagnostics/20260926_log_384_cw.txt"
        parsed, rows = MODULE.analyze([path])
        output = ROOT / "local_workspace/parking-analysis-pose-svg"
        MODULE.write_report(parsed, rows, output)
        full = ET.parse(output / "20260926_log_384_cw_txt_pose.svg").getroot()
        exit_plot = ET.parse(output / "20260926_log_384_cw_txt_exit_pose.svg").getroot()
        namespace = {"svg": "http://www.w3.org/2000/svg"}
        self.assertEqual(full.attrib["data-frame"], "field")
        self.assertEqual(exit_plot.attrib["data-frame"], "field")
        correction = next(node for node in full.findall("svg:line", namespace)
                          if node.attrib.get("data-series") == "correction")
        end = next(node for node in full.findall("svg:circle", namespace)
                   if node.attrib.get("data-role") == "end")
        self.assertEqual((end.attrib["cx"], end.attrib["cy"]),
                         (correction.attrib["x2"], correction.attrib["y2"]))
        exit_lines = [node for node in exit_plot.findall("svg:polyline", namespace)
                      if node.attrib.get("data-series") == "estimated"]
        self.assertEqual({node.attrib["data-phase"] for node in exit_lines}, {"exit"})
        plotted = [tuple(map(float, pair.split(","))) for node in exit_lines
                   for pair in node.attrib["points"].split()]
        field = [sample for sample in parsed[0].samples
                 if sample.state.startswith("segment_")]
        self.assertEqual(len(plotted), len(field))
        self.assertLess(len(plotted), len(parsed[0].samples))
        scale = float(exit_plot.attrib["data-scale-px-per-mm"])
        for index in range(1, len(field)):
            distance_mm = math.dist(field[index - 1].pose[:2], field[index].pose[:2])
            if distance_mm > 5.0:
                distance_px = math.dist(plotted[index - 1], plotted[index])
                self.assertAlmostEqual(distance_px / distance_mm, scale, delta=0.03)
                break
        else:
            self.fail("real exit lacks a moving pose pair")

    def test_rear_marker_motion_uses_settled_micro_correction(self):
        path = ROOT / "simulation/evidence/parking_exit_diagnostics/20260926_log_390_cw.txt"
        rows = MODULE.rear_motion_rows(MODULE.parse_log(path))
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[1]["direction"], "reverse")
        self.assertAlmostEqual(rows[1]["encoder_motion_mm"], -7.85, places=2)
        self.assertAlmostEqual(rows[1]["tof_motion_mm"], -6.0, places=2)
        self.assertLess(abs(rows[1]["encoder_minus_tof_mm"]), rows[1]["uncertainty_mm"])

    def test_multi_session_source_is_expanded_without_editing_evidence(self):
        path = ROOT / "simulation/evidence/parking_exit_diagnostics/20260927_log_398_cw.txt"
        sessions, rows = MODULE.analyze([path])
        self.assertEqual([len(session.samples) for session in sessions], [74, 77])
        self.assertEqual(sessions[0].source_line_start, 10)
        self.assertEqual(sessions[1].source_line_start, 2218)
        self.assertEqual(sessions[0].source_sha256, sessions[1].source_sha256)
        output = ROOT / "local_workspace/parking-analysis-multi-session"
        MODULE.write_report(sessions, rows, output)
        report = (output / "parking_exit_analysis.md").read_text(encoding="utf-8")
        self.assertIn("#session-1", report)
        self.assertIn("Source lines: 10-2217", report)
        self.assertIn("#session-2", report)
        self.assertIn("Source lines: 2218-", report)

    def test_combined_summary_excludes_aborted_rear_positioning(self):
        evidence = ROOT / "simulation/evidence/parking_exit_diagnostics"
        complete = evidence / "20260928_log_400_cw.txt"
        aborted = evidence / "20260929_log_414_cw.txt"
        sessions, rows = MODULE.analyze([complete, aborted])
        output = ROOT / "local_workspace/parking-analysis-completion"
        MODULE.write_report(sessions, rows, output)
        report = (output / "parking_exit_analysis.md").read_text(encoding="utf-8")
        self.assertIn("1 completed, untruncated exit sessions of 2", report)
        self.assertIn("20260929_log_414_cw.txt", report)
        with (output / "parking_exit_rear_motion.csv").open(encoding="utf-8") as handle:
            rear_rows = list(csv.DictReader(handle))
        self.assertEqual(len(rear_rows), 2)

    def test_enforces_new_firmware_sample_sub_budget(self):
        source = self.fixture.read_text(encoding="utf-8")
        source = source.replace("byte_limit=65536", "byte_limit=65536 sample_bytes=10")
        scratch = ROOT / "local_workspace/parking-analysis-test-budget.txt"
        scratch.parent.mkdir(exist_ok=True)
        scratch.write_text(source, encoding="utf-8")
        try:
            with self.assertRaisesRegex(ValueError, "sample bytes exceed"):
                MODULE.parse_log(scratch)
        finally:
            scratch.unlink(missing_ok=True)

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
