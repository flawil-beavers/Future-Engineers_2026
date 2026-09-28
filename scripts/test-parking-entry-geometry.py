"""Regression checks for the scout model's geometry primitives."""
import importlib.util
import math
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "scout_geometry", ROOT / "simulation" / "parking_entry_scout_sim.py")
MODEL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODEL)


class ScoutGeometryTests(unittest.TestCase):
    def test_disjoint_collinear_segments_keep_gap(self):
        self.assertAlmostEqual(MODEL.segment_distance((0, 0, 1, 0), (3, 0, 4, 0)), 2)

    def test_crossing_and_touching_segments(self):
        for segment in ((1, -1, 1, 1), (2, 0, 3, 0), (1, 0, 3, 0)):
            self.assertEqual(MODEL.segment_distance((0, 0, 2, 0), segment), 0)

    def test_parallel_separation(self):
        self.assertAlmostEqual(MODEL.segment_distance((0, 0, 2, 0), (0, 3, 2, 3)), 3)

    def test_quarter_circle_and_mirroring(self):
        travel = 100 * math.pi / 2
        cw = MODEL.scout_pose(0, 0, 0, "CW", travel, 100)
        ccw = MODEL.scout_pose(0, 0, 0, "CCW", travel, 100)
        for actual, expected in zip(cw, (-100, 100, -90)):
            self.assertAlmostEqual(actual, expected, places=7)
        for actual, expected in zip(ccw, (-100, -100, 90)):
            self.assertAlmostEqual(actual, expected, places=7)


if __name__ == "__main__":
    unittest.main()
