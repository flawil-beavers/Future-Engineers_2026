"""Checks for the offline first-corner arc search, not robot acceptance."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'simulation'))
from corner_forward_view_search import ALL_SEATS, advance, margins, visible, approach_poses, heading_stop


class ForwardViewTests(unittest.TestCase):
    def test_legal_seats_complete_and_unique(self):
        self.assertEqual(len(set(ALL_SEATS)), 24)
        self.assertIn((-1100, 0), ALL_SEATS)
        self.assertIn((500, 900), ALL_SEATS)

    def test_bicycle_circle_and_straight(self):
        self.assertEqual(advance((0, 0, 0), 0, 100), (100, 0, 0))
        x, y, h = advance((0, 0, 0), 45, math.pi * 50)
        self.assertAlmostEqual(x, 100)
        self.assertAlmostEqual(y, -100)
        self.assertAlmostEqual(h, -90)

    def test_non_target_obstacle_and_rear_are_checked(self):
        self.assertLess(margins((500, 900, 0))[1], 0)
        self.assertLess(margins((610, 900, 0))[1], 0)
        self.assertLess(margins((1490, 750, 90))[0], 0)

    def test_range_and_bearing_remain_required(self):
        self.assertTrue(visible((0, 0, 0), (400, 0)))
        self.assertFalse(visible((0, 0, 0), (300, 0)))
        self.assertFalse(visible((0, 0, 0), (400, 300)))

    def test_real_approach_only(self):
        root = Path(__file__).resolve().parents[1]
        path = root / 'simulation/evidence/parking_exit_diagnostics/20260926_log_393_cw.txt'
        rows = list(approach_poses(path))
        self.assertEqual(len(rows), 11)
        self.assertEqual(rows[0][0], 35705)
        self.assertLess(rows[-1][0], 38750)

    def test_gyro_stop_adjusts_travel_and_keeps_heading_during_coast(self):
        p = (-746, -907.9, 147.9)
        slow, a = heading_stop(p, 40, 120, .85, 20)
        fast, b = heading_stop(p, 40, 120, 1.15, 0)
        self.assertAlmostEqual(slow[2], 120)
        self.assertAlmostEqual(fast[2], 120)
        self.assertGreater(a, b)


if __name__ == '__main__':
    unittest.main()
