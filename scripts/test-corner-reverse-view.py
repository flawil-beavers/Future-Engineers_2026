"""Offline reverse peek geometry checks; no physical robot acceptance."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'simulation'))
from corner_reverse_view_search import evaluate, reverse_pose, hold_poses

class ReverseViewTests(unittest.TestCase):
    def test_signed_retrace(self):
        pose = (-847, -807.8, 125.67)
        result = reverse_pose(reverse_pose(pose, 190), -190)
        for a, b in zip(pose, result):
            self.assertAlmostEqual(a, b)

    def test_other_legal_seat_is_checked(self):
        result = evaluate((500, 900, 0), 170, (0, 0, 0))
        self.assertFalse(result['passed'])
        self.assertLess(result['pillar_mm'], 0)

    def test_real_nominal_holds_pass_all_seat_sweep(self):
        root = Path(__file__).resolve().parents[1]
        for log in (392, 393):
            path = root / f'simulation/evidence/parking_exit_diagnostics/20260926_log_{log}_cw.txt'
            for pose in hold_poses(path):
                self.assertTrue(evaluate(pose, 170, (0, 0, 0))['passed'])

if __name__ == '__main__':
    unittest.main()
