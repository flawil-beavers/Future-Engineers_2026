"""Additional corner arc geometry checks, independent of robot acceptance."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'simulation'))
from corner_extra_view_check import check
from corner_forward_view_search import advance, margins
from parking_entry_scout_sim import wrap180


class ExtraViewTests(unittest.TestCase):
    def test_signed_arcs_retrace_both_steering_directions(self):
        start = (-773.5, -946.3, 113.53)
        for steering in (-20, 20):
            for gain in (.85, 1, 1.15):
                end = advance(advance(start, steering, -130, gain), steering, 130, gain)
                self.assertAlmostEqual(end[0], start[0])
                self.assertAlmostEqual(end[1], start[1])
                self.assertAlmostEqual(wrap180(end[2]-start[2]), 0)

    def test_model_detects_pillar_and_wall_collision(self):
        self.assertLess(margins((-900, -500, 90))[1], 0)
        self.assertLess(margins((-1490, 0, 90))[0], 0)

    def test_recorded_scans_with_perturbations(self):
        root = Path(__file__).resolve().parents[1]
        result = check(root / 'simulation/evidence/parking_exit_diagnostics/20260927_log_398_cw.txt')
        self.assertEqual(result['scan_poses'], 2)
        self.assertEqual(result['cases'], 486)
        self.assertEqual(result['view_failures'], 0)
        self.assertEqual(result['clearance_failures'], 0)
        self.assertGreater(result['minimum_ray_separation_deg'], 3.25)


if __name__ == '__main__':
    unittest.main()
