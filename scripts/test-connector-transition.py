"""Regression tests for terminal alignment and conservative kinematic rollout."""
import math
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'simulation'))
import connector_transition_sim as MODEL


class ConnectorTransitionTests(unittest.TestCase):
    def setUp(self):
        self.path = ROOT / 'simulation/evidence/parking_exit_diagnostics/20260926_log_386_cw.txt'
        self.connector, self.route = MODEL.geometry(self.path)

    def test_end_heading_follows_displaced_route(self):
        corrected = MODEL.corrected(self.connector, self.route)
        self.assertAlmostEqual(corrected[-1]['h'], 132.56568, places=4)
        self.assertGreater(abs(MODEL.wrap(corrected[-1]['h'] - self.connector[-1]['h'])), 47)
        for key in ('x', 'y'):
            self.assertAlmostEqual(corrected[-1][key], self.route[0][key])

    def test_two_real_start_geometries_converge_with_original_limits(self):
        for number in (386, 387):
            path = self.path.with_name(f'20260926_log_{number}_cw.txt')
            connector, route = MODEL.geometry(path)
            self.assertNotEqual(MODEL.simulate(connector, route)['status'], 'pass')
            result = MODEL.simulate(MODEL.corrected(connector, route), route)
            self.assertEqual(MODEL.preflight_lookahead(MODEL.corrected(connector, route), route), 150)
            self.assertEqual(result['status'], 'pass')
            self.assertLessEqual(result['distance_mm'], 60)
            self.assertLessEqual(result['heading_deg'], 15)
            self.assertLessEqual(result['travel_mm'], 500)

    def test_synthetic_mirror_converges(self):
        corrected = MODEL.corrected(self.connector, self.route)
        result = MODEL.simulate(MODEL.reflect(corrected), MODEL.reflect(self.route), mirror=True)
        self.assertEqual(result['status'], 'pass')

    def test_wall_collision_rejects_before_handoff(self):
        points = [dict(x=0, y=-1490, h=0), dict(x=25, y=-1490, h=0), dict(x=50, y=-1490, h=0)]
        self.assertEqual(MODEL.simulate(points, points)['status'], 'clearance')

    def test_incomplete_geometry_is_rejected(self):
        with self.assertRaises(ValueError):
            MODEL.corrected(self.connector, self.route[:1])


if __name__ == '__main__':
    unittest.main()
