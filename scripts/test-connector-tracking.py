import importlib.util
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('connector_tracking', ROOT / 'simulation/analyze_connector_tracking.py')
MODEL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODEL)


class ConnectorTrackingTests(unittest.TestCase):
    def setUp(self):
        self.connector = [{'x': x, 'y': 0, 'h': 0} for x in (0, 25, 50)]
        self.route = [{'x': x, 'y': 0, 'h': 0} for x in (50, 75, 100, 125, 150, 175, 200)]

    def test_endpoint_clipping_and_continuation(self):
        finite = MODEL.lookahead(self.connector, self.route, 1, 150)
        continued = MODEL.lookahead(self.connector, self.route, 1, 150, True)
        self.assertEqual(finite['x'], 50)
        self.assertEqual(continued['x'], 175)
        # A near endpoint with a 5 mm lateral error demands too much steering;
        # extending the straight route reduces that demand at this pose.
        pose = {'x': 20, 'y': -5, 'h': 0}
        self.assertGreater(abs(MODEL.steering(pose, finite, 100)[2]), 42)
        self.assertLess(abs(MODEL.steering(pose, continued, 100)[2]), 42)

    def test_insufficient_route_cannot_silently_clip(self):
        with self.assertRaises(ValueError):
            MODEL.lookahead(self.connector, self.route[:2], 1, 150, True)

    def test_mirrored_steering_and_forward_guard(self):
        left = MODEL.steering({'x': 0, 'y': 0, 'h': 0}, {'x': 100, 'y': 20}, 100)
        right = MODEL.steering({'x': 0, 'y': 0, 'h': 0}, {'x': 100, 'y': -20}, 100)
        self.assertAlmostEqual(left[2], -right[2])
        self.assertLess(MODEL.steering({'x': 0, 'y': 0, 'h': 0}, {'x': -100, 'y': 0}, 100)[0], 1)

    def test_old_evidence_is_not_presented_as_replay(self):
        path = ROOT / 'simulation/evidence/parking_exit_diagnostics/20260926_log_384_cw.txt'
        self.assertEqual(MODEL.analyze(path)['status'], 'missing_connector_geometry_and_tail_pose')

    def test_real_cw_replay_and_ccw_preflight_rejection(self):
        directory = ROOT / 'simulation/evidence/parking_exit_diagnostics'
        report = MODEL.analyze(directory / '20260926_log_386_cw.txt')
        self.assertEqual(len(report['samples']), 8)
        last = report['samples'][-1]
        self.assertTrue(last['recorded_rejection'])
        self.assertFalse(last['handoff_gate_pass'])
        # Feasible candidate steering does not establish heading convergence.
        self.assertLess(last['finite_steering_deg'], -42)
        self.assertGreater(last['continuation_steering_deg'], 0)
        self.assertEqual(MODEL.analyze(directory / '20260926_log_388_ccw.txt')['status'],
                         'preflight_rejected_no_tracking')

    def test_complete_trace_and_corrupt_target(self):
        lines = []
        for kind, points in [('connector', self.connector), ('route', self.route)]:
            for index, point in enumerate(points):
                lines.append(f'[CONNECTOR_POINT] kind={kind} index={index} x={point["x"]} y=0 h=0')
        command = MODEL.steering({'x': 20, 'y': -5, 'h': 0}, {'x': 50, 'y': 0}, 100)[2]
        lines.append(f'[CONNECTOR_TRACK] t=100 progress=1 x=20 y=-5 h=0 tx=50 ty=0 '
                     f'forward=30 lateral=5 steering={command} end_distance=30.414 '
                     'end_heading=20 lookahead=150 wheelbase=100 limit=42 '
                     'gate_distance=60 gate_heading=15 rejected=1')
        scratch = ROOT / 'local_workspace/connector-tracking-test.txt'
        scratch.parent.mkdir(exist_ok=True)
        try:
            scratch.write_text('\n'.join(lines))
            row = MODEL.analyze(scratch)['samples'][0]
            self.assertTrue(row['recorded_rejection'])
            self.assertFalse(row['handoff_gate_pass'])
            self.assertTrue(row['continuation_tracking_guard_pass'])
            scratch.write_text('\n'.join(lines).replace('tx=50', 'tx=70'))
            with self.assertRaises(ValueError):
                MODEL.analyze(scratch)
        finally:
            scratch.unlink(missing_ok=True)


if __name__ == '__main__':
    unittest.main()
