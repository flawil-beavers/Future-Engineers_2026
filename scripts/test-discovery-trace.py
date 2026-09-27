import importlib.util
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('discovery_trace', ROOT / 'simulation/analyze_discovery_trace.py')
MODEL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODEL)


class DiscoveryTraceTests(unittest.TestCase):
    def report(self, number):
        return MODEL.analyze(ROOT / f'simulation/evidence/parking_exit_diagnostics/20260926_log_{number}_cw.txt')

    def test_empty_inner_seat_never_visible_during_both_recorded_approaches(self):
        for number in (392, 393):
            station = next(s for s in self.report(number)['stations'] if s['station'] == 3)
            self.assertEqual(station['visible_records'][0], 0)
            start = next(row for row in station['hold_records'] if row['reason'] == 'hold_start')
            self.assertIn('too_near', start['seats'][0]['reasons'])
            self.assertIn('outside_bearing_window', start['seats'][0]['reasons'])

    def test_other_seat_clear_and_raw_block_are_distinguished(self):
        station = next(s for s in self.report(392)['stations'] if s['station'] == 3)
        self.assertTrue(station['hold_records'][0]['seats'][1]['stored_clear'])
        station = next(s for s in self.report(393)['stations'] if s['station'] == 3)
        self.assertFalse(station['hold_records'][0]['seats'][1]['stored_clear'])
        self.assertIn('rejected_colour_overlap', station['hold_records'][0]['seats'][1]['reasons'])

    def test_old_logs_do_not_claim_discovery_coverage(self):
        self.assertEqual(self.report(390)['status'], 'missing_discovery_trace')


if __name__ == '__main__':
    unittest.main()
