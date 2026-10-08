import pathlib
import sys
import tempfile
import shutil
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'simulation'))
from visualize_robot_run import recorded_pillars, seat_position, session_text
from analyze_parking_exit_pose import parse_log_sessions


class RunVisualizationTests(unittest.TestCase):
    def test_both_directions_and_section_rotations(self):
        self.assertEqual(seat_position(5, 1), (500, -900))
        self.assertEqual(seat_position(9, 1), (900, 0))
        self.assertEqual(seat_position(12, 1), (500, 1100))
        self.assertEqual(seat_position(0, -1), (500, -900))
        self.assertEqual(seat_position(11, -1), (-1100, 500))
        for turn in (-1, 1):
            self.assertEqual(len({seat_position(i, turn) for i in range(24)}), 24)

    def test_rejected_sightings_and_clear_are_not_pillars(self):
        text = ('[RED SEAT] decision=outside_snap nearest/error=9/151 accepted=-1\n'
                '[MAP] Clear S0 station=0\n'
                '[MAP] Confirmed S0 station=2 side=LEFT color=GREEN\n'
                '[PATH] Live avoidance injected seat=5 color=GREEN clearance_mm=260\n')
        self.assertEqual(recorded_pillars(text, 1), [(5, 'GREEN', 500, -900)])

    def test_latest_runs_have_individual_maps(self):
        evidence = ROOT / 'simulation/evidence/parking_exit_diagnostics'
        expected = {476: {3:'GREEN',9:'RED',12:'RED',17:'GREEN',18:'GREEN',23:'RED'},
                    481: {5:'GREEN'}}
        for number, seats in expected.items():
            log = parse_log_sessions(evidence / f'20261006_log_{number}_ccw.txt')[0]
            self.assertEqual({seat: color for seat,color,x,y in recorded_pillars(session_text(log),1)},seats)

    def test_multi_session_text_does_not_pool_maps(self):
        path = ROOT / 'simulation/evidence/parking_exit_diagnostics/20260927_log_398_cw.txt'
        sessions = parse_log_sessions(path)
        self.assertEqual(len(sessions), 2)
        for log in sessions:
            self.assertEqual(session_text(log).count('[PARK_DIAG_CONFIG]'), 1)

    def test_batch_flag_generates_linked_images_without_changing_source(self):
        from analyze_parking_exit_batch import generate
        scratch = ROOT / 'local_workspace'
        scratch.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=scratch) as directory:
            root = pathlib.Path(directory)
            sources = root / 'sources'
            sources.mkdir()
            original = ROOT / 'simulation/evidence/parking_exit_diagnostics/20261006_log_481_ccw.txt'
            copied = sources / original.name
            shutil.copyfile(original, copied)
            output = root / 'output'
            self.assertEqual(generate(sources, output, whole_run_images=True), (1, 1, 0))
            self.assertEqual(copied.read_bytes(), original.read_bytes())
            self.assertIn('20261006_log_481_ccw_txt_whole_run.svg',
                          (output / 'parking_exit_batch.md').read_text())
            svg = output / '20261006_log_481_ccw_txt_whole_run.svg'
            self.assertIn('G5', svg.read_text())
            self.assertTrue(svg.with_suffix('.png').is_file())


if __name__ == '__main__':
    unittest.main()
