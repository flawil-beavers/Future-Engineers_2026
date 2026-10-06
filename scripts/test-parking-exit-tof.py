"""Geometry, observation provenance and route-preservation regressions."""
import math
import pathlib
import sys
import unittest
from types import SimpleNamespace

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
import analyze_parking_exit_pose as pose
import analyze_parking_exit_tof as tof


class TofAssessmentTests(unittest.TestCase):
    def setUp(self):
        self.geometry = tof.Geometry.load()

    def sample(self, t=100, heading=180, sensor=0, value="1,0,250,250,5,2,1,1", state="localize_drive"):
        return pose.Sample(t, state, 1, -1, 10, -60, -60, 0, 1, heading,
                           (100., -1215., heading), (100., -1215., heading),
                           "parking_edges", {f"s{sensor}": value})

    def session(self, samples, events=None, corrections=None):
        return SimpleNamespace(samples=samples, events=events if events is not None else
                               [{"type": "rebase", "detail": "field_start", "t": "10"}],
                               corrections=corrections or [], path=pathlib.Path("20261006_log_999_cw.txt"),
                               source_sha256="test", session_index=1, config={"build": "test"})

    def test_repository_geometry_matches_field_frame(self):
        g = self.geometry
        self.assertEqual((g.south, g.fixed_x, g.gap, g.length), (-1500, 480, 247.5, 200))
        self.assertEqual(g.left, (40, 35))
        self.assertEqual(g.right, (40, -35))

    def test_perpendicular_and_oblique_wall_geometry(self):
        walls = [("wall", (-1000, 0), (1000, 0))]
        for angle in (0, 10, 20, 30):
            hit = tof.ray_hit((0, 100), math.radians(-90 + angle), walls)
            self.assertAlmostEqual(hit[0], 100 / math.cos(math.radians(angle)))
            self.assertAlmostEqual(hit[2], angle)

    def test_mirrored_headings_use_opposite_side_sensor(self):
        cw = tof.observation_rows(self.session([self.sample()]), "legacy", self.geometry)[0]
        ccw = tof.observation_rows(self.session([self.sample(heading=0, sensor=1)]), "legacy", self.geometry)[0]
        self.assertTrue(cw["usable"])
        self.assertTrue(ccw["usable"])
        self.assertEqual(cw["target"], "black_south")
        self.assertEqual(ccw["target"], "black_south")
        self.assertAlmostEqual(cw["expected_mm"], ccw["expected_mm"])
        self.assertEqual((cw["incidence_deg"], ccw["incidence_deg"]), (0, 0))

    def test_pink_edge_mixed_fan_and_unidentified_ray(self):
        g = self.geometry
        self.assertEqual(tof.classify((485, -1200), -math.pi/2, g, 22)[0], "mixed-surface")
        self.assertEqual(tof.classify((100, -1200), math.pi/2, g, 22)[0], "unidentified")
        custom = tof.Geometry(-1500, 480, 247.5, 200, 200, g.left, g.right, g.rear)
        self.assertEqual(tof.classify((484, -1290), -math.pi/2, custom, 22)[0], "edge-sensitive")

    def test_duplicate_invalid_stale_and_seed_observations(self):
        samples = [self.sample(), self.sample(t=110), self.sample(t=120, value="same"),
                   self.sample(t=130, value="2,51,250,250,5,2,1,1"),
                   self.sample(t=140, value="3,0,-1,9999,-1,-1,0,0"),
                   self.sample(t=150, value="4,0,250,250,5,2,1,1", state="rear_settle")]
        rows = tof.observation_rows(self.session(samples), "legacy", self.geometry)
        self.assertEqual(len(rows), 4)
        self.assertIn("stale", rows[1]["reason"])
        self.assertIn("invalid-or-rejected", rows[2]["reason"])
        self.assertFalse(rows[3]["usable"])
        self.assertTrue(rows[0]["usable"])

    def test_missing_rebase_and_cached_read_crossing_pose_correction(self):
        rows = tof.observation_rows(self.session([self.sample()], events=[]), "legacy", self.geometry)
        self.assertEqual(rows[0]["raw_residual_mm"], "")
        sample = self.sample(value="1,20,250,250,5,2,1,1")
        rows = tof.observation_rows(self.session([sample], corrections=[{"t": "90"}]), "legacy", self.geometry)
        self.assertIn("acquisition-crosses-pose-correction", rows[0]["reason"])
        seed = self.sample(t=20, value="1,15,250,250,5,2,1,1")
        rows = tof.observation_rows(self.session([seed]), "legacy", self.geometry)
        self.assertIn("acquisition-crosses-field-seed", rows[0]["reason"])

    def test_filter_difference_and_unexplained_step_are_only_candidates(self):
        rows = tof.observation_rows(self.session([
            self.sample(), self.sample(t=200, value="2,0,300,260,5,2,1,1")]), "legacy", self.geometry)
        self.assertEqual(rows[1]["raw_filtered_delta_mm"], -40)
        self.assertTrue(rows[1]["target_change_candidate"])


if __name__ == "__main__":
    unittest.main()
