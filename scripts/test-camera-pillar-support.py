"""Offline recovery limits; no robot perception acceptance."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'simulation'))
from camera_pillar_support import candidate, components, production_limits, shape, valid, analyze


def rectangle(x0, x1, y0, y1):
    return frozenset((x, y) for x in range(x0, x1+1) for y in range(y0, y1+1))


class SupportTests(unittest.TestCase):
    def setUp(self):
        self.limits = production_limits()

    def test_real_near_green_images(self):
        root = Path(__file__).resolve().parents[1] / 'simulation/evidence/camera_diagnostics'
        for label in ('prototype', '01', '02'):
            p = root / ('20260927_green_camshot_' + label + '.png')
            result = analyze(p, self.limits)
            self.assertFalse(result['raw_valid'])
            for threshold in ('7', '9', '11'):
                recovered = result['candidates_by_minimum_column_samples'][threshold]
                self.assertEqual(len(recovered), 1)
                self.assertLessEqual(recovered[0]['width'], 35)
                self.assertEqual(recovered[0]['max_y'], result['raw']['max_y'])

    def test_far_valid_and_fragmented_raw_are_unchanged(self):
        solid = rectangle(75, 84, 40, 50)  # 440px area,21px height
        fragmented = solid - {(x, y) for x, y in solid if y in (43, 47) and x % 3 == 0}
        for raw in (solid, fragmented):
            self.assertTrue(valid(raw, self.limits))
            recovered, preserved = candidate(raw, self.limits)
            self.assertIs(preserved, raw)
            self.assertEqual(recovered, [raw])

    def test_short_or_thick_horizontal_background_remains_invalid(self):
        for raw in (rectangle(10, 150, 40, 42), rectangle(10, 150, 40, 54)):
            recovered, preserved = candidate(raw, self.limits)
            self.assertEqual(recovered, [])
            self.assertIs(preserved, raw)

    def test_two_pillars_split_and_raw_is_preserved(self):
        raw = rectangle(10, 150, 40, 41) | rectangle(50, 62, 40, 66) | rectangle(95, 107, 40, 66)
        recovered, preserved = candidate(raw, self.limits)
        self.assertEqual(len(recovered), 2)
        self.assertIs(preserved, raw)
        self.assertTrue(all(c <= raw for c in recovered))

    def test_separate_fragments_are_not_pooled(self):
        mask = rectangle(70, 90, 40, 46) | rectangle(70, 90, 49, 55)
        groups = components(mask)
        self.assertEqual(len(groups), 2)
        self.assertTrue(all(not candidate(c, self.limits)[0] for c in groups))

    def test_background_vertical_patch_is_an_explicit_ambiguity(self):
        # A coloured wall patch can have the same binary mask as a real pillar.
        # This counterexample requires additional real images before deployment.
        raw = rectangle(10, 150, 40, 41) | rectangle(75, 87, 40, 66)
        recovered, preserved = candidate(raw, self.limits)
        self.assertEqual(len(recovered), 1)
        self.assertIs(preserved, raw)


if __name__ == '__main__':
    unittest.main()
