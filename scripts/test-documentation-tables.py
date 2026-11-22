"""Run with python scripts/test-documentation-tables.py; requires Pandoc."""
import json
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


def table_widths(markdown):
    result = subprocess.run(
        ["pandoc", "--from=gfm", "--to=json",
         "--lua-filter=docs/pdf/fit-table-images.lua"],
        input=markdown, text=True, encoding="utf-8", capture_output=True, cwd=ROOT)
    if result.returncode:
        raise RuntimeError(result.stderr)
    table = next(block for block in json.loads(result.stdout)["blocks"]
                 if block["t"] == "Table")
    return [column[1]["c"] for column in table["c"][2]]


class TableTests(unittest.TestCase):
    def assertWidths(self, markdown, expected):
        actual = table_widths(markdown)
        self.assertEqual(len(actual), len(expected))
        for value, target in zip(actual, expected):
            self.assertAlmostEqual(value, target)

    def test_engineering_evidence_has_two_wide_prose_columns(self):
        self.assertWidths(
            "| Subsystem | Test evidence | Decision or improvement |\n"
            "|---|---|---|\n| Camera | Measured results | Retained design |\n",
            [0.18, 0.42, 0.40])

    def test_bom_quantity_column_stays_compact(self):
        self.assertWidths(
            "| Component | Quantity | Purpose |\n|---|---|---|\n"
            "| Motor | 1 | Drive |\n", [0.48, 0.14, 0.38])

    def test_current_bom_fits_all_four_columns(self):
        self.assertWidths(
            "| Component | Quantity | Function | Estimated CHF |\n"
            "|---|---|---|---|\n| Motor | 1 | Drive | 13.90 |\n",
            [0.31, 0.12, 0.43, 0.14])

    def test_current_integration_table_keeps_both_prose_columns_wide(self):
        self.assertWidths(
            "| Subsystems | What drove the change | What we chose and what we learned |\n"
            "|---|---|---|\n| Camera | Missed signs | Slower approach |\n",
            [0.18, 0.35, 0.47])


if __name__ == "__main__":
    unittest.main()
