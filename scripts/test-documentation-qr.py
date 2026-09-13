"""Run with python scripts/test-documentation-qr.py; requires Pandoc only."""
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = "https://github.com/flawil-beavers/Future-Engineers_2026"


def convert(markdown, output="latex", repository=REPOSITORY):
    args = ["pandoc", "--from=gfm", f"--to={output}", "--standalone",
            "--lua-filter=docs/pdf/qr-codes.lua"]
    if repository is not None:
        args += [f"--metadata=repository-url:{repository}"]
    return subprocess.run(args, input=markdown, text=True, encoding="utf-8",
                          capture_output=True, cwd=ROOT)


class QrTests(unittest.TestCase):
    def test_repository_and_both_challenges(self):
        result = convert("# Video\n\n## Open Challenge\n\n"
                         "[Watch](https://youtu.be/open123)\n\n"
                         "## Obstacle Challenge\n\n"
                         "[Watch](https://www.youtube.com/watch?v=obstacle456&feature=share)\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.count("\\DocumentationQr{"), 3)
        self.assertIn("Open Challenge video", result.stdout)
        self.assertIn("Obstacle Challenge video", result.stdout)
        self.assertIn("{https://youtu.be/obstacle456}", result.stdout)
        self.assertIn("{" + REPOSITORY + "}", result.stdout)

    def test_no_placeholder_for_missing_video(self):
        result = convert("# Video\n\n## Obstacle Challenge\n\n<!-- TODO: add video -->\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.count("\\DocumentationQr{"), 1)

    def test_duplicates_and_section_boundaries(self):
        result = convert("[Outside](https://youtu.be/outside)\n\n# Video\n\n"
                         "[Preview](https://youtu.be/abc123)\n\n"
                         "[Same](https://www.youtube.com/watch?v=abc123)\n\n"
                         "# Robot Photos\n\n[Outside](https://youtu.be/other)\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.count("\\DocumentationQr{"), 2)

    def test_html_unchanged(self):
        result = convert("# Video\n\n[Watch](https://youtu.be/abc123)\n", "html", None)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("qrcode", result.stdout)
        self.assertIn('href="https://youtu.be/abc123"', result.stdout)

    def test_canonical_repository_required(self):
        self.assertNotEqual(convert("# Video\n", repository=None).returncode, 0)
        self.assertNotEqual(convert("# Video\n", repository="not-a-url").returncode, 0)


if __name__ == "__main__":
    unittest.main()
