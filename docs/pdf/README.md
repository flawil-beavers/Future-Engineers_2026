# PDF documentation build

`README.pdf` is the committed WRO documentation artifact. Its source is the
repository-root `README.md`, with `title-page.tex` inserted as the first page.
The standalone `title_page.pdf` is no longer generated or tracked.

## Requirements

Install the following commands and make them available on `PATH`:

- `pandoc`
- `xelatex` from MiKTeX or TeX Live
- The LaTeX `qrcode` package (MiKTeX can install it on first use)
- `gswin64c`, `gswin32c`, `mgs`, or `gs` from Ghostscript or MiKTeX
- `pdfinfo` and `pdftoppm` from Poppler or MiKTeX

On the current Windows development machine, Pandoc and MiKTeX provide all of
these commands. MiKTeX may install missing LaTeX packages during the first
build.

## Build

Run this command from the repository root:

```powershell
& .\scripts\build-documentation.ps1
```

Use `& .\scripts\build-documentation.ps1 -Preview` to build and render without
replacing the committed `README.pdf`.

## Automatic QR codes

`qr-codes.lua` generates vector QR codes using LaTeX, without network requests
or generated image assets. The repository QR appears on the title page, with
its readable URL. Set the canonical `repository-url` in `pandoc.yaml`; the
local Git remote is intentionally not used, so building from a fork does not
change the printed destination.

In the Video section, each distinct HTTPS YouTube link automatically receives
a 28 mm QR code below its preview/link, within the same challenge subsection.
Both `youtu.be` and `www.youtube.com/watch?v=...` links are supported, as are
YouTube Shorts links. Tracking parameters are omitted from the QR destination.
Replacing a video link updates its QR on the next build. Adding the final
Obstacle Challenge subsection and video link generates its QR automatically;
no placeholder code is generated while that link is absent.

The filter changes only LaTeX/PDF output, leaving GitHub's README presentation
unchanged. QR codes remain vectors through Ghostscript compression. Review
their placement and scan the final printed codes before publication.

`qr-layout.tex` boxes the QR matrix rows explicitly and preserves a white
four-module quiet zone, so paragraph formatting cannot stretch the modules.
The QR matrix is black even when normal hyperlinks are blue.

Run `python scripts/test-documentation-qr.py` to check automatic insertion,
URL replacement and deduplication, absent videos, canonical repository metadata,
and unchanged HTML output.

## Build pipeline

The engineering testing table uses 18% / 42% / 40% column widths and
left-aligned text, rather than the BOM's compact quantity-column layout.
Run `python scripts/test-documentation-tables.py` to check both layouts.

The script performs the complete publication pipeline:

1. Pandoc and XeLaTeX generate the document, embedded title page and vector QR codes.
2. Ghostscript downsamples printed photographs to 150 DPI, uses JPEG quality
   82, subsets fonts, and preserves text and vector content.
3. `pdfinfo` verifies that the result is a valid multi-page PDF below 50 MiB.
4. `pdftoppm` renders every page to
   `local_workspace/pdf-build/rendered-pages/` for visual review.
5. The validated candidate replaces the repository-root `README.pdf`.

Temporary, raw, candidate, and rendered files stay below
`local_workspace/pdf-build/` and are ignored by Git. Review the rendered pages
before committing `README.pdf`.

When `README.md`, the PDF configuration, the build script, or documentation
photos change, commit the regenerated `README.pdf` in the same pull request.
The documentation PDF GitHub Actions guard enforces this and rejects PDFs over
50 MiB.
