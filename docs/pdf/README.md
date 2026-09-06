# PDF documentation build

`README.pdf` is the committed WRO documentation artifact. Its source is the
repository-root `README.md`, with `title-page.tex` inserted as the first page.
The standalone `title_page.pdf` is no longer generated or tracked.

## Requirements

Install the following commands and make them available on `PATH`:

- `pandoc`
- `xelatex` from MiKTeX or TeX Live
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

The script performs the complete publication pipeline:

1. Pandoc and XeLaTeX generate the document and embedded title page.
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
