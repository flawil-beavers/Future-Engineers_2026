"""Reproduce the official, trimmed FE mat and embed it in the offline Inspector.

Requires Poppler's pdftocairo only when regenerating. The shipped HTML needs
no Python, Poppler, local asset files or internet connection.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import urllib.request
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SOURCE = 'https://wro-association.org/wp-content/uploads/WRO-2026_FutureEngineers_Playfield.pdf'
SOURCE_HASH = '72687af113d0af0ec66dc86618449f5dc31e2fd67e58ebd019b89d5d4143744f'
SVG_NS = 'http://www.w3.org/2000/svg'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pdf', type=Path, default=ROOT / 'local_workspace/WRO-2026_FutureEngineers_Playfield.pdf')
    args = parser.parse_args()
    args.pdf.parent.mkdir(parents=True, exist_ok=True)
    if not args.pdf.exists():
        args.pdf.write_bytes(urllib.request.urlopen(SOURCE).read())
    data = args.pdf.read_bytes()
    if hashlib.sha256(data).hexdigest() != SOURCE_HASH:
        raise SystemExit('Source PDF changed: review source, crop and alignment before regeneration.')
    # These page boxes are plain PDF arrays in this pinned official source.
    trim = [float(v) for v in re.search(rb'/TrimBox\s*\[([^]]+)\]', data)[1].split()]
    media = [float(v) for v in re.search(rb'/MediaBox\s*\[([^]]+)\]', data)[1].split()]
    crop = [trim[0], media[3] - trim[3], trim[2] - trim[0], trim[3] - trim[1]]
    assert all(abs(v * 25.4 / 72 - 3200) < .01 for v in crop[2:])
    converter = shutil.which('pdftocairo')
    if not converter:
        raise SystemExit('Regeneration requires pdftocairo from Poppler on PATH.')
    raw = ROOT / 'local_workspace/official-fe-mat-raw.svg'
    raw.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([converter, '-svg', str(args.pdf), str(raw)], check=True)
    ET.register_namespace('', SVG_NS)
    ET.register_namespace('xlink', 'http://www.w3.org/1999/xlink')
    svg = ET.fromstring(raw.read_bytes())
    for node in svg.iter():
        if node.tag.rsplit('}', 1)[-1] in ('script', 'foreignObject'):
            raise SystemExit('Unexpected active SVG content')
        for key, value in node.attrib.items():
            if key.rsplit('}', 1)[-1] == 'href' and not value.startswith(('#', 'data:image/')):
                raise SystemExit('Unexpected external SVG resource')
    svg.set('width', '3200mm')
    svg.set('height', '3200mm')
    svg.set('viewBox', ' '.join(f'{v:.4f}' for v in crop))
    desc = ET.Element(f'{{{SVG_NS}}}desc')
    desc.text = f'Official WRO Future Engineers mat artwork; source {SOURCE}; SHA256 {SOURCE_HASH}; cropped to PDF TrimBox. WRO artwork and marks belong to their respective owners.'
    svg.insert(0, desc)
    rendered = ET.tostring(svg, encoding='utf-8', xml_declaration=True)
    assets = HERE / 'assets'
    assets.mkdir(exist_ok=True)
    (assets / 'fe-2026-mat.svg').write_bytes(rendered)
    metadata = dict(source_url=SOURCE, source_sha256=SOURCE_HASH,
                    asset_sha256=hashlib.sha256(rendered).hexdigest(),
                    media_box_pt=media, trim_box_pt=trim, svg_view_box=crop,
                    mat_size_mm=[3200, 3200], bounds_mm=[-1600, 1600, -1600, 1600],
                    orientation='PDF top is +Y; +X right; no CW/CCW mirroring',
                    converter='pdftocairo -svg (Poppler)',
                    verified_date='2026-10-09')
    (assets / 'fe-2026-mat.json').write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
    payload = 'data:image/svg+xml;base64,' + base64.b64encode(rendered).decode('ascii')
    html = HERE / 'Robot Run Inspector.html'
    content = html.read_text(encoding='utf-8')
    content, count = re.subn(r'/\* BEGIN OFFICIAL FE MAT \*/[\s\S]*?/\* END OFFICIAL FE MAT \*/',
                           '/* BEGIN OFFICIAL FE MAT */\nconst OFFICIAL_FE_MAT=' + json.dumps(payload) + ';\n/* END OFFICIAL FE MAT */', content)
    if count != 1:
        raise SystemExit('Expected exactly one embedded asset marker in Inspector')
    html.write_text(content, encoding='utf-8')
    print(f'Generated {len(rendered):,} byte SVG and embedded offline artwork.')


if __name__ == '__main__':
    main()
