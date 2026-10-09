"""Check the shipped crop, embedded copy, resources and printed-seat alignment."""
import base64
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
metadata = json.loads((HERE / 'assets/fe-2026-mat.json').read_text())
asset = (HERE / 'assets/fe-2026-mat.svg').read_bytes()
assert hashlib.sha256(asset).hexdigest() == metadata['asset_sha256']
html = (HERE / 'Robot Run Inspector.html').read_text(encoding='utf-8')
embedded = json.loads(re.search(r'const OFFICIAL_FE_MAT=("[^"]+");', html)[1])
assert base64.b64decode(embedded.split(',', 1)[1]) == asset
root = ET.fromstring(asset)
view = list(map(float, root.get('viewBox').split()))
assert all(abs(a - b) < .0001 for a, b in zip(view, metadata['svg_view_box']))
assert root.get('width') == root.get('height') == '3200mm'
assert all(abs(v * 25.4 / 72 - 3200) < .01 for v in view[2:])


def world(px, py):
    return ((px - view[0]) / view[2] * 3200 - 1600,
            1600 - (py - view[1]) / view[3] * 3200)


assert world(view[0] + view[2] / 2, view[1] + view[3] / 2) == (0, 0)
for x, y in [(-1500, 1500), (1500, -1500)]:
    # The 3000 mm track is inset 100 mm from the trimmed mat perimeter.
    px = view[0] + (x + 1600) / 3200 * view[2]
    py = view[1] + (1600 - y) / 3200 * view[3]
    assert all(abs(a - b) < 1e-9 for a, b in zip(world(px, py), (x, y)))

# Measure the 24 original yellow seat rings from the converted PDF paths,
# independently of the Inspector's seat-position helper. The other two yellow
# paths are large field/centre outlines, not sign seats. These source paths use
# absolute M/L/C commands; extrema of each ring give its printed centre.
centres = []
for node in root.iter():
    name = node.tag.rsplit('}', 1)[-1]
    assert name not in ('script', 'foreignObject')
    for key, value in node.attrib.items():
        if key.rsplit('}', 1)[-1] == 'href':
            assert value.startswith(('#', 'data:image/')), value[:80]
    if name != 'path' or '80.000305%' not in node.get('style', ''):
        continue
    path = node.get('d')
    assert not re.search(r'[a-zA-BD-KN-Y]', path)
    values = list(map(float, re.findall(r'-?\d+(?:\.\d+)?', path)))
    xs, ys = values[::2], values[1::2]
    if max(xs) - min(xs) < 300:
        centres.append(world((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2))
expected = [(a, b) for a in (-500, 0, 500) for b in (-1100, -900, 900, 1100)]
expected += [(b, a) for a in (-500, 0, 500) for b in (-1100, -900, 900, 1100)]
assert len(centres) == len(expected) == 24
for x, y in expected:
    assert any(abs(cx - x) < .6 and abs(cy - y) < .6 for cx, cy in centres), (x, y)
print('PASS: source crop, 3200 mm size, origin, track bounds, 24 printed seats, offline asset identity/resources')
