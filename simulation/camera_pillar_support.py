"""Offline column-support experiment; NOT used by robot firmware.

Keep valid raw detections unchanged. For a broad rejected connected component,
remove columns with insufficient original colour evidence and recheck geometry.
Never use the filtered mask to certify an empty seat: raw evidence is preserved.
"""
from collections import Counter
from pathlib import Path
import argparse
import hashlib
import json
import re


def green_mask(image):
    """Current integer HSV, logical 2px grid, H45..180/S>=30/V20..100."""
    mask = set()
    for y in range(80, 240, 2):
        for x in range(0, 320, 2):
            r, g, b = image.getpixel((x, y))
            hi, lo = max(r, g, b), min(r, g, b)
            delta = hi-lo
            sat = delta*255//hi if hi else 0
            if not delta:
                hue = 0
            elif hi == r:
                hue = int(60*(g-b)/delta)
            elif hi == g:
                hue = 120+int(60*(b-r)/delta)
            else:
                hue = 240+int(60*(r-g)/delta)
            if hue < 0:
                hue += 360
            if 45 <= hue <= 180 and sat >= 30 and 20 <= hi <= 100:
                mask.add((x//2, y//2))
    return mask


def components(mask):
    remaining = set(mask)
    groups = []
    while remaining:
        seed = min(remaining, key=lambda p: (p[1], p[0]))
        remaining.remove(seed)
        queue = [seed]
        for x, y in queue:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    at = x+dx, y+dy
                    if at in remaining:
                        remaining.remove(at)
                        queue.append(at)
        # Firmware discards components with fewer than20 colour samples.
        if len(queue) >= 20:
            groups.append(frozenset(queue))
    return sorted(groups, key=len, reverse=True)


def shape(points):
    xs, ys = zip(*points)
    return dict(area=len(points)*4, min_x=min(xs)*2, max_x=max(xs)*2,
        min_y=min(ys)*2, max_y=max(ys)*2,
        center_x=sum(xs)*2//len(xs), center_y=sum(ys)*2//len(ys),
        width=(max(xs)-min(xs))*2+1, height=(max(ys)-min(ys))*2+1)


def production_limits():
    text = (Path(__file__).resolve().parents[1] / 'include/config.h').read_text()
    names = ('OBSTACLE_GREEN_MIN_AREA', 'OBSTACLE_GREEN_MIN_HEIGHT',
        'OBSTACLE_MIN_BOTTOM_Y', 'OBSTACLE_MAX_TOP_Y', 'OBSTACLE_START_MIN_X',
        'OBSTACLE_START_MAX_X', 'OBSTACLE_MAX_START_WIDTH',
        'OBSTACLE_MAX_START_HEIGHT', 'OBSTACLE_MAX_WIDTH_HEIGHT_RATIO')
    return {name: float(re.search(r'\b'+name+r'\s*=\s*([0-9.]+)', text)[1]) for name in names}


def valid(points, limits):
    if not points:
        return False
    s = shape(points)
    return (s['area'] >= limits['OBSTACLE_GREEN_MIN_AREA'] and
        s['height'] >= limits['OBSTACLE_GREEN_MIN_HEIGHT'] and
        s['max_y'] >= limits['OBSTACLE_MIN_BOTTOM_Y'] and
        s['min_y'] <= limits['OBSTACLE_MAX_TOP_Y'] and
        limits['OBSTACLE_START_MIN_X'] <= s['center_x'] <= limits['OBSTACLE_START_MAX_X'] and
        s['width'] <= limits['OBSTACLE_MAX_START_WIDTH'] and
        s['height'] <= limits['OBSTACLE_MAX_START_HEIGHT'] and
        s['width'] <= s['height']*limits['OBSTACLE_MAX_WIDTH_HEIGHT_RATIO'])


def candidate(raw, limits, minimum_column_samples=9):
    if not raw:
        return [], raw
    if valid(raw, limits):
        return [raw], raw
    s = shape(raw)
    if (s['width'] <= limits['OBSTACLE_MAX_START_WIDTH'] and
            s['width'] <= s['height']*limits['OBSTACLE_MAX_WIDTH_HEIGHT_RATIO']):
        return [], raw
    # Count within THIS connected component, never across separate regions.
    counts = Counter(x for x, y in raw)
    kept = {p for p in raw if counts[p[0]] >= minimum_column_samples}
    return [c for c in components(kept) if valid(c, limits)], raw


def analyze(path, limits):
    from PIL import Image
    image = Image.open(path).convert('RGB')
    if image.size != (320, 240):
        raise ValueError('Expected decoded320x240 image')
    groups = components(green_mask(image))
    raw = groups[0] if groups else frozenset()
    sweeps = {}
    for threshold in (5, 7, 9, 11):
        candidates, preserved = candidate(raw, limits, threshold)
        assert preserved is raw
        sweeps[str(threshold)] = [shape(c) for c in candidates]
    return dict(image=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        raw=shape(raw) if raw else None, raw_valid=valid(raw, limits),
        candidates_by_minimum_column_samples=sweeps)


def comparison(path, limits, destination):
    """Scientific diagnostic figure: source, classification mask, offline probe."""
    from PIL import Image, ImageDraw
    image = Image.open(path).convert('RGB')
    mask = green_mask(image)
    groups = components(mask)
    recovered, _ = candidate(groups[0] if groups else frozenset(), limits, 9)
    figure = Image.new('RGB', (960, 286), 'white')
    figure.paste(image, (0, 24))
    for offset, points in ((320, mask), (640, set().union(*recovered))):
        panel = Image.new('RGB', (320, 240), 'black')
        for x, y in points:
            for dx in (0, 1):
                for dy in (0, 1):
                    panel.putpixel((x*2+dx, y*2+dy), (0, 255, 0))
        figure.paste(panel, (offset, 24))
    draw = ImageDraw.Draw(figure)
    for x, title in ((4, 'Kamerabild'), (324, 'Gruenmaske heute'), (644, 'Spaltenprobe, nur offline')):
        draw.text((x, 5), title, fill='black')
    draw.text((4, 269), 'Gleiche Aufnahme; keine Freigabe fuer Fahrbetrieb oder Leer-Erkennung.', fill='black')
    figure.save(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('images', type=Path, nargs='+')
    parser.add_argument('--report', type=Path,
        default=Path('local_workspace/camera-green-analysis/pillar-support-report.json'))
    args = parser.parse_args()
    limits = production_limits()
    report = dict(production_limits=limits, results=[analyze(p, limits) for p in args.images],
        limitations='Placements and physical/reference distances require separate reports; '
            'synthetic masks are not colour or camera tests. No runtime timing or '
            'firmware acceptance. Filtered pixels cannot prove CLEAR. '
            'Background with a vertical coloured patch can resemble a pillar.')
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2)+'\n')
    comparison(args.images[-1], limits, args.report.parent / 'support_comparison.png')
    print(json.dumps(report, indent=2))
