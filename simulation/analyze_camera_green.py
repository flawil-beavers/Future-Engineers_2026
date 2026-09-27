"""Replay the current green classifier/8-connectivity on a decoded camshot.

ROI sweeps are diagnostic experiments, not a proposed firmware ROI change.
Uses current green bounds H45..180,S>=30,V20..100 and the firmware's integer HSV.
"""
from pathlib import Path
import argparse
import json
from PIL import Image

parser = argparse.ArgumentParser()
parser.add_argument('image', type=Path)
parser.add_argument('--output', type=Path, default=Path('local_workspace/camera-green-analysis'))
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
image = Image.open(args.image).convert('RGB')
if image.size != (320, 240):
    raise ValueError('Expected decoded 320x240 image')
pixels = image.load()
mask = set()
for y in range(80, 240, 2):
    for x in range(0, 320, 2):
        r, g, b = pixels[x, y]
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

def components(y_min):
    remaining = {p for p in mask if p[1]*2 >= y_min}
    result = []
    while remaining:
        seed = min(remaining, key=lambda p:(p[1],p[0]))
        remaining.remove(seed)
        queue = [seed]
        for x, y in queue:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    at = x+dx, y+dy
                    if at in remaining:
                        remaining.remove(at)
                        queue.append(at)
        xs, ys = zip(*queue)
        result.append(dict(area=len(queue)*4, min_x=min(xs)*2,max_x=max(xs)*2,
            min_y=min(ys)*2,max_y=max(ys)*2,width=(max(xs)-min(xs))*2+1,
            height=(max(ys)-min(ys))*2+1))
    return sorted(result, key=lambda c:c['area'], reverse=True)[:3]

out = Image.new('RGB', image.size)
for x, y in mask:
    for dx in (0,1):
        for dy in (0,1):
            out.putpixel((x*2+dx,y*2+dy),(0,255,0))
out.save(args.output / (args.image.stem+'_green_mask.png'))
report = {str(y): components(y) for y in (80,82,84,86,88,90,100)}
(args.output / (args.image.stem+'_green_components.json')).write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
