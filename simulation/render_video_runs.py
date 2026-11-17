#!/usr/bin/env python3
"""Export onboard-pose replays, optionally filling gaps for presentation.

Requires numpy, Pillow and opencv-python. Outputs are working artifacts under
local_workspace; sources and archived originals are read only.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from analyze_parking_exit_pose import fields, wrap180
from visualize_robot_run import recorded_pillars

ROOT = Path(__file__).resolve().parents[1]
FPS = 30
# Original streams are sparse (typically 5-10 Hz). Render at 7.5 Hz and repeat
# frames for the 30 fps real-time export; 4x uses every rendered frame once.
RENDER_STEP = 4
W, H = 1920, 1080
BG = '#101c30'
INK = '#eff5fc'
MUTED = '#a6b8cf'
COLORS = {'Parking exit': '#267cce', 'Scan / discovery': '#ba7cdb',
          'Course driving': '#10ad9a', 'Final parking': '#f3ac40'}


@dataclass(frozen=True)
class Pose:
    t: int
    x: float
    y: float
    h: float
    phase: str
    source: str


def parse(path, *, include_scan_poses=False):
    text = path.read_text(encoding='utf-8', errors='replace')
    starts = re.findall(r'\[RUN_START\].*? t=(\d+)', text)
    ends = re.findall(r'\[RUN_END\] (.*)', text)
    if len(starts) != 1 or len(ends) != 1:
        raise ValueError(f'{path.name}: expected one obstacle session')
    end = fields(ends[0])
    start = int(starts[0])
    turn = int(re.search(r'\[PARK_DIAG_CONFIG\].*? turn=(-?\d+)', text)[1])
    poses, corrections = {}, []
    for line in text.splitlines():
        f = fields(line)
        if line.startswith('[PARK_DIAG]') and f.get('type') == 'correction':
            corrections.append(int(f['t']))
        if 't' not in f:
            continue
        phase = source = value = None
        if line.startswith('[PARK_DIAG]') and f.get('type') == 'sample':
            if f.get('state', '').startswith(('segment_', 'localize_')):
                value, phase, source = f['pose'], 'Parking exit', 'PARK_DIAG'
        elif line.startswith(('[DISCOVERY_TRACE]', '[RED SEAT]')) and 'pose' in f:
            value, phase, source = f['pose'], 'Scan / discovery', 'SPARSE_POSE'
        elif line.startswith('[CONNECTOR_TRACK]'):
            value = ','.join(f[k] for k in ('x', 'y', 'h'))
            phase, source = 'Scan / discovery', 'CONNECTOR_TRACK'
        elif line.startswith(('[LATER_TRACK]', '[DRIVE_CAM]')) and 'pose' in f:
            value, phase, source = f['pose'], 'Course driving', line.split(']')[0][1:]
        elif line.startswith('[FINAL_PARK_TRACE]'):
            value, phase, source = f['xyh'].replace('/', ','), 'Final parking', 'FINAL_PARK_TRACE'
        elif include_scan_poses:
            if line.startswith(('[GREEN SEAT PRIMARY]', '[SEAT CLEAR]', '[CORNER VIEW POSE]')):
                value, phase, source = f.get('pose'), 'Scan / discovery', 'SPARSE_POSE'
            elif line.startswith('[PARK_DIAG]') and f.get('type') == 'correction':
                value, phase, source = f.get('after'), 'Parking exit', 'PARK_DIAG correction'
        if value is not None:
            xyz = tuple(map(float, value.split(',')))
            if len(xyz) == 3 and all(math.isfinite(v) for v in xyz):
                t = int(f['t'])
                poses[t] = Pose(t, *xyz, phase, source)
    samples = sorted(poses.values(), key=lambda p: p.t)
    if not samples:
        raise ValueError(f'{path.name}: no field poses; schematic needs separate authorization')
    finish = max(int(end['t']), samples[-1].t)
    result = re.findall(r'\[FINAL PARK RESULT\] (.*)', text)
    return dict(start=start, finish=finish, samples=samples, corrections=corrections,
                turn=turn, end=end, result=result[-1] if result else None,
                pillars=recorded_pillars(text, turn))


def joinable(a, b, corrections):
    dt = b.t-a.t
    return (0 < dt <= 1500 and a.phase == b.phase
            and math.hypot(b.x-a.x, b.y-a.y) <= 1100*dt/1000+8
            and abs(wrap180(b.h-a.h)) <= 150*dt/1000+5
            and not any(a.t < t <= b.t for t in corrections))


def pose_at(samples, times, t, links, continuous=False):
    i = bisect_right(times, t)-1
    if i < 0:
        if continuous:
            return samples[0], -1, 'First recorded pose held'
        return None, -1, 'No field pose recorded'
    a = samples[i]
    if i+1 < len(samples) and (continuous or links[i]):
        b = samples[i+1]
        f = (t-a.t)/(b.t-a.t)
        return Pose(int(t), a.x+f*(b.x-a.x), a.y+f*(b.y-a.y),
                    a.h+f*wrap180(b.h-a.h), a.phase, a.source), i, 'Interpolated between recorded poses'
    if continuous:
        return a, i, 'Last recorded pose held'
    if t-a.t <= 250:
        return a, i, 'Logged pose sample'
    return None, i, 'No pose recorded in this interval'


def font(size, bold=False):
    names = ('arialbd.ttf','DejaVuSans-Bold.ttf') if bold else ('arial.ttf','DejaVuSans.ttf')
    for name in names:
        try:
            return ImageFont.truetype(name,size)
        except OSError:
            continue
    raise RuntimeError('Install Arial or DejaVu Sans to render video text')


class View:
    def __init__(self, rect, bounds):
        self.rect, self.bounds = rect, bounds
        l, top, r, bottom = rect
        xmin, xmax, ymin, ymax = bounds
        self.scale = min((r-l)/(xmax-xmin), (bottom-top)/(ymax-ymin))
        self.cx, self.cy = (l+r)/2, (top+bottom)/2

    def pt(self, x, y):
        xmin, xmax, ymin, ymax = self.bounds
        return (round(self.cx+(x-(xmin+xmax)/2)*self.scale),
                round(self.cy-(y-(ymin+ymax)/2)*self.scale))

    def box(self, draw, x, y, w, h, **kw):
        a, b = self.pt(x, y+h), self.pt(x+w, y)
        draw.rectangle((*a, *b), **kw)


def robot(draw, view, p, color, front=125, rear=40, width=135):
    angle = math.radians(p.h)
    def xy(x, y):
        return view.pt(p.x+x*math.cos(angle)-y*math.sin(angle),
                       p.y+x*math.sin(angle)+y*math.cos(angle))
    draw.polygon([xy(-rear,-width/2),xy(front,-width/2),
                  xy(front,width/2),xy(-rear,width/2)], fill=color, outline='#15253c', width=2)
    draw.line([xy(10,0),xy(front-10,0)],fill='white',width=3)
    draw.polygon([xy(front-9,0),xy(front-36,22),xy(front-36,-22)], fill='white')


def clock(ms):
    return f'{ms/1000:06.1f} s'


def make_base(number, data, continuous=False):
    im = Image.new('RGB',(W,H),BG)
    d = ImageDraw.Draw(im)
    d.text((65,35),'FUTURE ENGINEERS',font=font(23,True),fill='#6de4d3')
    d.text((65,70),f'Run {number}  /  '+('Clockwise' if data['turn']==-1 else 'Counterclockwise'),font=font(39,True),fill=INK)
    d.text((65,122),'CONTINUOUS INTERPOLATED REPLAY' if continuous else 'ONBOARD POSE REPLAY',font=font(18),fill=MUTED)
    extent = max(1550, max(max(abs(p.x),abs(p.y))+160 for p in data['samples'])) if continuous else 1550
    field = View((65,175,925,1035),(-extent,extent,-extent,extent))
    detail = View((1030,440,1855,870),(-650,850,-1520,-740))
    d.rounded_rectangle((1010,175,1855,365),20,fill='#1b2b43')
    d.text((1035,389),'PARKING DETAIL',font=font(20,True),fill=MUTED)
    for view in (field,detail):
        l,t,r,b=view.rect
        d.rectangle((l,t,r,b),fill='#f5f4ee')
        for v in range(-1500,1501,500):
            if view.bounds[0]<=v<=view.bounds[1]:
                d.line([view.pt(v,view.bounds[2]),view.pt(v,view.bounds[3])],fill='#e2e3df',width=1)
            if view.bounds[2]<=v<=view.bounds[3]:
                d.line([view.pt(view.bounds[0],v),view.pt(view.bounds[1],v)],fill='#e2e3df',width=1)
        if view is field:
            view.box(d,-1500,-1500,3000,3000,outline='#344357',width=4)
            view.box(d,-500,-500,1000,1000,fill='#dfe3e6',outline='#536273',width=3)
            d.text(view.pt(-285,50),'INNER FIELD',font=font(18,True),fill='#7a8792')
        else:
            d.line([view.pt(-650,-1500),view.pt(850,-1500)],fill='#344357',width=4)
        # Nominal start parking rails. Actual return scan may localize them differently.
        for x in (232.5,500):
            view.box(d,x,-1500,20,200,fill='#e7a2c4',outline='#ab618c',width=2)
        for seat,color,x,y in data['pillars']:
            if view.bounds[0] <= x <= view.bounds[1] and view.bounds[2]<=y<=view.bounds[3]:
                view.box(d,x-25,y-25,50,50,fill='#d85454' if color=='RED' else '#27a46a',outline='#2c3c49',width=1)
    d.text((1035,895),'Trail colors',font=font(18,True),fill=MUTED)
    for i,(name,c) in enumerate(COLORS.items()):
        x=1035+(i%2)*410;y=931+(i//2)*32
        d.line((x,y+10,x+27,y+10),fill=c,width=5)
        d.text((x+39,y-2),name,font=font(19),fill=INK)
    d.text((1035,1010),'Missing positions interpolated; endpoint poses held.' if continuous else 'Estimated motion; gaps are not reconstructed.',font=font(19),fill=MUTED)
    d.text((1035,1036),'Map / rails: nominal geometry. Not camera tracking.',font=font(17),fill=MUTED)
    return im, field, detail


def export(path, output, evidence, continuous=False):
    number = re.search(r'log_(\d+)',path.name)[1]
    data = parse(path)
    result_anchor = False
    if continuous and data['result']:
        final = fields(data['result']).get('pose_x_y_heading')
        t = int(data['end']['t'])
        if final and t > data['samples'][-1].t:
            xyz = tuple(map(float, final.split('/')))
            if len(xyz) == 3 and all(math.isfinite(v) for v in xyz):
                data['samples'].append(Pose(t,*xyz,'Final parking','FINAL_RESULT_AT_RUN_END'))
                result_anchor = True
    samples = data['samples'];times=[p.t for p in samples]
    links=[continuous or joinable(a,b,data['corrections']) for a,b in zip(samples,samples[1:])]+[False]
    base,field,detail=make_base(number,data,continuous)
    # Accumulate completed intervals. Default mode respects coverage breaks;
    # continuous mode deliberately connects every successive field pose.
    trail = {view: Image.new('RGBA',(W,H)) for view in (field,detail)}
    td = {v:ImageDraw.Draw(trail[v]) for v in trail}
    writers={}
    paths={}
    for speed in (1,4):
        target=output/f'run_{number}_{speed}x.mp4'
        writer=cv2.VideoWriter(str(target),cv2.VideoWriter_fourcc(*'mp4v'),FPS,(W,H))
        if not writer.isOpened():raise RuntimeError('MP4 encoder unavailable')
        writers[speed]=writer;paths[speed]=target
    duration=(data['finish']-data['start'])/1000
    count=math.ceil(duration*FPS)+1
    drawn=-1
    thumb=None
    fonts={s:font(s,bold) for s,bold in [(36,True),(24,True),(22,False),(19,False)]}
    try:
        for frame in range(0,count,RENDER_STEP):
            t=min(data['finish'],data['start']+frame*1000/FPS)
            p,i,status=pose_at(samples,times,t,links,continuous)
            while drawn < i:
                drawn+=1
                if drawn>0 and links[drawn-1]:
                    a,b=samples[drawn-1:drawn+1]
                    for v in trail:
                        td[v].line([v.pt(a.x,a.y),v.pt(b.x,b.y)],fill=COLORS[b.phase],width=3 if v is field else 4)
            im=base.copy()
            for v in trail:
                # Clip to each viewport; out-of-field estimates must not spill into text.
                rect=v.rect
                im.paste(trail[v].crop(rect),rect[:2],trail[v].crop(rect))
            if continuous and p and 0 <= i < len(samples)-1:
                # Extend the trail right up to the animated robot, rather than
                # revealing the entire interpolated span ahead of it.
                a=samples[i]
                for v in trail:
                    layer=Image.new('RGBA',(W,H));ld=ImageDraw.Draw(layer)
                    ld.line([v.pt(a.x,a.y),v.pt(p.x,p.y)],fill=COLORS[p.phase],width=3 if v is field else 4)
                    rect=v.rect;crop=layer.crop(rect);im.paste(crop,rect[:2],crop)
            d=ImageDraw.Draw(im)
            d.text((1035,193),clock(t-data['start']),font=fonts[36],fill=INK)
            d.text((1035,244),p.phase if p else 'Telemetry gap',font=fonts[24],fill=COLORS[p.phase] if p else '#e5c26b')
            d.text((1035,285),status,font=fonts[22],fill=MUTED)
            if p:
                d.text((1035,324),f'X {p.x:.0f} mm    Y {p.y:.0f} mm    Heading {p.h%360:.1f} deg',font=fonts[19],fill=MUTED)
                for v in trail:
                    # Render robot to a clipped overlay.
                    layer=Image.new('RGBA',(W,H));ld=ImageDraw.Draw(layer)
                    robot(ld,v,p,COLORS[p.phase],rear=30 if p.phase=='Final parking' and number!='601' else 40,
                          width=123 if p.phase=='Final parking' and number!='601' else 135)
                    rect=v.rect;crop=layer.crop(rect);im.paste(crop,rect[:2],crop)
            if t>=data['finish']-2000 and not continuous:
                label=('Logged completion' if data['end']['outcome']=='completed' else 'Logged abort')
                d=ImageDraw.Draw(im)
                d.rectangle((95,943,895,1020),fill='#172a42')
                d.text((115,955),label+': '+data['end']['reason'].replace('_',' '),font=fonts[22],fill=INK)
                d.text((115,988),'Physical outcome must be checked in the filmed run.',font=fonts[19],fill=MUTED)
            rgb=np.asarray(im)
            for speed,writer in writers.items():
                out=rgb.copy()
                cv2.putText(out,f'{speed}x playback',(1650,92),cv2.FONT_HERSHEY_SIMPLEX,.8,(166,184,207),2,cv2.LINE_AA)
                encoded=cv2.cvtColor(out,cv2.COLOR_RGB2BGR)
                repeats=min(RENDER_STEP,count-frame) if speed==1 else 1
                for _ in range(repeats):writer.write(encoded)
            if p and p.phase=='Final parking' and thumb is None:
                thumb=im.copy()
            if frame%(FPS*30)==0:
                print(f'run {number}: {frame/FPS:.0f}/{duration:.1f}s',flush=True)
        # Three-second end card at each playback rate, preserving the full timeline first.
        for speed,writer in writers.items():
            card=base.copy();d=ImageDraw.Draw(card)
            for v in trail:
                rect=v.rect;crop=trail[v].crop(rect);card.paste(crop,rect[:2],crop)
            if continuous:
                last=samples[-1]
                for v in trail:
                    layer=Image.new('RGBA',(W,H));ld=ImageDraw.Draw(layer)
                    robot(ld,v,last,COLORS[last.phase],rear=30 if last.phase=='Final parking' and number!='601' else 40,
                          width=123 if last.phase=='Final parking' and number!='601' else 135)
                    rect=v.rect;crop=layer.crop(rect);card.paste(crop,rect[:2],crop)
            d.text((1035,195),'RUN '+number+' / END',font=fonts[36],fill=INK)
            d.text((1035,249),'Completed in log' if data['end']['outcome']=='completed' else 'Aborted in log',font=fonts[24],fill='#6de4d3')
            d.text((1035,290),data['end']['reason'].replace('_',' '),font=fonts[22],fill=INK)
            d.text((1035,328),'Original elapsed time: '+clock(data['finish']-data['start']),font=fonts[19],fill=MUTED)
            bgr=cv2.cvtColor(np.asarray(card),cv2.COLOR_RGB2BGR)
            for _ in range(FPS*3):writer.write(bgr)
        (thumb or im).save(output/f'run_{number}_preview.png')
    finally:
        for writer in writers.values():writer.release()
    sha=hashlib.sha256(path.read_bytes()).hexdigest()
    archived=evidence.get(sha)
    if not archived:raise ValueError('Source does not match a portable archived original')
    info=dict(run=number,mode='continuous_interpolation' if continuous else 'coverage_preserving',
              final_result_anchored_at_run_end=result_anchor,
              source=archived,sha256=sha,
              archived_original=archived,elapsed_seconds=duration,samples=len(samples),
              joined_intervals=sum(links),unjoined_intervals=len(links)-1-sum(links),
              first_pose_seconds=(samples[0].t-data['start'])/1000,
              last_pose_seconds=(samples[-1].t-data['start'])/1000,
              outcome=data['end'],final_result_without_timestamp=data['result'],exports=[])
    for speed,target in paths.items():
        cap=cv2.VideoCapture(str(target));frames=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));fps=cap.get(cv2.CAP_PROP_FPS)
        ok,decoded=cap.read();cap.release()
        if not ok or frames<1 or decoded.shape[:2]!=(H,W):raise RuntimeError('Video verification failed')
        info['exports'].append(dict(file=target.name,speed=speed,frames=frames,fps=fps,
                                    seconds=frames/fps,bytes=target.stat().st_size))
    print(json.dumps(info),flush=True)
    return info


def main():
    cv2.setNumThreads(1)
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=ROOT/'simulation/evidence/parking_exit_diagnostics')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--continuous',action='store_true',help='Interpolate every gap; hold first/last poses outside coverage')
    parser.add_argument('--runs',nargs='+',choices=['601','632','633','634'],help='Render selected runs; retain other manifest entries')
    args=parser.parse_args()
    if args.output is None:
        args.output=ROOT/'local_workspace/video-documentation'
        if args.continuous:args.output=args.output/'continuous'
    args.output.mkdir(parents=True,exist_ok=True)
    evidence={hashlib.sha256(p.read_bytes()).hexdigest():p.relative_to(ROOT).as_posix()
              for p in (ROOT/'simulation/evidence/parking_exit_diagnostics').glob('*.txt')}
    manifest=[]
    if args.runs and (args.output/'manifest.json').exists():
        manifest=[r for r in json.loads((args.output/'manifest.json').read_text(encoding='utf-8')) if r['run'] not in args.runs]
    for p in sorted(args.source.glob('*.txt')):
        if re.search(r'(?:^|_)log_(601|632|633|634)(?:_|\.)',p.name):
            if args.runs and re.search(r'log_(\d+)',p.name)[1] not in args.runs:continue
            manifest.append(export(p,args.output,evidence,args.continuous))
            manifest.sort(key=lambda r:int(r['run']))
            (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    if args.continuous:
        rows=['# Continuous interpolated run videos','','Robot visible throughout; continuous trails across every missing interval. 1920 x 1080 MP4, 30 fps, silent, at 1x and 4x.','','| Run | Real time | Accelerated |','| --- | --- | --- |']
        rows.extend(f"| {r['run']} | [1x](run_{r['run']}_1x.mp4) | [4x](run_{r['run']}_4x.mp4) |" for r in manifest)
        rows += ['', 'XY positions use straight-line interpolation between successive recorded field poses, including gaps, phase changes and estimator corrections. Heading follows the shortest angular change. These connecting paths are presentation approximations and may cut across corners or obstacles. Before the first recorded position and after the last, the nearest recorded pose is held; no unsupported motion is extrapolated. For completed runs 633/634, the separately logged final parking pose is assigned to RUN_END as an approximate timing anchor; the unrecorded parking tail interpolates to it. The robot remains visible on the end card.', '', 'The full-field view includes every pose with chassis margin. The parking inset shows the fixed parking region. Nominal map geometry and logged completion/abort labels retain their original meaning. Run 602 stays excluded. Original coverage-preserving videos are in the parent folder.', '', 'Regenerate: `python simulation/render_video_runs.py --continuous`', '', 'See manifest.json for unchanged source hashes, archived evidence references, durations and export metadata.']
        if (args.output/'log_602_schematic/README.md').exists():
            rows=[s.replace('Run 602 stays excluded.', 'Log 602 is available separately as an Open Challenge schematic.') for s in rows]
            rows += ['', '## Log 602 / Open Challenge schematic', '', '[Standard illustrative pace](log_602_schematic/run_602_schematic.mp4) | [4x illustrative pace](log_602_schematic/run_602_schematic_4x.mp4) | [Notes and preview](log_602_schematic/README.md)', '', 'Includes both completed Open Challenge sequences. XY path and pacing are illustrative; standard pace is not real time.']
        if (args.output.parent/'sensor-usage/README.md').exists():
            rows += ['', '## Sensor activity and use', '', '[Sensor-overlay clips and static maps for all five logs](../sensor-usage/README.md)']
        (args.output/'README.md').write_text('\n'.join(rows)+'\n',encoding='utf-8')
    print('Verified exports:',len(manifest)*2)


if __name__=='__main__':main()
