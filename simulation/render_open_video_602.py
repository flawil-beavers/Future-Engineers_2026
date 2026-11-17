#!/usr/bin/env python3
"""Present 602's two untimed Open Challenge sequences on an explicit schematic."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re

import cv2
import numpy as np
from PIL import Image, ImageDraw

from render_video_runs import BG, INK, MUTED, FPS, W, H, Pose, View, font, robot

ROOT=Path(__file__).resolve().parents[1]
ARCHIVE=ROOT/'simulation/evidence/parking_exit_diagnostics/20261031_log_602_cw.txt'
SOURCE=ARCHIVE
OUTPUT=ROOT/'local_workspace/video-documentation/continuous/log_602_schematic'


def sequences(text):
    result=[]
    for chunk in re.split(r'(?=\[STATE CHANGE\] Manual enable -> FOLLOWING)',text):
        if 'Mode OPEN_CHALLENGE completed.' not in chunk:continue
        events=[]
        for block in re.split(r'(?=\[STATE CHANGE\])',chunk):
            m=re.match(r'\[STATE CHANGE\] ([^\n]+)',block)
            if not m:continue
            h=re.search(r'Current angle: ([-\d.]+)',block)
            d=re.search(r'Distance since last state: ([-\d.]+)',block)
            if h and d:
                item=dict(event=m[1].strip(),heading=float(h[1]),distance=float(d[1]))
                for name in ('LEFT','RIGHT'):
                    reading=re.search(rf'{name}:\s+Dist: ([-\d.]+) mm \(Raw: ([-\d.]+)\)',block)
                    if reading:item[name.lower()]=dict(filtered=float(reading[1]),raw=float(reading[2]))
                events.append(item)
        corners=[e for e in events if e['event']=='Corner detected -> TURNING']
        turns=[e for e in events if e['event']=='Turn finished -> FOLLOWING']
        stop=next(e for e in events if e['event']=='Mission complete')
        if len(corners)!=12 or len(turns)!=12:raise ValueError('Expected twelve paired corners')
        result.append(dict(corners=corners,turns=turns,stop=stop))
    if len(result)!=2:raise ValueError('Expected two complete Open Challenge sequences')
    return result


def rotate(x,y,k):
    a=k*math.pi/2
    return x*math.cos(a)-y*math.sin(a),x*math.sin(a)+y*math.cos(a)


def location(kind,corner,f):
    """A normalized circuit, not reconstructed XY or calibrated distance."""
    if kind=='straight':
        start=0 if corner==0 else -750
        x,y=rotate(start+(750-start)*f,-1050,corner)
        return x,y,corner*90
    if kind=='turn':
        a=f*math.pi/2
        x,y=rotate(750+300*math.sin(a),-750-300*math.cos(a),corner)
        return x,y,corner*90+f*90
    return -750+750*f,-1050,1080


def base():
    im=Image.new('RGB',(W,H),BG);d=ImageDraw.Draw(im)
    d.text((65,35),'FUTURE ENGINEERS',font=font(23,True),fill='#6de4d3')
    d.text((65,72),'Log 602 / Open Challenge',font=font(39,True),fill=INK)
    d.text((65,124),'SCHEMATIC RECONSTRUCTION / ILLUSTRATIVE TIMING',font=font(18),fill=MUTED)
    v=View((65,175,925,1035),(-1550,1550,-1550,1550))
    d.rectangle(v.rect,fill='#f5f4ee')
    v.box(d,-1500,-1500,3000,3000,outline='#344357',width=4)
    v.box(d,-500,-500,1000,1000,fill='#dfe3e6',outline='#536273',width=3)
    d.text(v.pt(-280,50),'INNER FIELD',font=font(18,True),fill='#7a8792')
    d.rounded_rectangle((1010,175,1855,635),20,fill='#1b2b43')
    d.text((1035,690),'HOW TO READ THIS CLIP',font=font(21,True),fill='#6de4d3')
    lines=['Two completed sequences in the saved log.',
           'Each records 12 turns / 3 laps.',
           'Robot position follows a normalized circuit.',
           'Headings and straight distances are logged values.',
           'Corner sequence is preserved; pacing is illustrative.',
           'No timed XY trajectory exists for these runs.']
    for i,line in enumerate(lines):d.text((1035,740+i*38),line,font=font(21),fill=INK if i<2 else MUTED)
    d.text((1035,1010),'Schematic path and timing; not measured motion.',font=font(19),fill=MUTED)
    d.text((1035,1036),'The earlier aborted obstacle prefix is omitted.',font=font(18),fill=MUTED)
    return im,v


def main():
    cv2.setNumThreads(1)
    raw=SOURCE.read_bytes();sha=hashlib.sha256(raw).hexdigest()
    if hashlib.sha256(ARCHIVE.read_bytes()).hexdigest()!=sha:raise ValueError('Source/archive mismatch')
    seqs=sequences(raw.decode('utf-8'));OUTPUT.mkdir(parents=True,exist_ok=True)
    writers={speed:cv2.VideoWriter(str(OUTPUT/('run_602_schematic'+('_4x' if speed==4 else '')+'.mp4')),
                                  cv2.VideoWriter_fourcc(*'mp4v'),FPS,(W,H)) for speed in (1,4)}
    if not all(w.isOpened() for w in writers.values()):raise RuntimeError('Encoder unavailable')
    preview=Image.new('RGB',(1920,1080));frame_no=0
    try:
        for seqno,seq in enumerate(seqs,1):
            scene,view=base();trail=Image.new('RGBA',(W,H));td=ImageDraw.Draw(trail)
            previous=None;segments=[]
            for k,(corner,turn) in enumerate(zip(seq['corners'],seq['turns'])):
                segments.extend([('straight',k,2.0,corner),('turn',k,1.0,turn)])
            segments.append(('finish',11,2.0,seq['stop']))
            segments.append(('hold',11,2.0,seq['stop']))
            for kind,k,duration,event in segments:
                steps=round(duration*FPS/4)
                for j in range(steps):
                    f=j/max(1,steps-1)
                    x,y,h=location('finish' if kind=='hold' else kind,k,1 if kind=='hold' else f)
                    if previous:td.line([view.pt(*previous),view.pt(x,y)],fill='#10ad9a',width=4)
                    previous=(x,y)
                    im=scene.copy();im.paste(trail,(0,0),trail);d=ImageDraw.Draw(im)
                    robot(d,view,Pose(0,x,y,h,'Course driving','schematic'),'#10ad9a')
                    d.text((1035,195),f'OPEN SEQUENCE {seqno} / 2',font=font(32,True),fill=INK)
                    lap=min(3,k//4+1)
                    d.text((1035,255),f'Lap {lap} / 3     Corner {k+1} / 12',font=font(28,True),fill='#6de4d3')
                    action={'straight':'Following straight','turn':'Corner turn','finish':'Finishing straight','hold':'Completed in log'}[kind]
                    d.text((1035,310),action,font=font(27,True),fill=INK)
                    d.text((1035,378),'Recorded event endpoint',font=font(20,True),fill=MUTED)
                    d.text((1035,423),f"Gyro angle: {event['heading']:.1f} deg",font=font(27),fill=INK)
                    distance=(seq['corners'][k]['distance'] if kind=='turn' else event['distance'])
                    d.text((1035,474),f'Logged straight distance: {distance:.0f} mm',font=font(25),fill=INK)
                    d.text((1035,548),'ILLUSTRATIVE PACING / NOT RUN TIME',font=font(21,True),fill='#e5c26b')
                    rgb=np.asarray(im)
                    for speed,w in writers.items():
                        out=rgb.copy();label='standard pace' if speed==1 else '4x illustrative pace'
                        cv2.putText(out,label,(1510,95),cv2.FONT_HERSHEY_SIMPLEX,.75,(166,184,207),2,cv2.LINE_AA)
                        bgr=cv2.cvtColor(out,cv2.COLOR_RGB2BGR)
                        for _ in range(4 if speed==1 else 1):w.write(bgr)
                    if kind=='turn' and k in (1,9) and j==steps//2:
                        col=0 if k==1 else 1;preview.paste(im.resize((960,540)),(col*960,(seqno-1)*540))
                    frame_no+=4
            print(f'Open sequence {seqno}: 12 corners / 3 laps exported',flush=True)
        preview.save(OUTPUT/'preview.png')
    finally:
        for w in writers.values():w.release()
    exports=[]
    for speed in (1,4):
        target=OUTPUT/('run_602_schematic'+('_4x' if speed==4 else '')+'.mp4')
        v=cv2.VideoCapture(str(target));count=int(v.get(cv2.CAP_PROP_FRAME_COUNT))
        for i in (0,count//2,count-1):
            v.set(cv2.CAP_PROP_POS_FRAMES,i);ok,a=v.read()
            if not ok or a.shape[:2]!=(H,W):raise RuntimeError('Video verification failed')
        v.release();exports.append(dict(file=target.name,pace_multiplier=speed,frames=count,seconds=count/FPS))
    manifest=dict(source=SOURCE.relative_to(ROOT).as_posix(),archived_original=ARCHIVE.relative_to(ROOT).as_posix(),
                  sha256=sha,mode='normalized_circuit_schematic',timing='illustrative_not_recorded',
                  sequences=seqs,exports=exports)
    (OUTPUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    (OUTPUT/'README.md').write_text('# Log 602: Open Challenge schematic\n\n'
        '[Standard pace](run_602_schematic.mp4) | [4x pace](run_602_schematic_4x.mp4) | [Preview](preview.png)\n\n'
        'Both completed Open Challenge sequences are included: 12 recorded corners / three laps each. '
        'The robot stays visible and its schematic trail is continuous within each sequence. The second sequence starts a fresh trail.\n\n'
        'This log has no timed field XY poses for the Open Challenge. The rounded circuit, robot orientation and pacing are illustrative. '
        'Recorded event order, endpoint gyro angles and straight distances appear alongside the animation. Displayed distances do not set schematic path lengths. '
        'Standard pace is not real time; 4x is relative to that illustrative pacing. Earlier aborted obstacle and incomplete enable prefixes are omitted.\n\n'
        '1920 x 1080, 30 fps, silent MP4. See manifest.json for source hash, archive identity, exact extracted events and exports. '
        'Source matches the complete Git-tracked archive; firmware and original logs are unchanged.\n\n'
        'Regenerate: `python simulation/render_open_video_602.py`\n',encoding='utf-8')
    print(json.dumps(exports),flush=True)


if __name__=='__main__':main()
