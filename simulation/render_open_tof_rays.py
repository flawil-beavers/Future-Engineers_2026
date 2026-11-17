#!/usr/bin/env python3
"""Render log602 ToF range rays: explanatory and clean field-only exports."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from functools import lru_cache
import hashlib
import json
import math

import cv2
import numpy as np
from PIL import Image, ImageDraw

from render_open_video_602 import SOURCE, ARCHIVE, ROOT, sequences, location
from render_video_runs import BG, INK, MUTED, FPS, View, Pose, font, robot

OUT=ROOT/'local_workspace/video-documentation/sensor-usage/open-challenge-rays'
LEFT='#eea12c'
RIGHT='#428bdf'
font=lru_cache(maxsize=20)(font)


@dataclass
class Reading:
    tick: int
    sequence: int
    corner: int
    x: float
    y: float
    h: float
    left: dict
    right: dict
    source: str


def ray(x,y,h,side,distance):
    """Nominal configured side ToF mounts, measured range from sensor face."""
    a=math.radians(h);sign=1 if side=='left' else -1
    origin=(x+40*math.cos(a)-sign*35*math.sin(a),
            y+40*math.sin(a)+sign*35*math.cos(a))
    direction=(math.cos(a+sign*math.pi/2),math.sin(a+sign*math.pi/2))
    valid=math.isfinite(distance) and 0<distance<9999
    if valid:
        end=(origin[0]+distance*direction[0],origin[1]+distance*direction[1])
    else:
        candidates=[]
        for i in (0,1):
            if abs(direction[i])>1e-8:
                edge=1500 if direction[i]>0 else -1500
                t=(edge-origin[i])/direction[i]
                if t>0:candidates.append(t)
        length=min(candidates) if candidates else 0
        end=(origin[0]+length*direction[0],origin[1]+length*direction[1])
    return origin,end,valid


def schedule(seqs):
    frames=[];readings=[]
    for seqno,seq in enumerate(seqs,1):
        for k,(corner,turn) in enumerate(zip(seq['corners'],seq['turns'])):
            for kind,steps,event in [('straight',15,corner),('turn',8,turn)]:
                for j in range(steps):
                    x,y,h=location(kind,k,j/(steps-1))
                    if j==steps-1:
                        readings.append(Reading(len(frames),seqno,k+1,x,y,event['heading'],
                                                event['left'],event['right'],event['event']))
                    frames.append(dict(sequence=seqno,corner=k+1,kind=kind,x=x,y=y,h=h))
        for kind in ('finish','hold'):
            for j in range(15):
                x,y,h=location('finish',11,1 if kind=='hold' else j/14)
                frames.append(dict(sequence=seqno,corner=12,kind=kind,x=x,y=y,h=h))
    return frames,readings


def scene(field_only,extent):
    width,height=(1080,1080) if field_only else (1920,1080)
    im=Image.new('RGB',(width,height),'#f5f4ee' if field_only else BG)
    d=ImageDraw.Draw(im)
    view=View((0,0,1080,1080) if field_only else (65,175,925,1035),(-extent,extent,-extent,extent))
    d.rectangle(view.rect,fill='#f5f4ee')
    view.box(d,-1500,-1500,3000,3000,outline='#344357',width=4)
    view.box(d,-500,-500,1000,1000,fill='#dfe3e6',outline='#536273',width=3)
    if not field_only:
        d.text((65,35),'FUTURE ENGINEERS',font=font(23,True),fill='#6de4d3')
        d.text((65,72),'Log 602 / Open Challenge / ToF rays',font=font(35,True),fill=INK)
        d.text((65,124),'SCHEMATIC POSITION / ILLUSTRATIVE TIMING',font=font(18),fill=MUTED)
        d.rounded_rectangle((1010,175,1855,555),20,fill='#1b2b43')
        d.text((1035,604),'MEASUREMENT RAYS',font=font(23,True),fill=INK)
        for i,(label,color) in enumerate([('LEFT ToF',LEFT),('RIGHT ToF',RIGHT)]):
            y=658+i*45
            d.line((1035,y+12,1095,y+12),fill=color,width=4)
            d.text((1115,y),label,font=font(23,True),fill=color)
        notes=['Solid line + cross: logged filtered range endpoint.',
               'Dashed line: out of range / invalid; no target point.',
               'Dashed rays end at the outer field boundary.',
               'Rays pulse at the logged snapshot location.',
               'Nominal mounts: forward 40 mm, left/right 35 mm.',
               'Camera measurements are not logged in these runs.']
        for i,line in enumerate(notes):d.text((1035,770+i*34),line,font=font(19),fill=MUTED)
        d.text((1035,1010),'Endpoints depend on the schematic robot position.',font=font(19),fill=MUTED)
        d.text((1035,1036),'They do not establish a measured wall/target location.',font=font(18),fill=MUTED)
    return im,view


def draw_reading(draw,view,r):
    for side,color in [('left',LEFT),('right',RIGHT)]:
        distance=r.__dict__[side]['filtered']
        origin,end,valid=ray(r.x,r.y,r.h,side,distance)
        a=np.array(view.pt(*origin),float);b=np.array(view.pt(*end),float)
        if valid:
            draw.line([tuple(a),tuple(b)],fill=color,width=4)
            x,y=b
            draw.line((x-5,y-5,x+5,y+5),fill=color,width=3)
            draw.line((x-5,y+5,x+5,y-5),fill=color,width=3)
        else:
            length=np.linalg.norm(b-a)
            if length>0:
                for offset in np.arange(0,length,15):
                    p=a+(b-a)*offset/length;q=a+(b-a)*min(offset+9,length)/length
                    draw.line([tuple(p),tuple(q)],fill=color,width=3)


def label(distance):
    if not math.isfinite(distance) or distance<=0:return 'Invalid / unavailable'
    if distance>=9999:return 'Out of range'
    return f'{distance:.0f} mm'


def render():
    cv2.setNumThreads(1)
    raw=SOURCE.read_bytes();sha=hashlib.sha256(raw).hexdigest()
    if sha!=hashlib.sha256(ARCHIVE.read_bytes()).hexdigest():raise ValueError('Archive/source mismatch')
    frames,readings=schedule(sequences(raw.decode('utf-8')))
    assert len(frames)==612 and len(readings)==48
    # Keep all finite endpoints visible; dashed rays always stop at outer walls.
    finite=[]
    for r in readings:
        for side in ('left','right'):
            _,end,valid=ray(r.x,r.y,r.h,side,r.__dict__[side]['filtered'])
            if valid:finite.extend(abs(v)+80 for v in end)
    extent=max([1580]+finite)
    OUT.mkdir(parents=True,exist_ok=True)
    scenes={mode:scene(mode,extent) for mode in (False,True)}
    writers={};paths={}
    for mode in (False,True):
        for speed in (1,4):
            name=f'run_602_tof_rays_{"field_only" if mode else "with_panel"}_{"standard" if speed==1 else "4x"}.mp4'
            path=OUT/name;size=(1080,1080) if mode else (1920,1080)
            writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'mp4v'),FPS,size)
            if not writer.isOpened():raise RuntimeError('Encoder unavailable')
            writers[mode,speed]=writer;paths[mode,speed]=path
    trail={mode:Image.new('RGBA',im.size) for mode,(im,v) in scenes.items()}
    last=None;last_seq=0;index=0;latest=None;previews={}
    try:
        for tick,f in enumerate(frames):
            if f['sequence']!=last_seq:
                for mode,(im,v) in scenes.items():trail[mode]=Image.new('RGBA',im.size)
                last=None;latest=None;last_seq=f['sequence']
            while index<len(readings) and readings[index].tick<=tick:
                latest=readings[index];index+=1
            for mode,(base,v) in scenes.items():
                td=ImageDraw.Draw(trail[mode]);point=v.pt(f['x'],f['y'])
                if last is not None:td.line([v.pt(*last),point],fill='#10ad9a',width=4)
                im=base.copy();im.paste(trail[mode],(0,0),trail[mode]);d=ImageDraw.Draw(im)
                active=latest is not None and latest.sequence==f['sequence'] and tick-latest.tick<=7
                if active:draw_reading(d,v,latest)
                robot(d,v,Pose(0,f['x'],f['y'],f['h'],'Course driving','schematic'),'#10ad9a')
                if not mode:
                    d.text((1035,195),f"OPEN SEQUENCE {f['sequence']} / 2",font=font(30,True),fill=INK)
                    d.text((1035,248),f"Lap {(f['corner']-1)//4+1} / 3    Corner {f['corner']} / 12",font=font(26,True),fill='#6de4d3')
                    d.text((1035,308),'Latest logged snapshot',font=font(22,True),fill=INK)
                    d.text((1035,354),'LEFT: '+(label(latest.left['filtered']) if latest else 'Awaiting first snapshot'),font=font(23),fill=LEFT)
                    d.text((1035,397),'RIGHT: '+(label(latest.right['filtered']) if latest else 'Awaiting first snapshot'),font=font(23),fill=RIGHT)
                    d.text((1035,452),'Rays visible for one illustrative second.' if active else 'No recent snapshot shown as a ray.',font=font(20),fill=MUTED)
                    d.text((1035,503),'SCHEMATIC / NOT RECORDED RUN TIME',font=font(20,True),fill='#e5c26b')
                arr=np.asarray(im)
                for speed in (1,4):
                    out=arr.copy()
                    if not mode:cv2.putText(out,'standard pace' if speed==1 else '4x illustrative pace',(1510,105),cv2.FONT_HERSHEY_SIMPLEX,.75,(166,184,207),2,cv2.LINE_AA)
                    bgr=cv2.cvtColor(out,cv2.COLOR_RGB2BGR)
                    for _ in range(4 if speed==1 else 1):writers[mode,speed].write(bgr)
                if active and latest.corner==1 and f['sequence']==1 and mode not in previews:
                    previews[mode]=im.copy()
            last=(f['x'],f['y'])
            if tick%150==0:print(f'602 rays: {tick}/{len(frames)} presentation frames',flush=True)
        for mode,im in previews.items():im.save(OUT/f'preview_{"field_only" if mode else "with_panel"}.png')
    finally:
        for writer in writers.values():writer.release()
    exports=[]
    for (mode,speed),path in paths.items():
        cap=cv2.VideoCapture(str(path));n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH));height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        for frame in (0,n//2,n-1):
            cap.set(cv2.CAP_PROP_POS_FRAMES,frame);ok,a=cap.read()
            if not ok:raise RuntimeError(f'Cannot decode {path.name}')
        cap.release();exports.append(dict(file=path.name,field_only=mode,pace_multiplier=speed,frames=n,width=width,height=height,seconds=n/FPS))
    manifest=dict(source=SOURCE.relative_to(ROOT).as_posix(),archived_original=ARCHIVE.relative_to(ROOT).as_posix(),sha256=sha,
        timing='illustrative',positions='normalized schematic circuit',range='logged filtered Dist values',
        sensor_mounts_mm=dict(forward=40,left=35,right=-35),field_view_half_extent_mm=extent,
        ray_records=[asdict(r) for r in readings],exports=exports)
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    (OUT/'README.md').write_text('# Open Challenge / log 602 / ToF measurement rays\n\n'
        '| View | Standard illustrative pace | 4x illustrative pace |\n| --- | --- | --- |\n'
        '| With explanatory panel (1920 x 1080) | [Standard](run_602_tof_rays_with_panel_standard.mp4) | [4x](run_602_tof_rays_with_panel_4x.mp4) |\n'
        '| Field only (1080 x 1080; no text/panel) | [Standard](run_602_tof_rays_field_only_standard.mp4) | [4x](run_602_tof_rays_field_only_4x.mp4) |\n\n'
        '[Panel preview](preview_with_panel.png) | [Field-only preview](preview_field_only.png)\n\n'
        'Both completed Open Challenge sequences are included. Orange rays represent LEFT ToF; blue rays represent RIGHT ToF. '
        'Solid rays use each logged filtered Dist value in millimetres, ending in a small cross. A 9999/out-of-range or invalid value gets a dashed ray to the outer field boundary without a target marker. '
        'Circles from the previous sensor overlay are removed. Rays pulse for one illustrative second at the snapshot pose; unrecorded readings are not synthesized.\n\n'
        'Robot XY position and pacing remain schematic because log 602 has no timed field-position stream. '
        'Ray origins use configured nominal sensor mounts (40 mm forward, 35 mm left/right) and each snapshot\'s logged relative gyro heading. '
        'Consequently a finite endpoint is a projected location from an approximate pose, not independently established wall geometry. '
        'Finite rays can cross nominal walls if the schematic pose and logged range disagree; lengths are preserved. Dashed boundary endpoints are clipping limits, not measured detections. '
        'No camera readings are logged for these Open Challenge sequences.\n\n'
        'The field-only files contain only the field, trail, robot and rays; use the file name to distinguish playback pace. '
        'Both views use the same timeline and data. Standard pace is illustrative, not real time. All files are silent 30 fps MP4. '
        'Original logs, firmware and previous visualizations are unchanged. See manifest.json for source SHA-256 and the 48 paired snapshots.\n\n'
        'Regenerate: `python simulation/render_open_tof_rays.py`\n',encoding='utf-8')
    print(json.dumps(exports),flush=True)


if __name__=='__main__':render()
