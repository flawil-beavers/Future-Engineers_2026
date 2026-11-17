#!/usr/bin/env python3
"""Annotate existing presentation videos with traceable, sampled sensor evidence."""
from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter
import csv
from dataclasses import asdict, dataclass
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import re

import cv2
import numpy as np
from PIL import Image, ImageDraw

from analyze_parking_exit_pose import fields
from render_video_runs import ROOT, W, H, FPS, View, font, parse, pose_at, Pose
from render_open_video_602 import sequences, location

BASE=ROOT/'local_workspace/video-documentation/continuous'
OUT=ROOT/'local_workspace/video-documentation/sensor-usage'
COLORS={'tof':(64,172,243),'camera':(220,80,208)}  # BGR
font=lru_cache(maxsize=16)(font)


@dataclass
class Event:
    t: int
    sensor: str
    kind: str
    x: float
    y: float
    h: float
    label: str
    source: str
    line: int
    start_t: int | None = None
    start_x: float | None = None
    start_y: float | None = None
    details: str = ''


def xyz(value):
    result=tuple(map(float,value.replace('/',',').split(',')))
    if len(result)!=3 or not all(math.isfinite(v) for v in result):raise ValueError(value)
    return result


def obstacle_events(path):
    events=[];previous_counter={};seen_tof=set()
    for n,line in enumerate(path.read_text(encoding='utf-8').splitlines(),1):
        f=fields(line)
        if 't' not in f:continue
        t=int(f['t']);tag=line.split(']')[0][1:]
        def add(sensor,kind,p,label,**kw):
            events.append(Event(t,sensor,kind,*p,label,tag,n,**kw))
        if tag=='PARK_DIAG':
            if f.get('type')=='sample' and f.get('state','').startswith(('segment_','localize_')):
                p=xyz(f['pose'])
                for i,name in enumerate(('L','R','rear')):
                    value=f.get(f's{i}','none')
                    if value in ('none','same'):continue
                    a=value.split(',')
                    if len(a)!=8:continue
                    key=('exit',i,a[0])
                    if key in seen_tof:continue
                    seen_tof.add(key)
                    add('tof','activity',p,f'{name} cached range logged',details=f'seq={a[0]}; age_ms={a[1]}; raw={a[2]}; filtered={a[3]}; valid={a[6]}; filter_accepted={a[7]}')
            elif f.get('type')=='correction':
                add('tof','used',xyz(f['before']),f"Pose correction: {f['source']}",details=f"delta={f.get('delta','')}; before-pose marker")
        elif tag in ('DRIVE_CAM','DISCOVERY_TRACE','RED SEAT','GREEN SEAT PRIMARY','SEAT CLEAR') and 'pose' in f:
            p=xyz(f['pose'])
            if tag=='DRIVE_CAM':
                label='Camera diagnostic result logged'
            elif tag=='DISCOVERY_TRACE':
                label='Camera discovery evaluation'
            else:
                label=f"Camera {f.get('decision','clear observation recorded')}"
            accepted=tag=='SEAT CLEAR' or f.get('decision')=='confirmed'
            add('camera','used' if accepted else 'activity',p,label,
                details=f"accepted={f.get('accepted','')}; frame_t={f.get('frame_t','')}; {f.get('bearing/range','')}")
        elif tag in ('LATER_TRACK','FINAL_PARK_TRACE'):
            p=xyz(f['pose'] if tag=='LATER_TRACK' else f['xyh'])
            if tag=='FINAL_PARK_TRACE':
                a=f['tof_seq/age/raw/expected'].split('/')
                key=('final',a[0])
                if key not in seen_tof:
                    seen_tof.add(key)
                    add('tof','activity',p,'Outer ToF cached range logged',details=f'seq={a[0]}; age_ms={a[1]}; raw={a[2]}; expected={a[3]}')
            used=tuple(int(v) for v in f.get('tof_used','0,0').split(','))
            if tag=='LATER_TRACK' and any(used):
                sides='/'.join(name for name,flag in zip(('L','R'),used) if flag)
                add('tof','used',p,f'ToF used flag: {sides}',details=f"correction={f.get('correction','')}; latest sampled result")
            counter=int(f.get('cs','0'))
            prev=previous_counter.get(tag)
            if prev and counter>prev[1]:
                pt,pc,pp=prev
                add('tof','interval',p,f'{counter-pc} correction(s) since previous record',
                    start_t=pt,start_x=pp[0],start_y=pp[1],details='Counter change: location/time inside interval unknown; dashed chord is a sampled-interval symbol, not a sensor beam or driven route')
            previous_counter[tag]=(t,counter,p)
    return sorted(events,key=lambda e:e.t)


def schematic_events(path):
    events=[];tick=0
    for seqno,seq in enumerate(sequences(path.read_text(encoding='utf-8')),1):
        for k,(corner,turn) in enumerate(zip(seq['corners'],seq['turns'])):
            for kind,steps,record in (('straight',15,corner),('turn',8,turn)):
                p=location(kind,k,1);t=round((tick+steps-1)*4000/FPS)
                for side in ('left','right'):
                    if side in record:
                        events.append(Event(t,'tof','activity',*p,f'{side} range at state change',
                            record['event'],0,details=f"sequence={seqno}; schematic location/time; filtered={record[side]['filtered']}; raw={record[side]['raw']}"))
                if kind=='straight':
                    events.append(Event(t,'tof','used',*p,'ToF-triggered corner transition',record['event'],0,
                        details=f'sequence={seqno}; ToF role from navigation implementation; not an individual range-use flag'))
                tick+=steps
        # Original generator: 15 finish frames and 15 held frames per sequence.
        tick+=30
    return sorted(events,key=lambda e:e.t)


def panel(events,t,number,totals):
    im=Image.new('RGB',(860,200),'#101c30');d=ImageDraw.Draw(im)
    d.text((10,0),'SENSOR EVIDENCE / AT LOGGED ROBOT LOCATIONS',font=font(19,True),fill='#eff5fc')
    for i,(name,color) in enumerate((('ToF','#f3ac40'),('Camera','#d050dc'))):
        x=10+i*420
        d.text((x,31),name+'   hollow: activity / filled: used',font=font(17,True),fill=color)
    d.text((10,65),'Dashed amber: corrections occurred between records (not a beam).',font=font(17),fill='#a6b8cf')
    for j,sensor in enumerate(('tof','camera')):
        recent=[e for e in events if e.sensor==sensor and 0<=t-e.t<=1000]
        if recent:
            # Prefer explicit use over an accompanying activity record.
            e=max(recent,key=lambda e:({'activity':0,'interval':1,'used':2}[e.kind],e.t))
            label=e.label
        else:label='No recent logged record'
        if number=='602' and sensor=='camera':label='No camera activity logged; usage unknown'
        d.text((10,97+j*29),f'{sensor.upper()}: {label}',font=font(17),fill='#eff5fc')
    d.text((10,165),'Sampled records only. Missing markers do not mean sensors were off.',font=font(17),fill='#a6b8cf')
    return cv2.cvtColor(np.asarray(im),cv2.COLOR_RGB2BGR)


def marker(canvas,mask,view,event):
    l,top,r,bottom=view.rect;x,y=view.pt(event.x,event.y)
    if not(l+5<=x<=r-5 and top+5<=y<=bottom-5):return
    color=COLORS[event.sensor]
    if event.kind=='activity':color=tuple(round(c*.65+255*.35) for c in color)
    radius=4 if event.kind=='used' else 3
    thickness=-1 if event.kind=='used' else 1
    if event.sensor=='camera':
        points=np.array([(x,y-radius-1),(x-radius-1,y+radius),(x+radius+1,y+radius)],np.int32)
        if thickness==-1:
            cv2.fillPoly(canvas,[points],color);cv2.fillPoly(mask,[points],255)
        else:
            cv2.polylines(canvas,[points],True,color,1);cv2.polylines(mask,[points],True,255,1)
    elif event.kind!='interval':
        cv2.circle(canvas,(x,y),radius,color,thickness);cv2.circle(mask,(x,y),radius,255,thickness)
    if event.kind=='interval' and event.start_x is not None:
        a=np.array(view.pt(event.start_x,event.start_y),float);z=np.array((x,y),float)
        distance=np.linalg.norm(z-a)
        if distance<2:return
        for offset in np.arange(0,distance,9):
            p1=tuple(np.rint(a+(z-a)*offset/distance).astype(int))
            p2=tuple(np.rint(a+(z-a)*min(offset+4,distance)/distance).astype(int))
            valid,p1,p2=cv2.clipLine((l,top,r-l,bottom-top),p1,p2)
            if valid:
                cv2.line(canvas,p1,p2,(140,190,210),1);cv2.line(mask,p1,p2,255,1)


def schematic_pose(frame):
    tick=(frame//4)%306
    if tick<276:
        k,j=divmod(tick,23)
        value=location('straight',k,j/14) if j<15 else location('turn',k,(j-15)/7)
    else:
        value=location('finish',11,min(1,(tick-276)/14))
    return Pose(0,*value,'Course driving','schematic')


def restore_robot(video,original,view,p,number):
    front=125;rear=30 if p.phase=='Final parking' and number!='601' else 40
    width=123 if p.phase=='Final parking' and number!='601' else 135
    angle=math.radians(p.h);co,si=math.cos(angle),math.sin(angle)
    points=np.array([view.pt(p.x+x*co-y*si,p.y+x*si+y*co)
        for x,y in [(-rear,-width/2),(front,-width/2),(front,width/2),(-rear,width/2)]],np.int32)
    l,top,r,bottom=view.rect
    x0=max(l,int(points[:,0].min())-3);x1=min(r,int(points[:,0].max())+4)
    y0=max(top,int(points[:,1].min())-3);y1=min(bottom,int(points[:,1].max())+4)
    if x0>=x1 or y0>=y1:return
    region=np.zeros((y1-y0,x1-x0),np.uint8)
    cv2.fillConvexPoly(region,points-np.array((x0,y0)),255)
    region=cv2.dilate(region,np.ones((3,3),np.uint8))>0
    video[y0:y1,x0:x1][region]=original[y0:y1,x0:x1][region]


def overview(number,events,data):
    image=Image.new('RGB',(W,H),'#101c30');d=ImageDraw.Draw(image)
    d.text((55,25),f'Run {number} / Where sensor evidence was logged',font=font(31,True),fill='#eff5fc')
    groups=[('ToF snapshots','tof',('activity',)),('ToF use / correction intervals','tof',('used','interval')),
            ('Camera activity','camera',('activity',)),('Camera accepted observations','camera',('used',))]
    for i,(title,sensor,kinds) in enumerate(groups):
        x=55+(i%2)*945;y=85+(i//2)*470
        d.text((x,y),title,font=font(23,True),fill='#f3ac40' if sensor=='tof' else '#d050dc')
        v=View((x,y+38,x+420,y+458),(-1550,1550,-1550,1550))
        d.rectangle(v.rect,fill='#f5f4ee');v.box(d,-1500,-1500,3000,3000,outline='#344357',width=2)
        v.box(d,-500,-500,1000,1000,fill='#dfe3e6',outline='#536273',width=1)
        selected=[e for e in events if e.sensor==sensor and e.kind in kinds]
        overlay=np.zeros((H,W,3),np.uint8);mask=np.zeros((H,W),np.uint8)
        for event in selected:marker(overlay,mask,v,event)
        rgb=np.asarray(image).copy();rgb[mask>0]=overlay[:,:,::-1][mask>0];image=Image.fromarray(rgb);d=ImageDraw.Draw(image)
        d.text((x+445,y+80),f'{len(selected)} logged records',font=font(23,True),fill='#eff5fc')
        notes=['Markers locate the robot,','not the sensor target.']
        if sensor=='tof' and 'interval' in kinds:notes+=['Dashed chords symbolize','sampled intervals only.']
        if number=='602':notes+=['Schematic positions/timing.']
        if not selected:notes+=['No events logged.','Usage remains unknown.']
        for j,line in enumerate(notes):d.text((x+445,y+133+j*32),line,font=font(21),fill='#a6b8cf')
    d.text((55,1030),'Sampled/capped evidence, not all acquisitions. Activity does not imply control use. Nominal map / onboard poses.',font=font(20),fill='#a6b8cf')
    image.save(OUT/f'run_{number}_sensor_map.png')


def annotate(number):
    sources=list((ROOT/'simulation/evidence/parking_exit_diagnostics').glob(f'*_log_{number}_*.txt'))
    if len(sources)!=1:
        raise ValueError(f'Expected one archived source for run {number}, found {len(sources)}')
    path=sources[0]
    if number=='602':
        data=None;events=schematic_events(path);start=0
        source=BASE/'log_602_schematic/run_602_schematic.mp4'
        views=[View((65,175,925,1035),(-1550,1550,-1550,1550))]
    else:
        data=parse(path);events=obstacle_events(path);start=data['start']
        if data['result']:
            final=fields(data['result']).get('pose_x_y_heading')
            if final and int(data['end']['t'])>data['samples'][-1].t:
                data['samples'].append(Pose(int(data['end']['t']),*xyz(final),'Final parking','FINAL_RESULT_AT_RUN_END'))
        source=BASE/f'run_{number}_1x.mp4'
        extent=max(1550,max(max(abs(p.x),abs(p.y))+160 for p in data['samples']))
        views=[View((65,175,925,1035),(-extent,extent,-extent,extent)),
               View((1030,440,1855,870),(-650,850,-1520,-740))]
    cap=cv2.VideoCapture(str(source));count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if count<1:raise ValueError(source)
    if abs(cap.get(cv2.CAP_PROP_FPS)-FPS)>.01:raise ValueError('Base clip must use the expected 30 fps timeline')
    if number=='602' and count!=2448:raise ValueError('602 schematic pacing changed; update the sensor event mapping first')
    OUT.mkdir(parents=True,exist_ok=True)
    paths={s:OUT/f'run_{number}_sensors_{"standard" if number=="602" and s==1 else str(s)+"x"}.mp4' for s in (1,4)}
    writers={s:cv2.VideoWriter(str(p),cv2.VideoWriter_fourcc(*'mp4v'),FPS,(W,H)) for s,p in paths.items()}
    if not all(w.isOpened() for w in writers.values()):raise RuntimeError('Encoder unavailable')
    canvas=np.zeros((H,W,3),np.uint8);mask=np.zeros((H,W),np.uint8)
    times=[e.t for e in events];shown=0;totals=Counter((e.sensor,e.kind) for e in events)
    preview=None;reset=False
    pose_times=[p.t for p in data['samples']] if data else []
    links=[True]*len(pose_times)
    try:
        for frame in range(count):
            ok,video=cap.read()
            if not ok:raise RuntimeError(f'Cannot read {source} frame {frame}')
            if frame%4:continue
            original=video.copy()
            t=start+round(frame*1000/FPS)
            if number=='602' and frame>=1224 and not reset:
                canvas.fill(0);mask.fill(0);reset=True
            while shown<len(events) and events[shown].t<=t:
                for view in views:marker(canvas,mask,view,events[shown])
                shown+=1
            visible=mask>0;video[visible]=canvas[visible]
            p=schematic_pose(frame) if number=='602' else pose_at(data['samples'],pose_times,t,links,True)[0]
            for view in views:restore_robot(video,original,view,p,number)
            recent=events[bisect_right(times,t-1000):bisect_right(times,t)]
            for sensor in ('tof','camera'):
                candidates=[e for e in recent if e.sensor==sensor and e.kind!='interval']
                if not candidates:continue
                e=max(candidates,key=lambda e:(e.kind=='used',e.t))
                for view in views:
                    x,y=view.pt(e.x,e.y);l,top,r,bottom=view.rect
                    radius=max(18,round(view.scale*155))
                    if l+radius<x<r-radius and top+radius<y<bottom-radius:
                        cv2.circle(video,(x,y),radius,COLORS[e.sensor],2,cv2.LINE_AA)
            # A compact panel replaces the old trail legend / explanatory footer.
            video[880:1080,1010:1870]=panel(recent,t,number,totals)
            if number=='602':
                cv2.rectangle(video,(1010,660),(1870,879),(16,28,48),-1)
                note=Image.new('RGB',(860,220),'#101c30');d=ImageDraw.Draw(note)
                for j,line in enumerate(('OPEN CHALLENGE / SENSOR EVIDENCE',
                    'Circles: cached ToF snapshots at state changes.',
                    'Filled circles: recorded ToF-triggered corners.',
                    'Camera: no activity/use records in these sequences.',
                    'Position and event timing remain schematic.')):
                    d.text((10,10+j*38),line,font=font(20,j==0),fill='#eff5fc' if j==0 else '#a6b8cf')
                video[660:880,1010:1870]=cv2.cvtColor(np.asarray(note),cv2.COLOR_RGB2BGR)
            if preview is None and recent and any(e.kind=='used' for e in recent):preview=video.copy()
            for s,w in writers.items():
                out=video
                if s==4:
                    out=video.copy()
                    cv2.rectangle(out,(1480,45),(1919,120),(48,28,16),-1)
                    cv2.putText(out,'4x illustrative pace' if number=='602' else '4x playback',
                        (1510,95),cv2.FONT_HERSHEY_SIMPLEX,.75,(166,184,207),2,cv2.LINE_AA)
                for _ in range(min(4,count-frame) if s==1 else 1):w.write(out)
            if frame%(FPS*30)==0:print(f'{number}: {frame/FPS:.0f}/{count/FPS:.1f}s annotated',flush=True)
        if preview is not None:cv2.imwrite(str(OUT/f'run_{number}_sensors_preview.png'),preview)
    finally:
        cap.release()
        for w in writers.values():w.release()
    with (OUT/f'run_{number}_sensor_events.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=list(Event.__dataclass_fields__));writer.writeheader()
        writer.writerows(asdict(e) for e in events)
    result=dict(run=number,source=path.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                base_video=source.relative_to(ROOT).as_posix(),event_counts={f'{s}_{k}':n for (s,k),n in totals.items()},
                timing='schematic' if number=='602' else 'logged timestamps; positions from logged poses',exports=[])
    overview(number,events,data)
    for s,p in paths.items():
        v=cv2.VideoCapture(str(p));n=int(v.get(cv2.CAP_PROP_FRAME_COUNT))
        for at in (0,n//2,n-1):
            v.set(cv2.CAP_PROP_POS_FRAMES,at);ok,a=v.read()
            if not ok or a.shape[:2]!=(H,W):raise RuntimeError(f'Export decode failed: {p}')
        v.release();result['exports'].append(dict(file=p.name,pace=s,frames=n,seconds=n/FPS))
    print(json.dumps(result),flush=True)
    return result


def write_index(results):
    dji_open_available = (OUT/'open-challenge-dji-synced/README.md').exists()
    tof_positioned_open_available = (OUT/'open-challenge-tof-positioned/README.md').exists()
    filmed_open_available = (OUT/'open-challenge-video-aligned/README.md').exists()
    rows=['# Sensor activity and use visualizations','','Hollow amber circles: logged ToF snapshots. Hollow magenta triangles: camera results/evaluations. Filled symbols: explicit ToF use flags or pose corrections, accepted camera observations/clear records. Dashed amber: ToF correction counters changed between sampled records; this is an interval symbol, not a beam or measured route. Pulses persist for one second of source time for readability.','','| Run | Standard / 1x | 4x | Static map |','| --- | --- | --- | --- |']
    for r in sorted(results,key=lambda r:int(r['run'])):
        if r['run']=='602' and dji_open_available:
            rows.append('| 602 DJI first Open run | [Same-speed video path](open-challenge-dji-synced/open_dji_video_panel_1x.mp4) | [4x](open-challenge-dji-synced/open_dji_video_panel_4x.mp4) | [Field preview](open-challenge-dji-synced/preview_field_only.png) |')
            continue
        if r['run']=='602' and tof_positioned_open_available:
            rows.append('| 602 first Open run | [ToF-positioned estimate](open-challenge-tof-positioned/run_602_tof_positioned_with_panel_standard.mp4) | [4x](open-challenge-tof-positioned/run_602_tof_positioned_with_panel_4x.mp4) | [Field preview](open-challenge-tof-positioned/preview_field_only.png) |')
            continue
        if r['run']=='602' and filmed_open_available:
            rows.append('| 602 filmed run | [1x / filmed trajectory](open-challenge-video-aligned/open_1318_with_video_panel_1x.mp4) | [4x](open-challenge-video-aligned/open_1318_with_video_panel_4x.mp4) | [Field preview](open-challenge-video-aligned/preview_field_only.png) |')
            continue
        if r['run']=='602' and (OUT/'open-challenge-rays/README.md').exists():
            rows.append('| 602 | [Illustrative pace / rays](open-challenge-rays/run_602_tof_rays_with_panel_standard.mp4) | [4x / rays](open-challenge-rays/run_602_tof_rays_with_panel_4x.mp4) | [Ray preview](open-challenge-rays/preview_field_only.png) |')
            continue
        a,b=r['exports'];rows.append(f"| {r['run']} | [{ 'Illustrative pace' if r['run']=='602' else '1x'}]({a['file']}) | [4x]({b['file']}) | [Sensor map](run_{r['run']}_sensor_map.png) |")
    if dji_open_available:
        rows[2:2]=['**Latest Open Challenge:** [DJI overhead path at source speed, field-only and synchronized film-panel versions](open-challenge-dji-synced/README.md). Logged range crosses and red residuals distinguish measurements from modeled wall intersections. Sensor-event synchronization remains estimated.','']
    elif tof_positioned_open_available:
        rows[2:2]=['**Latest Open Challenge revision:** [ToF-positioned first run / field-only and panel versions](open-challenge-tof-positioned/README.md). Inner-wall ToF sets lateral position against the 1000 x 1800 mm enclosure; outer-wall disagreement is shown as a residual. Along-track positions and pacing remain estimates. Older exports contain both Open sequences.','']
    elif filmed_open_available:
        rows[2:2]=['**Filmed Open Challenge update**. [Log 602 first Open sequence, video-aligned field-only and source-film panel versions](open-challenge-video-aligned/README.md). The selected centre is 1000 x 1800 mm. Sensor synchronization is estimated from the film and untimed log events.','']
    elif (OUT/'open-challenge-rays/README.md').exists():
        rows[2:2]=['**Open Challenge update:** [Log 602 ToF rays, with panel and field-only versions](open-challenge-rays/README.md). Orange = left ToF, blue = right ToF; solid = logged range, dashed = out of range. The circle/triangle legend below applies to the other runs.','']
    rows += ['', 'The videos retain the continuous interpolated robot motion. Sensor symbols mark the robot location attached to each logged record, not a measured target position. Cached ranges and camera diagnostics do not prove fresh acquisition or use. Filter acceptance is not equivalent to a control correction. Accepted camera observations do not establish physical correctness. Missing markers mean missing evidence, not sensors off. Records are sampled/capped: counts are logged records, not every sensor frame. Rear-positioning samples in a separate local coordinate frame are omitted from the field map; their position cannot be directly joined to the field coordinates.', '', 'ToF correction counter changes are summarized only between successive sampled records of the same stream. The first counter value is a baseline; prior corrections are not located. A used flag describes the sampled latest result. A counter change can include additional corrections whose exact poses/sides are unavailable. Camera confirmations use decision=confirmed and SEAT CLEAR; other processed results remain activity.', '', 'Older 602 exports include both Open Challenge sequences; the latest DJI reconstruction covers the first sequence only. Only state-change L/R range snapshots are available. The schematic marks range snapshots at corner-entry and turn-finish records. Filled corner markers represent logged corner transitions with ToF control role established from the navigation implementation; they are not per-reading acceptance flags. Their timing/positions are schematic. No camera activity/use is logged for those sequences, so camera use is unknown.', '', 'Each run has a CSV containing source tag/line, timestamp, pose, classification and interval bounds. Obstacle records retain original timestamps; 602 uses illustrative presentation milliseconds and line=0 because its state blocks have no timestamps. Sources and firmware are unchanged. No additional robot test was requested.', '', 'Regenerate: `python simulation/render_sensor_usage.py`']
    (OUT/'README.md').write_text('\n'.join(rows)+'\n',encoding='utf-8')


def main():
    cv2.setNumThreads(1)
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--runs',nargs='+',choices=['601','602','632','633','634'],default=['601','602','632','633','634'])
    args=parser.parse_args();results=[]
    if (OUT/'manifest.json').exists():
        results=[r for r in json.loads((OUT/'manifest.json').read_text()) if r['run'] not in args.runs]
    for number in args.runs:
        result=annotate(number)
        for original in (ROOT/'simulation/evidence/parking_exit_diagnostics').glob('*.txt'):
            if hashlib.sha256(original.read_bytes()).hexdigest()==result['sha256']:
                result['archived_original']=original.relative_to(ROOT).as_posix();break
        if 'archived_original' not in result:raise ValueError('No matching portable archived original')
        results.append(result);(OUT/'manifest.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
    write_index(results)


if __name__=='__main__':main()
