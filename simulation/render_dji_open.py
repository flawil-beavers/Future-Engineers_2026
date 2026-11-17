#!/usr/bin/env python3
"""Film-clock Open Challenge reconstruction with explicit ToF wall residuals."""
import argparse
from functools import lru_cache
from dataclasses import asdict
import json,math
from pathlib import Path
import cv2,numpy as np
from PIL import Image,ImageDraw
from scipy.ndimage import gaussian_filter1d
from render_video_runs import ROOT,View,Pose,robot,font
from render_open_video_602 import SOURCE,ARCHIVE,sequences
from render_video_aligned_open import initial_ranges,sha256
from render_open_tof_rays import ray,draw_reading,Reading,LEFT,RIGHT
from render_tof_positioned_open import lateral_coordinate,wall_hit,valid_range,chassis_clear

OUT=ROOT/'local_workspace/video-documentation/sensor-usage/open-challenge-dji-synced'
font=lru_cache(maxsize=20)(font)


def align(data,text):
    track=data['track'];ts=np.array([p['t'] for p in track]);xy=np.array([[p['x'],p['y']] for p in track])
    hs=np.array([p['heading'] for p in track]);speeds=np.array([p['speed'] for p in track])
    # The image centroid approximates the chassis centre; convert to rear axle.
    xy-=42.5*np.column_stack([np.cos(np.deg2rad(hs)),np.sin(np.deg2rad(hs))])
    moving=np.where(speeds>60)[0];start=max(0,ts[moving[0]]-.2);stop=min(data['duration'],ts[moving[-1]]+.2)
    seq=sequences(text)[0];anchors=[]
    def anchor(t,section,event,kind,corner):
        pos=np.array([np.interp(t,ts,xy[:,a]) for a in (0,1)])
        original=pos.copy();axis=(1,0,1,0)[section%4]
        pos[axis],source=lateral_coordinate(section,event,pos[axis])
        if source=='left':
            # Keep the nominal side ray on the finite face. This changes only
            # the approximate along-track coordinate when a corner is clipped.
            _,end,_=ray(*pos,event['heading'],'left',event['left']['filtered'])
            a=1-axis;bound=500 if a==0 else 900
            pos[a]+=np.clip(end[a],-bound+8,bound-8)-end[a]
        return dict(t=float(t),corner=corner,kind=kind,section=section,x=float(pos[0]),y=float(pos[1]),
                    heading=event['heading'],left=event['left'],right=event['right'],source=source,
                    video_estimate=original.tolist(),correction_mm=(pos-original).tolist())
    anchors.append(anchor(start,0,dict(heading=0,**initial_ranges(text)),'initial',0))
    previous=start
    for k,(entry,exit) in enumerate(zip(seq['corners'],seq['turns'])):
        middle=np.where((ts>previous+.5)&(hs>k*90+35))[0]
        if not len(middle):raise ValueError('Missing filmed turn midpoint')
        a=np.where((ts>max(previous+.3,ts[middle[0]]-2))&(hs>k*90+8))[0]
        b=np.where((ts>ts[middle[0]])&(hs>k*90+87))[0]
        if not len(a) or not len(b):raise ValueError('Missing filmed turn')
        t0,t1=ts[a[0]],ts[b[0]]
        anchors.append(anchor(t0,k%4,entry,'entry',k+1))
        anchors.append(anchor(t1,(k+1)%4,exit,'exit',k+1));previous=t1
    anchors.append(anchor(stop,0,seq['stop'],'stop',12))
    ats=np.array([a['t'] for a in anchors]);correction=np.array([a['correction_mm'] for a in anchors])
    # Spread each correction smoothly across neighbouring filmed samples,
    # retaining filmed progress and turn timing, without measurement holds.
    u=np.clip(np.searchsorted(ats,ts)-1,0,len(ats)-2)
    fraction=np.clip((ts-ats[u])/(ats[u+1]-ats[u]),0,1)
    blend=(1-np.cos(np.pi*fraction))/2
    adjustment=correction[u]*(1-blend[:,None])+correction[u+1]*blend[:,None]
    fitted=xy+adjustment
    offsets=np.array([a['heading']-np.interp(a['t'],ts,hs) for a in anchors])
    heading=hs+np.interp(ts,ats,offsets)
    # Explicit stationary film intervals prevent centroid jitter from becoming motion.
    for side,t in ((ts<start,start),(ts>stop,stop)):
        for a in (0,1):fitted[side,a]=np.interp(t,ts,fitted[:,a])
        heading[side]=np.interp(t,ts,heading)
    for a in anchors:
        a['sensors']={}
        for side in ('left','right'):
            hit=wall_hit(a['x'],a['y'],a['heading'],side);v=a[side]['filtered']
            a['sensors'][side]=dict(logged=v,modeled=hit['distance'],residual=hit['distance']-v if valid_range(v) else None,**hit)
        if a['source']=='left':assert abs(a['sensors']['left']['residual'])<.01
    return ts,fitted,heading,anchors,start,stop


def render(video,track_path):
    cv2.setNumThreads(1)
    data=json.loads(track_path.read_text());text=SOURCE.read_text(encoding='utf-8')
    data=data.get('track_calibration',data)
    assert sha256(SOURCE)==sha256(ARCHIVE)
    assert video.name==data['video']
    video_digest=sha256(video)
    if data.get('source_video_sha256') is not None:assert video_digest==data['source_video_sha256']
    ts,xy,hs,anchors,start,stop=align(data,text)
    OUT.mkdir(parents=True,exist_ok=True)
    fps=data['fps'];count=round(data['duration']*fps)
    cap=cv2.VideoCapture(str(video));assert cap.isOpened()
    scenes={};writers={};paths={}
    for field in (False,True):
        im=Image.new('RGB',(1080,1080) if field else (1920,1080),'#f5f4ee' if field else '#101d30');d=ImageDraw.Draw(im)
        view=View((0,0,1080,1080) if field else (35,160,940,1065),(-1580,1580,-1580,1580))
        d.rectangle(view.rect,fill='#f5f4ee');view.box(d,-1500,-1500,3000,3000,outline='#344357',width=4)
        view.box(d,-500,-900,1000,1800,fill='#dfe3e6',outline='#536273',width=4)
        if not field:
            d.text((35,35),'OPEN CHALLENGE / DJI OVERHEAD RUN',font=font(34,True),fill='white')
            d.text((35,95),'Source-video clock / video path with ToF pose correction',font=font(24),fill='#b8c9db')
            for j,line in enumerate(['Orange LEFT / blue RIGHT ToF','Cross = logged range endpoint','Red extension = residual to modeled wall','Dashed = out of range; no detection','1000 x 1800 mm middle enclosure','Video-derived XY; sensor timing estimated']):
                d.text((985,800+j*35),line,font=font(22),fill='#b8c9db')
        scenes[field]=(im,view)
        for speed in (1,4):
            path=OUT/f'open_dji_{"field_only" if field else "video_panel"}_{speed}x.mp4'
            pending=path.with_suffix('.rendering.mp4')
            w=cv2.VideoWriter(str(pending),cv2.VideoWriter_fourcc(*'mp4v'),fps,im.size);assert w.isOpened()
            writers[field,speed]=w;paths[field,speed]=(pending,path)
    trails={m:Image.new('RGBA',im.size) for m,(im,v) in scenes.items()};previous=None;ats=np.array([a['t'] for a in anchors])
    clear_failures=0
    try:
        for index in range(count):
            ok,source=cap.read()
            if not ok:raise ValueError(f'Missing video frame {index}')
            if index%2 and index!=count-1:continue
            t=index/fps;p=Pose(round(t*1000),*[float(np.interp(t,ts,xy[:,a])) for a in (0,1)],float(np.interp(t,ts,hs)),'Course driving','DJI track + ToF')
            if not chassis_clear(np.array([p.x,p.y]),p.h):clear_failures+=1
            ai=int(np.argmin(abs(ats-t)));a=anchors[ai];active=abs(t-a['t'])<=.3
            for field,(base,v) in scenes.items():
                if previous:ImageDraw.Draw(trails[field]).line([v.pt(*previous),v.pt(p.x,p.y)],fill='#10ad9a',width=4)
                im=base.copy();im.paste(trails[field],(0,0),trails[field]);d=ImageDraw.Draw(im)
                if active:
                    # The rays always originate on the currently displayed robot.
                    r=Reading(index,1,a['corner'],p.x,p.y,p.h,a['left'],a['right'],a['kind']);draw_reading(d,v,r)
                    for side in ('left','right'):
                        value=a[side]['filtered']
                        if valid_range(value):
                            _,end,_=ray(p.x,p.y,p.h,side,value);hit=wall_hit(p.x,p.y,p.h,side)
                            if abs(hit['distance']-value)>.5:d.line([v.pt(*end),v.pt(*hit['point'])],fill='#e14b60',width=3)
                robot(d,v,p,'#10ad9a')
                if not field:
                    inset=Image.fromarray(cv2.cvtColor(source,cv2.COLOR_BGR2RGB)).resize((900,506));im.paste(inset,(980,160));d=ImageDraw.Draw(im)
                    d.text((985,690),f'Video time {t:06.2f} s / {data["duration"]:.2f} s',font=font(24,True),fill='white')
                    desc=f"Snapshot {ai+1}/26: {a['kind']} / L {a['left']['filtered']:.0f} / R {a['right']['filtered']:.0f} mm" if active else 'Between logged snapshots: no readings synthesized'
                    d.text((985,740),desc,font=font(22),fill='#b8c9db')
                pixels=cv2.cvtColor(np.asarray(im),cv2.COLOR_RGB2BGR)
                for speed in (1,4):
                    if speed==4 and index%4:continue
                    encoded=pixels.copy()
                    if not field:cv2.putText(encoded,f'{speed}x playback',(1640,110),cv2.FONT_HERSHEY_SIMPLEX,.85,(215,220,230),2,cv2.LINE_AA)
                    for _ in range(min(2,count-index) if speed==1 else 1):writers[field,speed].write(encoded)
                if index==count-1:im.save(OUT/f'preview_{"field_only" if field else "video_panel"}.png')
            previous=(p.x,p.y)
            if index%600==0:print(f'DJI render {t:.1f}/{data["duration"]:.2f}s',flush=True)
    finally:
        cap.release()
        for w in writers.values():w.release()
    exports=[]
    for (field,speed),(pending,path) in paths.items():
        c=cv2.VideoCapture(str(pending));n=int(c.get(cv2.CAP_PROP_FRAME_COUNT));rate=c.get(cv2.CAP_PROP_FPS)
        for frame in (0,n//4,n//2,3*n//4,n-1):
            c.set(cv2.CAP_PROP_POS_FRAMES,frame);ok,image=c.read();assert ok
        c.release();assert abs(n/rate-data['duration']/speed)<.1
        pending.replace(path);exports.append(dict(file=path.name,seconds=n/rate,frames=n,fps=rate))
    result=dict(source_video=video.name,source_video_sha256=video_digest,log_sha256=sha256(ARCHIVE),
                timing='Source video clock; no holds or time warping. Untimed sensor events approximately aligned to filmed turns.',
                track_calibration=data,anchors=anchors,start_seconds=start,stop_seconds=stop,
                pose='Tracked chassis centroid minus42.5mm forward offset, then smooth ToF lateral corrections',
                chassis_clearance_failures=clear_failures,exports=exports)
    (OUT/'alignment.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(dict(exports=exports,clearance_failures=clear_failures,start=start,stop=stop)),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--video',type=Path,required=True);p.add_argument('--track',type=Path,required=True);args=p.parse_args();render(args.video,args.track)
