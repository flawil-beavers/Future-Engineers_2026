"""Render run633 with original pose anchors, filmed gaps and sensor projections.

Run prepare_run_633_drone.py and run_633_video_alignment.py first.
Generated artifacts and cached footage stay under local_workspace.
"""
import argparse
from bisect import bisect_left
from functools import lru_cache
import hashlib
import json
import math

import cv2
import numpy as np
from PIL import Image, ImageDraw

from render_video_runs import ROOT, View, font, robot
from run_633_reconstruction import (
    Motion, SIDE_COLORS, active_sensors, latest, project_ray, rectangles,
    sensor_records,
)
from run_633_video_alignment import ARCHIVE, OUTPUT

DEST = ROOT / 'local_workspace/video-documentation/sensor-usage/run-633-drone'
BG = '#101c30'
INK = '#eff5fc'
MUTED = '#a6b8cf'
GREEN = '#10ad9a'
CAMERA = '#ce64d6'
font = lru_cache(maxsize=30)(font)
FPS = 30
KINDS = {'used':'Confirmed use', 'control':'Control role',
         'activity':'Logged activity', 'interval':'Correction interval',
         'guide':'Projection guide'}


def dashed(draw, a, b, color, width=3):
    a,b = np.array(a,dtype=float),np.array(b,dtype=float)
    length = float(np.linalg.norm(b-a))
    for start in np.arange(0,length,15):
        end = min(start+8,length)
        draw.line([tuple(a+(b-a)*start/length),tuple(a+(b-a)*end/length)],
                  fill=color,width=width)


class Renderer:
    ray_font_size = 21

    def __init__(self, motion=None):
        self.motion = motion or Motion()
        self.readings,self.cameras = sensor_records(self.motion)
        self.scene = rectangles(self.motion.log['pillars'])
        self.film = json.loads((OUTPUT/'film_frames.json').read_text())
        self.frame_times = [frame['time'] for frame in self.film['frames']]
        self.route_times = np.arange(0,self.motion.duration+.001,1/15)
        self.route = [self.motion.at(float(t))[0] for t in self.route_times]
        extent = max(1580,max(max(abs(p.x),abs(p.y))+150 for p in self.route))
        self.field_bounds = (-extent,extent,-extent,extent)
        self.bases = {}

    @lru_cache(maxsize=8)
    def film_image(self,index):
        return Image.open(OUTPUT/self.film['frames'][index]['image']).convert('RGB')

    def source_frame(self,time):
        value = time+self.motion.offset
        index = min(bisect_left(self.frame_times,value),len(self.frame_times)-1)
        if index and value-self.frame_times[index-1]<self.frame_times[index]-value:
            index -= 1
        return self.film_image(index),self.film['frames'][index]

    def base(self,size,bounds):
        key = (size,bounds)
        if key in self.bases:
            return self.bases[key].copy()
        image = Image.new('RGB',size,'#f5f4ee')
        draw = ImageDraw.Draw(image)
        view = View((0,0,*size),bounds)
        for value in range(-1500,1501,500):
            draw.line([view.pt(value,-1500),view.pt(value,1500)],fill='#e3e4df')
            draw.line([view.pt(-1500,value),view.pt(1500,value)],fill='#e3e4df')
        self.draw_possible_pillars(draw,view)
        for name,(xmin,xmax,ymin,ymax) in self.scene:
            fill = None
            outline = '#536273'
            if name=='inner wall':
                fill = '#dfe3e6'
            elif 'rail' in name:
                fill,outline = '#e7a2c4','#ab618c'
            elif 'pillar' in name:
                fill = '#d85454' if 'red' in name else '#27a46a'
            view.box(draw,xmin,ymin,xmax-xmin,ymax-ymin,fill=fill,outline=outline,width=3)
        self.bases[key] = image
        return image.copy()

    def draw_possible_pillars(self,draw,view):
        pass

    def draw_trail(self,draw,view,time,pose,image=None):
        index = bisect_left(self.route_times,time)
        points = [view.pt(p.x,p.y) for p in self.route[:index]]+[view.pt(pose.x,pose.y)]
        if len(points)>1:
            draw.line(points,fill=GREEN,width=3)

    def camera_at(self,time):
        return latest(self.cameras,time,.65)

    def sensors_at(self,time,pose):
        return active_sensors(self.readings,time,pose,self.motion)

    def ray_label(self,side,hit,record=None):
        return f'{side[0].upper()} {hit["distance"]:.0f} mm'

    def draw_camera(self,draw,view,pose,camera):
        h = math.radians(pose.h)
        origin = (pose.x+125*math.cos(h),pose.y+125*math.sin(h))
        rays = []
        for angle in (-32,33.4):
            direction = h+math.radians(angle)
            rays.append((origin[0]+420*math.cos(direction),origin[1]+420*math.sin(direction)))
        draw.line([view.pt(*rays[0]),view.pt(*origin),view.pt(*rays[1])],
                  fill=CAMERA,width=4 if camera['kind']=='used' else 2)
        if camera['kind']=='used' and camera['target']:
            draw.line([view.pt(*origin),view.pt(*camera['target'])],fill=CAMERA,width=5)

    def viewport(self,time,size,bounds,labels=True):
        image = self.base(size,bounds)
        draw = ImageDraw.Draw(image,'RGBA')
        view = View((0,0,*size),bounds)
        pose,filled = self.motion.at(time)
        self.draw_trail(draw,view,time,pose,image=image)
        camera = self.camera_at(time)
        if camera:
            self.draw_camera(draw,view,pose,camera)
        active = self.sensors_at(time,pose)
        measurements = []
        labels_to_draw = []
        for side,record in active.items():
            hit = project_ray(pose,side,self.scene)
            if hit is None:
                continue
            color = SIDE_COLORS[side]
            guide = record['kind']=='guide'
            invalid = record.get('valid') is False
            a,b = view.pt(*hit['origin']),view.pt(*hit['endpoint'])
            width = 5 if record['kind']=='used' else 4 if record['kind']=='control' else 2
            if guide:
                color = '#8794a6'
            if guide or invalid or record['kind']=='interval':
                dashed(draw,a,b,color,3)
            else:
                draw.line([a,b],fill=color,width=width)
            if not guide and not invalid:
                draw.line((b[0]-5,b[1]-5,b[0]+5,b[1]+5),fill=color,width=2)
                draw.line((b[0]-5,b[1]+5,b[0]+5,b[1]-5),fill=color,width=2)
            if labels and (bounds==self.field_bounds or hit['distance']<500):
                label = self.ray_label(side,hit,record)
                box = draw.textbbox((0,0),label,font=font(self.ray_font_size,True),anchor='lt')
                w = box[2]-box[0]+12
                height = box[3]-box[1]+12
                x = max(4,min(size[0]-w-4,(a[0]+b[0])//2+9))
                y = max(4,min(size[1]-height-4,(a[1]+b[1])//2-height))
                if side=='rear':
                    x = max(4,min(a[0],b[0])-w-15)
                    y = max(4,min(size[1]-height-4,min(a[1],b[1])-height))
                labels_to_draw.append((x,y,w,height,label,color))
            measurements.append((side,record,hit))
        robot(draw,view,pose,GREEN,front=125,rear=40,width=123)
        angle = math.radians(pose.h)
        body = [view.pt(pose.x+x*math.cos(angle)-y*math.sin(angle),
                        pose.y+x*math.sin(angle)+y*math.cos(angle))
                for x,y in ((-40,-61.5),(125,-61.5),(125,61.5),(-40,61.5))]
        body_left,body_right = min(p[0] for p in body)-6,max(p[0] for p in body)+6
        body_top,body_bottom = min(p[1] for p in body)-6,max(p[1] for p in body)+6
        placed_labels = []
        for x,y,w,height,label,color in labels_to_draw:
            if x<body_right and x+w>body_left and y<body_bottom and y+height>body_top:
                y = body_bottom+8
                if y>size[1]-height-4:
                    y = max(4,body_top-height-8)
            for previous_x,previous_y,previous_w,previous_height in placed_labels:
                if x<previous_x+previous_w and x+w>previous_x and y<previous_y+previous_height and y+height>previous_y:
                    y = previous_y+previous_height+8
                    if y>size[1]-height-4:
                        y = max(4,previous_y-height-8)
            draw.rounded_rectangle((x,y,x+w,y+height),5,fill=BG)
            draw.text((x+6,y+6),label,font=font(self.ray_font_size,True),fill=color,anchor='lt')
            placed_labels.append((x,y,w,height))
        return image,pose,filled,measurements,camera

    def frame(self,time):
        field,pose,filled,measurements,camera = self.viewport(time,(1000,1000),self.field_bounds)
        panel = Image.new('RGB',(1920,1080),BG)
        panel.paste(field,(0,80))
        draw = ImageDraw.Draw(panel)
        draw.text((30,15),'RUN 633  /  OBSTACLE CHALLENGE',font=font(32,True),fill=INK)
        draw.text((1030,22),f'{time:06.1f} s  /  {pose.phase.upper()}',font=font(27,True),fill=INK)
        draw.text((1030,74),'Drone gap fill' if filled else 'Logged pose anchors / interpolation',
                  font=font(21),fill='#efa12c' if filled else '#6de4d3')
        parking = pose.phase in ('Parking exit','Final parking')
        source,row = self.source_frame(time)
        if parking:
            zoom,*_ = self.viewport(time,(860,430),(-400,1000,-1530,-830))
            panel.paste(zoom,(1030,120))
            draw = ImageDraw.Draw(panel)
            draw.text((1045,132),'PARKING / UNPARKING DETAIL',font=font(20,True),fill=BG)
            panel.paste(source.resize((420,236),Image.Resampling.LANCZOS),(1030,600))
            table_x,table_y = 1480,600
            draw.text((1030,565),'SYNCHRONIZED DRONE FOOTAGE',font=font(20,True),fill=MUTED)
        else:
            panel.paste(source.resize((860,484),Image.Resampling.LANCZOS),(1030,120))
            table_x,table_y = 1030,670
        draw = ImageDraw.Draw(panel)
        draw.text((table_x,table_y),'ToF wall projection',font=font(24,True),fill=INK)
        for index,side in enumerate(SIDE_COLORS):
            match = next((m for m in measurements if m[0]==side),None)
            y = table_y+41+index*55
            value = f'{match[2]["distance"]:.0f} mm' if match else '--'
            draw.text((table_x,y),f'{side.upper()}   {value}',font=font(21,True),fill=SIDE_COLORS[side])
            if match:
                record = match[1]
                status = KINDS[record['kind']]
                if record.get('valid') is False:
                    status = 'Invalid range / projection'
                draw.text((table_x,y+25),status,font=font(17),fill=MUTED)
        y = 870 if parking else 910
        message = 'Camera observation accepted' if camera and camera['kind']=='used' else 'Camera evaluation logged' if camera else 'Camera / no recent logged event'
        draw.text((1030,y),message,font=font(22,True),fill=CAMERA if camera else MUTED)
        if parking:
            draw.text((1030,y+38),'Rear ToF references the bay rail; side ToF references the wall.',font=font(21),fill=INK)
        else:
            draw.text((1030,y+38),'Camera evaluates pillars; accepted sightings update the map.',font=font(21),fill=INK)
        draw.text((1030,976),'Thin ray = activity    Thick ray = confirmed use / control role',font=font(20),fill=MUTED)
        draw.text((1030,1006),'Dashed = interval / invalid    Grey dashed = untimed guide',font=font(20),fill=MUTED)
        draw.text((1030,1040),'Distances projected to modeled surfaces; raw ranges retained in metadata.',font=font(18),fill=MUTED)
        draw.text((1030,840 if parking else 620),
                  f'DJI {row["video"]:04d}   {row["source_seconds"]//60:.0f}:{row["source_seconds"]%60:05.2f}',
                  font=font(18),fill=MUTED)
        field_only = Image.new('RGB',(1080,1080),BG)
        field_only.paste(field.resize((1000,1000)),(40,40))
        fd = ImageDraw.Draw(field_only)
        fd.text((43,7),f'RUN 633   /   {time:06.1f} s',font=font(24,True),fill=INK)
        fd.text((43,1045),'ToF wall projections   L orange / R blue / rear purple   Camera magenta',font=font(22),fill=MUTED)
        return panel,field_only

    def verify_geometry(self):
        for sample,time in zip(self.motion.samples,self.motion.times):
            actual,_ = self.motion.at(float(time))
            assert abs(actual.x-sample.x)<1e-6 and abs(actual.y-sample.y)<1e-6
            assert abs((actual.h-sample.h+180)%360-180)<1e-6
        for time in self.route_times:
            pose,_ = self.motion.at(float(time))
            assert not(abs(pose.x)<500 and abs(pose.y)<500)
            for side in SIDE_COLORS:
                hit = project_ray(pose,side,self.scene)
                assert hit is not None and hit['distance']>0
                assert abs(np.linalg.norm(np.array(hit['endpoint'])-hit['origin'])-hit['distance'])<1e-6

    def preview(self):
        DEST.mkdir(parents=True,exist_ok=True)
        sheet = Image.new('RGB',(1920,1620),BG)
        for index,time in enumerate((.5,4.,14.,45.,90.,125.,170.,183.,189.)):
            panel,field = self.frame(time)
            sheet.paste(panel.resize((640,360)),((index%3)*640,(index//3)*540))
            # Enlarged parking detail beneath each thumbnail aids geometry review.
            sheet.paste(panel.crop((1030,120,1890,550)).resize((360,180)),
                        ((index%3)*640,(index//3)*540+360))
        sheet.save(DEST/'review.jpg')
        panel,field = self.frame(4.)
        panel.save(DEST/'preview_panel.png')
        field.save(DEST/'preview_field.png')

    def export(self):
        self.verify_geometry()
        self.preview()
        count = math.ceil(self.motion.duration*FPS)
        writers = {}
        targets = {}
        for name,size in (('panel',(1920,1080)),('field',(1080,1080))):
            for speed in (1,4):
                target = DEST/f'run_633_{name}_{speed}x.mp4'
                partial = target.with_name(target.stem+'.rendering.mp4')
                writer = cv2.VideoWriter(str(partial),cv2.VideoWriter_fourcc(*'mp4v'),FPS,size)
                if not writer.isOpened():
                    raise RuntimeError(f'Cannot open encoder for {partial.name}')
                writers[name,speed] = writer
                targets[name,speed] = (partial,target,size)
        try:
            for frame in range(0,count,2):
                images = self.frame(min(frame/FPS,self.motion.duration))
                for (name,speed),writer in writers.items():
                    if speed==4 and frame%4:
                        continue
                    image = images[0 if name=='panel' else 1].copy()
                    draw = ImageDraw.Draw(image)
                    draw.text((image.width-180,17),f'{speed}x playback',font=font(20),fill=MUTED)
                    encoded = cv2.cvtColor(np.asarray(image),cv2.COLOR_RGB2BGR)
                    for _ in range(min(2,count-frame) if speed==1 else 1):
                        writer.write(encoded)
                if frame%900==0:
                    print(f'Rendering run633 {frame/FPS:.0f}/{self.motion.duration:.1f}s',flush=True)
        finally:
            for writer in writers.values():
                writer.release()
        exports = []
        for (name,speed),(partial,target,size) in targets.items():
            capture = cv2.VideoCapture(str(partial))
            frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
            expected = count if speed==1 else math.ceil(count/4)
            assert frames==expected,(partial.name,frames,expected)
            for at in (0,frames//2,frames-1):
                capture.set(cv2.CAP_PROP_POS_FRAMES,at)
                ok,image = capture.read()
                assert ok and image.shape[:2]==(size[1],size[0])
            capture.release()
            partial.replace(target)
            exports.append(dict(file=target.name,speed=speed,frames=frames,fps=FPS,
                                seconds=frames/FPS,bytes=target.stat().st_size))
        manifest = dict(run=633,source=ARCHIVE.relative_to(ROOT).as_posix(),
                        sha256=hashlib.sha256(ARCHIVE.read_bytes()).hexdigest(),
                        alignment=self.motion.alignment,anchors_preserved=len(self.motion.samples),
                        last_timed_pose_seconds=self.motion.last_timed_seconds,
                        scene_rectangles=self.scene,
                        sensor_records=self.readings,camera_records=self.cameras,exports=exports)
        (DEST/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
        self.write_notes()
        print('Finished and verified all four run633 exports.',flush=True)

    def write_notes(self):
        text = '''# Run 633 with sensor rays and filmed gap reconstruction

| View | Original speed | Accelerated |
| --- | --- | --- |
| Field and synchronized footage, with parking detail | [1x](run_633_panel_1x.mp4) | [4x](run_633_panel_4x.mp4) |
| Field only | [1x](run_633_field_1x.mp4) | [4x](run_633_field_4x.mp4) |

![Preview](preview_panel.png)

The 1x clips cover the 190.338-second logged run at original speed. Alignment estimates RUN_START at 0066 4:42.85 and RUN_END at 0067 2:27.63. No additional holds are inserted. The recording boundary is at run 42.709 seconds. Video is silent at 30 fps, with motion sampled at 15 Hz. Four-times playback is explicitly labelled.

## Position reconstruction

All 574 original timed field poses are retained. The separate final result is assigned to RUN_END as an approximate 575th anchor. Between nearby anchors, position and shortest-angle heading are interpolated. Gaps longer than 1.5 seconds, the initial missing interval and the last 23.181 seconds use the tracked drone path. Smooth endpoint corrections make those paths meet the original anchors. Logged poses remain authoritative when they disagree with the footage. The median centered course-alignment discrepancy is about 115 mm and 8.5 degrees. Tracking, calibration and synchronization remain estimates.

The inner enclosure is 1000 x 1000 mm for this obstacle run. The outer field is 3000 x 3000 mm. Pillars use accepted logged map centers. Pink parking rails have inward faces at X 232.5 and 480 mm. With the first field pose these reproduce the logged right range of 101 mm and rear range of 62.3 mm. The physical body uses front/rear overhangs of 125/40 mm and width 123 mm. The displayed rear-axle position is the logged pose reference.

## Sensor display

At the user's request, every numeric distance drawn in the video is projected from the sensor aperture to the nearest modeled wall, parking rail or pillar. These adjusted values are labelled wall projections. Original raw/filtered ranges, ages, validity, timestamps and source lines are retained in manifest.json. Original evidence is unchanged.

Orange is left ToF, blue is right ToF and purple is rear ToF. Thin rays indicate recent logged activity. Thick rays indicate explicit use or the documented controller role, distinguished in the panel. Controller role alone does not establish per-reading acceptance. Dashed colored rays mark invalid readings or correction-counter intervals. Counter intervals establish that a correction occurred between samples. Grey dashed rays in the untimed parking tail are geometry guides. Individual final rear corrections have no timestamps and are not assigned invented measurement times.

Magenta camera boundaries use the calibrated approximately 65-degree horizontal field of view. Thick accepted observations with logged sighting coordinates connect to the target. Other camera evaluations show the field of view only. Sensor events persist for 0.55 seconds and camera events for 0.65 seconds for readability. Missing events represent gaps in sampled evidence.

## Reproduction

Prepare footage with simulation/prepare_run_633_drone.py, align with simulation/run_633_video_alignment.py and render with `python simulation/render_run_633_drone.py`. Cached frames and video tracking live in local_workspace/run-633-drone. A `--preview` option runs geometry checks and saves review images without exporting videos.
'''
        (DEST/'README.md').write_text(text,encoding='utf-8')
        for index in (DEST.parent/'README.md',DEST.parents[1]/'README.md'):
            existing = index.read_text(encoding='utf-8')
            relative = 'run-633-drone/README.md' if index.parent==DEST.parent else 'sensor-usage/run-633-drone/README.md'
            link = f'[Run 633 with filmed gap reconstruction, wall-reaching sensor rays and parking detail]({relative})'
            if relative not in existing:
                index.write_text(link+'\n\n'+existing,encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview',action='store_true')
    args = parser.parse_args()
    cv2.setNumThreads(2)
    renderer = Renderer()
    if args.preview:
        renderer.verify_geometry()
        renderer.preview()
        print('Geometry and pose anchors verified. Preview images saved.',flush=True)
    else:
        renderer.export()


if __name__=='__main__':
    main()
