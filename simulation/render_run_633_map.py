"""Render only the run633 map with selective sensor graphics and an aging trail."""
import argparse
from bisect import bisect_left
import hashlib
import json
import math

import cv2
import numpy as np
from PIL import Image,ImageDraw

from render_run_633_drone import Renderer, DEST, GREEN, FPS
from run_633_reconstruction import latest, project_ray, rectangles, surface_hit
from run_633_log_timed import LogTimedMotion
from run_633_video_alignment import ARCHIVE, OUTPUT
from visualize_robot_run import seat_position

MAP_DEST = DEST/'map-only'
# Recorded lap2->lap3 boundary interval. Alpha(age)=1/(1+age/period).
FADE_PERIOD = (112940-92108)/1000


class MapRenderer(Renderer):
    ray_font_size = 42

    def __init__(self):
        self.clock_calibration = json.loads((OUTPUT/'clock_calibration.json').read_text())
        self.playback_scale = self.clock_calibration['seconds_per_log_second']
        super().__init__(LogTimedMotion(self.clock_calibration))
        self.playback_duration = self.motion.duration*self.playback_scale
        # The overhead view shows the top green pillar at seat 14. The robot
        # logged it at seat 13. Keep that identification in the source data.
        self.pillars = []
        for seat,color,x,y in self.motion.log['pillars']:
            if (seat,color)==(13,'GREEN'):
                seat = 14
                x,y = seat_position(seat,self.motion.log['turn'])
            self.pillars.append((seat,color,x,y))
        self.scene = rectangles(self.pillars)
        self.cameras = [r for r in self.cameras if r['kind']=='used']
        for camera in self.cameras:
            if camera['seat']==13 and camera['source']=='GREEN SEAT PRIMARY':
                camera['display_seat'] = 14
                camera['target'] = list(seat_position(14,self.motion.log['turn']))
        self.selected_readings = {side:[] for side in self.readings}
        for side,records in self.readings.items():
            for original in records:
                record = dict(original)
                if record['source']=='FINAL_PARK_TRACE':
                    if record.get('valid') and record.get('age_ms',999)<=80:
                        if record['state'] in (3,4,5,6,7):
                            record['kind'] = 'control'
                            record['label'] = 'Parking wall / marker scan'
                        elif record['state']==11 and record['raw']<190:
                            record['kind'] = 'proximity'
                if record['kind'] in ('used','control','interval','proximity') and record.get('valid') is not False:
                    self.selected_readings[side].append(record)

    def draw_possible_pillars(self,draw,view):
        for seat in range(24):
            x,y = seat_position(seat,self.motion.log['turn'])
            view.box(draw,x-25,y-25,50,50,fill='#eeefeb',outline='#d4d8d2',width=1)

    def draw_camera(self,draw,view,pose,camera):
        if camera['target'] is None:
            return
        angle = math.radians(pose.h)
        origin = np.array([pose.x+125*math.cos(angle),pose.y+125*math.sin(angle)])
        target = np.array(camera['target'])
        direction = target-origin
        length = float(np.linalg.norm(direction))
        if length<1:
            return
        right = math.atan2(164.4-320,248.9)
        left = math.atan2(164.4,248.9)
        depth = max(450,length*1.1)
        boundary = []
        for offset in np.linspace(right,left,19):
            ray = np.array([math.cos(angle+offset),math.sin(angle+offset)])
            hit = surface_hit(origin,ray,self.scene)
            distance = min(depth,hit['distance']) if hit else depth
            boundary.append(view.pt(*(origin+ray*distance)))
        # The angular limits follow the calibrated camera. The illustrated
        # depth follows the accepted target, clipped by modeled occluders.
        draw.polygon([view.pt(*origin),*boundary],fill=(206,100,214,18))
        for endpoint in (boundary[0],boundary[-1]):
            draw.line([view.pt(*origin),endpoint],fill=(206,100,214,190),width=2)
        draw.line([view.pt(*origin),view.pt(*target)],fill=(206,100,214,240),width=3)
        tip = view.pt(*target)
        draw.line((tip[0]-5,tip[1]-5,tip[0]+5,tip[1]+5),fill='#ce64d6',width=2)
        draw.line((tip[0]-5,tip[1]+5,tip[0]+5,tip[1]-5),fill='#ce64d6',width=2)

    def draw_trail(self,draw,view,time,pose,image=None):
        layer = Image.new('RGBA',image.size)
        draw = ImageDraw.Draw(layer)
        count = bisect_left(self.route_times,time)
        previous = None
        for index,p in enumerate(self.route[:count]):
            point = view.pt(p.x,p.y)
            if previous is not None:
                age = max(0,time-float(self.route_times[index]))
                opacity = round(255/(1+age/FADE_PERIOD))
                draw.line([previous,point],fill=(16,173,154,opacity),width=3)
            previous = point
        if previous:
            draw.line([previous,view.pt(pose.x,pose.y)],fill=GREEN,width=3)
        image.paste(layer,(0,0),layer)

    def sensors_at(self,time,pose):
        active = {side:record for side,records in self.selected_readings.items()
                  if (record:=latest(records,time,.4)) is not None}
        if 'right' in active and active['right']['kind']=='proximity':
            hit = project_ray(pose,'right',self.scene)
            if hit is None or 'rail' not in hit['target']:
                del active['right']
        if self.motion.rear_reference_window(time,pose,self.scene):
            active['rear'] = dict(t=time,side='rear',kind='interval',raw=None,timing='estimated',
                                  source='Untimed accepted rear corrections / filmed reference window')
        return active

    def ray_label(self,side,hit,record=None):
        return f'{hit["distance"]:.0f} mm'

    def map_frame(self,time):
        return self.viewport(time,(1080,1080),self.field_bounds)[0]

    def preview_map(self):
        MAP_DEST.mkdir(parents=True,exist_ok=True)
        times = (.5,14,37.9,43.3,58.97,69.1,125,145,153,170,184,190.3)
        sheet = Image.new('RGB',(1440,1920),'white')
        for index,time in enumerate(times):
            image = self.map_frame(time)
            sheet.paste(image.resize((480,480)),((index%3)*480,(index//3)*480))
        sheet.save(MAP_DEST/'review.jpg')
        preview_time = self.cameras[-1]['t']+.1 if self.cameras else 104
        self.map_frame(preview_time).save(MAP_DEST/'preview.png')

    def export_map(self):
        self.verify_geometry()
        self.preview_map()
        count = math.ceil(self.playback_duration*FPS)
        target = MAP_DEST/'run_633_map_video_synced_large_text_1x.mp4'
        partial = target.with_name(target.stem+'.rendering.mp4')
        writer = cv2.VideoWriter(str(partial),cv2.VideoWriter_fourcc(*'mp4v'),FPS,(1080,1080))
        if not writer.isOpened():
            raise RuntimeError('Cannot open map video encoder')
        try:
            for frame in range(0,count,2):
                image = self.map_frame(frame/FPS/self.playback_scale)
                encoded = cv2.cvtColor(np.asarray(image),cv2.COLOR_RGB2BGR)
                for _ in range(min(2,count-frame)):
                    writer.write(encoded)
                if frame%900==0:
                    print(f'Map render {frame/FPS:.0f}/{self.playback_duration:.1f}s',flush=True)
        finally:
            writer.release()
        self.finalize_map(partial,target,count)

    def finalize_map(self,partial,target,count):
        capture = cv2.VideoCapture(str(partial))
        frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        assert frames==count
        for frame in (0,frames//2,frames-1):
            capture.set(cv2.CAP_PROP_POS_FRAMES,frame)
            ok,image = capture.read()
            assert ok and image.shape[:2]==(1080,1080)
        capture.release()
        partial.replace(target)
        manifest = dict(run=633,source=ARCHIVE.relative_to(ARCHIVE.parents[3]).as_posix(),
                        sha256=hashlib.sha256(ARCHIVE.read_bytes()).hexdigest(),
                        track='local_workspace/run-633-drone/video_track_measurement.json',
                        fade=dict(formula='1/(1+age_seconds/period_seconds)',period_seconds=FADE_PERIOD),
                        rendered_anchors=len(self.motion.samples),
                        timed_field_poses_preserved=self.motion.original_pose_count,
                        clock_calibration=self.clock_calibration,
                        parking_policy='Logged coordinates; uniform calibrated log-to-video clock; footage in pose gaps',
                        logged_pillars=self.motion.log['pillars'],displayed_pillars=self.pillars,
                        camera_display='Calibrated side limits during accepted scan events, straight line to accepted target',
                        tof_display='Distance-only labels; geometric projection and timing uncertainty retained in metadata',
                        distance_label_font_pixels=self.ray_font_size,
                        gap_video_clocks=self.motion.gap_clocks,
                        mapped_initial_local_poses=self.motion.mapped_initial_count,
                        final_reverse=dict(logged_distance_mm=self.motion.final_reverse_distance,
                                           filmed_distance_mm=self.motion.final_reverse_observed_mm,
                                           estimated_start_seconds=self.motion.final_reverse_start*self.playback_scale,
                                           matched_film_seconds=[self.motion.offset+t*self.playback_scale
                                                                for t in self.motion.final_reverse_film_times],
                                           finish_seconds=self.playback_duration),
                        sensors=self.selected_readings,cameras=self.cameras,
                        export=dict(file=target.name,frames=frames,fps=FPS,seconds=frames/FPS))
        (MAP_DEST/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        (MAP_DEST/'README.md').write_text(f'''# Run 633 map-only revision

[Original-speed map video]({target.name})

![Preview](preview.png)

1080 x 1080, silent MP4, 30 fps. Distance-label text is {self.ray_font_size}px bold, twice the previous 21px size. Playback lasts {self.playback_duration:.3f} seconds. Each logged second maps to {self.playback_scale:.9f} seconds of filmed time. All {self.motion.original_pose_count} timed field poses retain their recorded coordinates. Positions and sensor events share the same uniform clock conversion. Initial local rear-positioning samples are transformed into the field frame using the logged field-start rebase.

## Footage synchronization

Align the first frame with DJI0066 at 4:39.788. Append DJI0067 from its first frame. The source-file boundary occurs about {(self.motion.alignment['seam_video_seconds']-self.motion.offset):.3f} seconds into this simulation. The simulated finish corresponds to DJI0067 at {(self.motion.offset+self.playback_duration-self.motion.alignment['seam_video_seconds']):.3f} seconds, approximately 2:31.13.

Five corresponding events establish the clock conversion. These are the initial unparking heading peak, three positive-X bottom course crossings and the final settled reverse. A single uniform scale fits those events with a maximum residual of about 0.14 seconds. Separate departure and parking stop checks have residuals up to about 0.61 seconds. Tracked event times, stop windows and spatial registration carry estimation error. The calibrated replay is about 3.45 percent longer than the 190.338-second log timeline. See manifest.json for every matched event, residual and conversion parameter.

Intervals up to 1.5 seconds interpolate between logged poses. Longer pose gaps follow the tracked footage, fitted to the surrounding log anchors. The tracker uses the minimum enclosing chassis rectangle for heading, with its axis resolved by continuous silhouette orientation. The timing inside each gap is estimated.

## Parking reconstruction

Timed parking poses and ToF events use the same calibrated clock as the course. The final straight reverse has a recorded distance of {self.motion.final_reverse_distance:.1f} mm and a logged final pose. The footage shows about {self.motion.final_reverse_observed_mm:.1f} mm of motion. Its pacing places the reverse at approximately {self.motion.final_reverse_start*self.playback_scale:.3f} seconds within the untimed parking tail, ending at the calibrated finish.

Trail opacity decreases continuously with age according to 1/(1 + log-age/20.832 seconds). This gives half-strength after about {20.832*self.playback_scale:.2f} seconds of playback and one-third after twice that interval.

All 24 firmware candidate pillar positions are shown faintly. The solid top green pillar is drawn at the outer middle seat 14, as seen in the rectified overhead footage. The log assigned it to seat 13. Its camera target and map geometry use seat 14, with the original assignment retained in metadata. Other solid pillars use their logged seats. Camera graphics appear during accepted scan observations, including accepted clear-seat evaluations. Two side lines show the calibrated angular field of view from the camera aperture, about 33.4 degrees left and 32.0 degrees right of the robot heading. The faint cone is clipped at modeled walls, rails and pillars. Its illustrated depth follows the target range. A separate straight line terminates at the displayed pillar or clear-seat center.

ToF rays appear for explicit use, documented control roles, marker scanning, relevant close-rail observations and correction intervals. General cached activity is omitted. Labels show only the distance in millimetres, as requested. The displayed values remain geometric intersections with modeled surfaces. Three accepted closing rear-reference corrections are logged without timestamps or individual ranges. Their dashed rear rays appear during the eligible near-straight low-speed filmed reference window before the last reverse. Those distances and times remain estimates, documented here and in metadata. Normal event persistence is 0.4 seconds for ToF and 0.65 seconds for camera. Raw readings remain in metadata and original evidence.

Recreate with `python simulation/refine_run_633_tracking.py`, `python simulation/calibrate_run_633_clock.py` and `python simulation/render_run_633_map.py`. Use `--preview` on the renderer for map review without video exports.
''',encoding='utf-8')
        print('Map-only export finished and verified.',flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview',action='store_true')
    args = parser.parse_args()
    cv2.setNumThreads(2)
    renderer = MapRenderer()
    if args.preview:
        renderer.verify_geometry()
        renderer.preview_map()
    else:
        renderer.export_map()


if __name__=='__main__':
    main()
