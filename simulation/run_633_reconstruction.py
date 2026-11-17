"""Logged-pose replay with drone-shaped gaps and projected sensor geometry."""
from bisect import bisect_right
from pathlib import Path
import json
import math

import numpy as np

from analyze_parking_exit_pose import fields, wrap180
from render_video_runs import ROOT, Pose, parse
from render_sensor_usage import obstacle_events
from visualize_robot_run import seat_position
from run_633_video_alignment import ARCHIVE, OUTPUT

SIDE_COLORS = {'left':'#efa12c', 'right':'#438be0', 'rear':'#af77e6'}


class Motion:
    def __init__(self, track_name='video_track.json', align_gaps=False, *, include_scan_poses=False,
                 clock_calibration=None):
        self.log = parse(ARCHIVE, include_scan_poses=include_scan_poses)
        self.alignment = json.loads((OUTPUT / 'alignment.json').read_text())
        self.offset = self.alignment['offset_seconds']
        self.clock_scale = 1.
        if clock_calibration:
            self.offset = clock_calibration['film_start_seconds']
            self.clock_scale = clock_calibration['seconds_per_log_second']
        self.duration = (self.log['finish']-self.log['start'])/1000
        self.samples = list(self.log['samples'])
        self.original_pose_count = len(self.samples)
        self.last_timed_seconds = (self.samples[-1].t-self.log['start'])/1000
        result = fields(self.log['result'])['pose_x_y_heading']
        self.samples.append(Pose(self.log['finish'], *map(float,result.split('/')),
                                 'Final parking', 'FINAL_RESULT_AT_RUN_END'))
        self.mapped_initial_count = 0
        if align_gaps:
            self.add_initial_rear_poses()
        self.times = np.array([(p.t-self.log['start'])/1000 for p in self.samples])
        data = json.loads((OUTPUT / track_name).read_text())['track']
        self.video_times = np.array([p['t'] for p in data])
        self.video_heading = np.array([p['body_heading'] for p in data])
        self.video_xy = np.array([[p['x'],p['y']] for p in data])
        self.video_xy -= 42.5*np.column_stack([np.cos(np.deg2rad(self.video_heading)),
                                             np.sin(np.deg2rad(self.video_heading))])
        self.gap_clocks = {}
        self.initial_clock_shift = 0.
        if align_gaps:
            self.initial_clock_shift = self.match_clock(self.samples[0],self.times[0])-self.times[0]
            for index,(start,finish) in enumerate(zip(self.times,self.times[1:])):
                if finish-start<=1.5:
                    continue
                left = self.match_clock(self.samples[index],start)
                right = self.match_clock(self.samples[index+1],finish)
                if self.samples[index+1].source=='FINAL_RESULT_AT_RUN_END':
                    right = finish
                if right>left+.1:
                    self.gap_clocks[index] = (left,right)

    def match_clock(self,pose,time):
        candidates = np.arange(time-3,time+3.001,.025)
        costs = []
        for candidate in candidates:
            xy,h = self.filmed(float(candidate))
            costs.append(np.linalg.norm(xy-[pose.x,pose.y])
                         +1.5*abs(wrap180(h-pose.h))+15*abs(candidate-time))
        return float(candidates[np.argmin(costs)])

    def add_initial_rear_poses(self):
        first = self.samples[0]
        local = []
        for line in ARCHIVE.read_text().splitlines():
            data = fields(line)
            if (line.startswith('[PARK_DIAG]') and data.get('type')=='sample'
                    and data.get('state','').startswith('rear_') and int(data['t'])<first.t):
                local.append((int(data['t']),*map(float,data['pose'].split(','))))
        if not local:
            return
        # The field-start rebase fixes the rigid transform of the preceding
        # local rear-positioning frame. Retain its recorded translation/heading.
        _,last_x,last_y,last_h = local[-1]
        rotation = math.radians(wrap180(first.h-last_h))
        mapped = []
        for time,x,y,h in local:
            dx,dy = x-last_x,y-last_y
            mapped.append(Pose(time,first.x+dx*math.cos(rotation)-dy*math.sin(rotation),
                               first.y+dx*math.sin(rotation)+dy*math.cos(rotation),
                               first.h+wrap180(h-last_h),'Parking exit','MAPPED_REAR_LOCAL_POSE'))
        self.mapped_initial_count = len(mapped)
        self.samples = mapped+self.samples

    def filmed(self, time):
        clock = time*self.clock_scale+self.offset
        xy = np.array([np.interp(clock,self.video_times,self.video_xy[:,axis]) for axis in (0,1)])
        h = float(np.interp(clock,self.video_times,self.video_heading))
        return xy,h

    def at(self, time):
        index = bisect_right(self.times,time)-1
        if index<0:
            a = self.samples[0]
            xy,h = self.filmed(time+self.initial_clock_shift)
            anchor_xy,anchor_h = self.filmed(self.times[0]+self.initial_clock_shift)
            xy += np.array([a.x,a.y])-anchor_xy
            h += wrap180(a.h-anchor_h)
            return Pose(round(time*1000),*xy,h,'Parking exit','Drone gap fill'),True
        if index>=len(self.samples)-1:
            p = self.samples[-1]
            return Pose(round(time*1000),p.x,p.y,p.h,p.phase,p.source),True
        a,b = self.samples[index:index+2]
        start,finish = self.times[index:index+2]
        if time==start:
            return Pose(round(time*1000),a.x,a.y,a.h,a.phase,a.source),False
        fraction = (time-start)/(finish-start)
        drone_gap = finish-start>1.5 or b.source=='FINAL_RESULT_AT_RUN_END'
        if drone_gap:
            video_start,video_finish = self.gap_clocks.get(index,(start,finish))
            video_time = video_start+fraction*(video_finish-video_start)
            xy,h = self.filmed(video_time)
            left,left_h = self.filmed(video_start)
            right,right_h = self.filmed(video_finish)
            blend = (1-math.cos(math.pi*fraction))/2
            xy += (1-blend)*(np.array([a.x,a.y])-left)+blend*(np.array([b.x,b.y])-right)
            left_error = wrap180(a.h-left_h)
            right_error = wrap180(b.h-right_h)
            h += left_error+blend*wrap180(right_error-left_error)
            if self.gap_clocks and b.source=='FINAL_RESULT_AT_RUN_END':
                xy = self.keep_parking_body_inside(xy,h)
            return Pose(round(time*1000),*xy,h,a.phase,'Drone gap fill'),True
        p = Pose(round(time*1000),a.x+fraction*(b.x-a.x),a.y+fraction*(b.y-a.y),
                 a.h+fraction*wrap180(b.h-a.h),a.phase,a.source)
        return p,False

    @staticmethod
    def keep_parking_body_inside(xy,heading):
        # Endpoint shifts of the filmed path must not push the unlogged chassis
        # through the bay rails. Logged endpoints themselves remain untouched.
        angle = math.radians(heading)
        corners = np.array([[x*math.cos(angle)-y*math.sin(angle),
                             x*math.sin(angle)+y*math.cos(angle)]
                            for x,y in ((-40,-61.5),(125,-61.5),(125,61.5),(-40,61.5))])
        body = corners+xy
        if body[:,1].min()<-1300 and body[:,1].max()>-1500:
            xy[0] = np.clip(xy[0],234.5-corners[:,0].min(),478-corners[:,0].max())
            xy[1] = max(xy[1],-1498-corners[:,1].min())
        return xy


def sensor_origin(pose, side):
    h = math.radians(pose.h)
    if side == 'rear':
        along,lateral,angle = -25.,0.,h+math.pi
    else:
        sign = 1 if side=='left' else -1
        along,lateral,angle = 40.,sign*35.,h+sign*math.pi/2
    origin = (pose.x+along*math.cos(h)-lateral*math.sin(h),
              pose.y+along*math.sin(h)+lateral*math.cos(h))
    return np.array(origin),np.array([math.cos(angle),math.sin(angle)])


def rectangles(pillars):
    result = [('outer wall',(-1500,1500,-1500,1500)),
              ('inner wall',(-500,500,-500,500)),
              ('rear parking rail',(212.5,232.5,-1500,-1300)),
              ('front parking rail',(480,500,-1500,-1300))]
    result += [(f'{color.lower()} pillar {seat}',(x-25,x+25,y-25,y+25))
               for seat,color,x,y in pillars]
    return result


def project_ray(pose, side, scene):
    origin,direction = sensor_origin(pose,side)
    return surface_hit(origin,direction,scene)


def surface_hit(origin,direction,scene):
    hits = []
    for name,(xmin,xmax,ymin,ymax) in scene:
        for axis,value,lower,upper in ((0,xmin,ymin,ymax),(0,xmax,ymin,ymax),
                                       (1,ymin,xmin,xmax),(1,ymax,xmin,xmax)):
            if abs(direction[axis])<1e-10:
                continue
            distance = (value-origin[axis])/direction[axis]
            point = origin+direction*distance
            if distance>.01 and lower-1e-6<=point[1-axis]<=upper+1e-6:
                hits.append(dict(distance=float(distance), endpoint=point.tolist(),target=name))
    if not hits:
        return None
    hit = min(hits,key=lambda p:p['distance'])
    hit['origin'] = origin.tolist()
    return hit


def sensor_records(motion):
    readings = {side:[] for side in SIDE_COLORS}
    cameras = []
    lines = ARCHIVE.read_text().splitlines()
    seen = set()
    previous_counter = None
    for number,line in enumerate(lines,1):
        data = fields(line)
        if 't' not in data:
            continue
        time = (int(data['t'])-motion.log['start'])/1000
        if line.startswith('[PARK_DIAG]') and data.get('type')=='sample':
            state = data.get('state','')
            for index,side in enumerate(SIDE_COLORS):
                values = data.get(f's{index}','none').split(',')
                if len(values)!=8:
                    continue
                key = (side,values[0])
                if key in seen:
                    continue
                seen.add(key)
                valid = values[6]=='1'
                use = valid and ((side=='rear' and 'rear_drive' in state)
                                 or (side=='right' and state.startswith('localize_')))
                readings[side].append(dict(t=time,side=side,kind='control' if use else 'activity',
                                           raw=float(values[2]),filtered=float(values[3]),
                                           age_ms=int(values[1]),valid=valid,label=state.replace('_',' '),
                                           line=number,source='PARK_DIAG'))
        elif line.startswith('[PARK_DIAG]') and data.get('type')=='correction':
            readings['right'].append(dict(t=time,side='right',kind='used',raw=None,
                                          label='Wall / marker pose correction',line=number,source='PARK_DIAG correction'))
        elif line.startswith('[LATER_TRACK]'):
            for side,used in zip(('left','right'),data.get('tof_used','0,0').split(',')):
                if used=='1':
                    readings[side].append(dict(t=time,side=side,kind='used',raw=None,
                                               label='ToF position update',line=number,source='LATER_TRACK use flag'))
        elif line.startswith('[FINAL_PARK_TRACE]'):
            sequence,age,raw,expected = data['tof_seq/age/raw/expected'].split('/')
            counter = int(data.get('cs','0'))
            updated = previous_counter is not None and counter>previous_counter
            previous_counter = counter
            readings['right'].append(dict(t=time,side='right',kind='interval' if updated else 'activity',
                                          raw=float(raw),expected=float(expected),age_ms=int(age),
                                          state=int(data['state/phase'].split('/')[0]),
                                          phase=int(data['state/phase'].split('/')[1]),
                                          label='Parking wall reference',line=number,source='FINAL_PARK_TRACE',
                                          valid=0<float(raw)<9999))
    for event in obstacle_events(ARCHIVE):
        if event.sensor!='camera':
            continue
        data = fields(lines[event.line-1])
        target = None
        sighting = None
        if data.get('decision')=='confirmed' and 'sighting' in data:
            sighting = list(map(float,data['sighting'].split(',')))
        seat = int(data.get('accepted',data.get('seat','-1')))
        if event.kind=='used' and 0<=seat<24:
            target = list(seat_position(seat,motion.log['turn']))
        cameras.append(dict(t=(event.t-motion.log['start'])/1000,kind=event.kind,
                            label=event.label,source=event.source,line=event.line,target=target,
                            raw_sighting=sighting,seat=seat))
    for side in readings:
        readings[side].sort(key=lambda r:r['t'])
    return readings,cameras


def latest(records,time,window):
    index = bisect_right([record['t'] for record in records],time)-1
    if index<0 or time-records[index]['t']>window:
        return None
    return records[index]


def active_sensors(readings,time,pose,motion):
    result = {}
    for side,records in readings.items():
        record = latest(records,time,.55)
        if record:
            result[side] = record
    if time>motion.last_timed_seconds and pose.phase=='Final parking':
        result.setdefault('right',dict(t=time,side='right',kind='guide',raw=None,
                                       label='Parking wall projection guide',source='geometry guide'))
        if pose.y<-1280 and abs(wrap180(pose.h))<25:
            result.setdefault('rear',dict(t=time,side='rear',kind='guide',raw=None,
                                          label='Rear bay reference guide',source='untimed rear corrections / geometry guide'))
    return result
