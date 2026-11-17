"""Replay run 633 on the log clock with filmed motion only in pose gaps."""
import math

import numpy as np
from scipy.ndimage import gaussian_filter1d

from analyze_parking_exit_pose import fields, wrap180
from render_video_runs import Pose
from run_633_reconstruction import Motion, project_ray
from run_633_video_alignment import ARCHIVE


class LogTimedMotion(Motion):
    def __init__(self,clock_calibration=None):
        super().__init__('video_track_measurement.json', align_gaps=True, include_scan_poses=True,
                         clock_calibration=clock_calibration)
        tail_index = len(self.samples)-2
        gap_start, gap_end = self.times[tail_index:]
        film_start, _ = self.gap_clocks.get(tail_index, (gap_start, gap_end))
        film_times, progress, observed_distance = self.find_final_reverse()
        self.final_reverse_observed_mm = observed_distance

        # The reverse has a logged distance and end pose but no timestamp.
        # Place it inside the missing interval using the filmed timing.
        self.final_reverse_times = np.interp(film_times, (film_start, film_times[-1]), (gap_start, gap_end))
        self.final_reverse_start = round(float(self.final_reverse_times[0]), 3)
        self.final_reverse_times[0] = self.final_reverse_start
        self.final_reverse_progress = progress
        self.final_reverse_film_times = (float(film_times[0]), float(film_times[-1]))

        final_segment = next(
            fields(line) for line in reversed(ARCHIVE.read_text().splitlines())
            if line.startswith('[FINAL PARK SEGMENT]')
        )
        self.final_reverse_distance = float(final_segment['distance_mm'])
        self.final_pose = self.samples[-1]
        angle = math.radians(self.final_pose.h)
        direction = np.array([math.cos(angle), math.sin(angle)])
        self.final_reverse_start_xy = np.array([self.final_pose.x,self.final_pose.y])+self.final_reverse_distance*direction
        self.samples.insert(-1, Pose(
            self.log['start']+round(self.final_reverse_start*1000),
            *self.final_reverse_start_xy, self.final_pose.h, 'Final parking', 'FINAL_REVERSE_DISTANCE_ANCHOR',
        ))
        self.times = np.insert(self.times, -1, self.final_reverse_start)
        self.gap_clocks[tail_index] = (film_start, float(film_times[0]))

    def find_final_reverse(self):
        times = (self.video_times-self.offset)/self.clock_scale
        positions = gaussian_filter1d(self.video_xy,3,axis=0)
        velocity = np.gradient(positions,times,axis=0)
        heading = np.deg2rad(self.video_heading)
        signed = velocity[:,0]*np.cos(heading)+velocity[:,1]*np.sin(heading)
        wrapped = (self.video_heading+180)%360-180
        moving = np.flatnonzero((times>180)&(abs(wrapped)<12)&(signed<-12))
        groups = np.split(moving,np.where(np.diff(moving)>4)[0]+1)
        groups = [g for g in groups if len(g)>3 and positions[g[0],0]-positions[g[-1],0]>25]
        if not groups:
            raise ValueError('The filmed final straight reverse could not be established')
        group = groups[-1]
        start = max(0,group[0]-2)
        finish = min(len(times)-1,group[-1]+4)
        travel = positions[start,0]-positions[finish,0]
        progress = np.clip((positions[start,0]-positions[start:finish+1,0])/travel,0,1)
        return times[start:finish+1], np.maximum.accumulate(progress), float(travel)

    def at(self,time):
        if time>=self.final_reverse_start:
            fraction = float(np.interp(time,self.final_reverse_times,self.final_reverse_progress))
            final = self.final_pose
            xy = self.final_reverse_start_xy+fraction*(np.array([final.x,final.y])-self.final_reverse_start_xy)
            return Pose(round(time*1000),*xy,final.h,'Final parking','Measured final reverse / estimated gap timing'),True
        return super().at(time)

    def rear_reference_window(self,time,pose,scene):
        if time<=self.last_timed_seconds+3 or time>=self.final_reverse_start or abs(wrap180(pose.h))>12:
            return False
        hit = project_ray(pose,'rear',scene)
        before,_ = self.at(max(self.last_timed_seconds,time-.2))
        after,_ = self.at(min(self.duration,time+.2))
        speed = math.hypot(after.x-before.x,after.y-before.y)/.4
        return (hit is not None and hit['target']=='rear parking rail'
                and 20<hit['distance']<150 and speed<25)
