"""Match run633 log events to the overhead recording clock."""
import json

import numpy as np
from scipy.ndimage import gaussian_filter1d

from run_633_log_timed import LogTimedMotion
from run_633_video_alignment import OUTPUT


def bottom_crossings(times,positions):
    crossings = []
    for index in range(1,len(times)):
        left,right = positions[index-1:index+1]
        if left[0]<0<=right[0] and right[1]<-700:
            fraction = -left[0]/(right[0]-left[0])
            time = float(times[index-1]+fraction*(times[index]-times[index-1]))
            if not crossings or time-crossings[-1]>5:
                crossings.append(time)
    return crossings


def calibrate():
    motion = LogTimedMotion()
    log_times = np.array([(p.t-motion.log['start'])/1000 for p in motion.log['samples']])
    log_positions = np.array([[p.x,p.y] for p in motion.log['samples']])
    early = [p for p,t in zip(motion.log['samples'],log_times) if t<12]
    peak = max(early,key=lambda p:(p.h+180)%360-180)
    filmed_heading = (motion.video_heading+180)%360-180
    indices = np.flatnonzero((motion.video_times>285)&(motion.video_times<291))
    film_peak = indices[np.argmax(filmed_heading[indices])]
    events = [dict(name='initial unparking heading peak',
                   log_seconds=(peak.t-motion.log['start'])/1000,
                   film_seconds=float(motion.video_times[film_peak]))]
    log_crossings = bottom_crossings(log_times,log_positions)
    film_crossings = bottom_crossings(motion.video_times,motion.video_xy)
    if len(log_crossings)!=3 or len(film_crossings)!=3:
        raise ValueError('Expected three corresponding bottom course crossings')
    for lap,(logged,filmed) in enumerate(zip(log_crossings,film_crossings),1):
        events.append(dict(name=f'bottom positive-X crossing {lap}',log_seconds=logged,film_seconds=filmed))
    events.append(dict(name='final reverse settled',log_seconds=motion.duration,
                       film_seconds=motion.offset+motion.final_reverse_film_times[-1]))
    logged = np.array([e['log_seconds'] for e in events])
    filmed = np.array([e['film_seconds'] for e in events])
    scale,offset = np.polyfit(logged,filmed,1)
    for event in events:
        event['residual_seconds'] = float(offset+scale*event['log_seconds']-event['film_seconds'])

    # Check a separate event from the departure and three parking stop windows.
    smooth = gaussian_filter1d(motion.video_xy,3,axis=0)
    validation = []
    windows = [('unparking forward endpoint',0,15,290,293,'max'),
               ('parking scan start',138,140,415,430,'max'),
               ('parking scan reverse endpoint',154,156,430,445,'min'),
               ('parking capture endpoint',163,164,443,453,'max')]
    for name,log_lo,log_hi,film_lo,film_hi,kind in windows:
        log_indices = np.flatnonzero((log_times>=log_lo)&(log_times<=log_hi))
        film_indices = np.flatnonzero((motion.video_times>=film_lo)&(motion.video_times<=film_hi))
        choose = np.argmax if kind=='max' else np.argmin
        log_index = log_indices[choose(log_positions[log_indices,0])]
        film_index = film_indices[choose(smooth[film_indices,0])]
        logged_time = float(log_times[log_index])
        filmed_time = float(motion.video_times[film_index])
        validation.append(dict(name=name,log_seconds=logged_time,film_seconds=filmed_time,
                               residual_seconds=float(offset+scale*logged_time-filmed_time)))
    assert max(abs(e['residual_seconds']) for e in events)<.2
    assert max(abs(e['residual_seconds']) for e in validation)<1
    result = dict(method='uniform affine log-to-film clock fit',
                  seconds_per_log_second=float(scale),film_start_seconds=float(offset),
                  log_duration_seconds=motion.duration,playback_duration_seconds=float(scale*motion.duration),
                  fit_events=events,independent_validation=validation,
                  model='film_seconds = film_start_seconds + seconds_per_log_second * log_seconds',
                  uncertainty='Tracked event times and stop-window extrema are estimates')
    (OUTPUT/'clock_calibration.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':
    calibrate()
