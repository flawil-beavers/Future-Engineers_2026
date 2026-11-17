"""Reject pillar detections and measure chassis orientation from its silhouette."""
import argparse
import json

import cv2
import numpy as np
from scipy.ndimage import gaussian_filter1d

from prepare_run_633_drone import OUTPUT, FIELD_IMAGE


def refine():
    film = json.loads((OUTPUT/'film_frames.json').read_text())
    matrix = FIELD_IMAGE @ np.array(film['image_to_world'])
    background = cv2.imread(str(OUTPUT/'field_background.jpg'))
    yy,xx = np.indices((1000,1000))
    wx,wy = (xx-500)/.3,(500-yy)/.3
    region = ((abs(wx)<1470)&(abs(wy)<1470)&((abs(wx)>540)|(abs(wy)>540)))
    records = []
    for index,row in enumerate(film['frames']):
        image = cv2.imread(str(OUTPUT/row['image']))
        field = cv2.warpPerspective(image,matrix,(1000,1000))
        gray = cv2.cvtColor(field,cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(field,cv2.COLOR_BGR2HSV)
        delta = background.astype(np.int16).mean(axis=2)-field.astype(np.int16).mean(axis=2)
        binary = ((gray<125)&(hsv[:,:,1]<165)&(delta>25)&region).astype(np.uint8)*255
        binary = cv2.morphologyEx(binary,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
        count,labels,stats,centers = cv2.connectedComponentsWithStats(binary)
        choices = [i for i in range(1,count) if 600<stats[i,cv2.CC_STAT_AREA]<4000
                   and max(stats[i,2:4])<115]
        if choices:
            component = max(choices,key=lambda i:stats[i,cv2.CC_STAT_AREA])
            y,x = np.where(labels==component)
            coords = np.column_stack((x,-y))
            values,vectors = np.linalg.eigh(np.cov(coords.T))
            axis = vectors[:,-1]
            angle = np.degrees(np.arctan2(axis[1],axis[0]))
            center = centers[component]
            records.append(dict(t=row['time'],x=(center[0]-500)/.3,y=(500-center[1])/.3,
                                angle=angle,anisotropy=values[-1]/values[0],
                                area=int(stats[component,cv2.CC_STAT_AREA])))
        if index%600==0:
            print('Refining silhouette',index,'/',len(film['frames']),flush=True)
    times = np.array([r['t'] for r in records])
    angles = np.degrees(np.unwrap(np.radians([2*r['angle'] for r in records])))/2
    angles -= round(angles[0]/180)*180
    uniform = np.array([r['time'] for r in film['frames']])
    xy = np.column_stack([np.interp(uniform,times,[r[a] for r in records]) for a in ('x','y')])
    xy = gaussian_filter1d(xy,1.2,axis=0)
    angles = gaussian_filter1d(np.interp(uniform,times,angles),1.5)
    data = json.loads((OUTPUT/'video_track.json').read_text())
    track = [dict(t=float(t),x=float(p[0]),y=float(p[1]),body_heading=float(h))
             for t,p,h in zip(uniform,xy,angles)]
    data.update(track=track,detected_frames=len(records),
                tracking_method='Robot-sized foreground silhouettes, PCA orientation with 180-degree continuity',
                silhouette_observations=records)
    (OUTPUT/'video_track_refined.json').write_text(json.dumps(data,indent=2)+'\n')
    print('Valid robot silhouettes',len(records),'/',len(uniform),flush=True)


def refine_heading():
    film = json.loads((OUTPUT/'film_frames.json').read_text())
    data = json.loads((OUTPUT/'video_track_refined.json').read_text())
    matrix = FIELD_IMAGE @ np.array(film['image_to_world'])
    background = cv2.imread(str(OUTPUT/'field_background.jpg'))
    angles = []
    for index,(row,track) in enumerate(zip(film['frames'],data['track'])):
        cx,cy = 500+.3*track['x'],500-.3*track['y']
        translation = np.array([[1,0,40-cx],[0,1,40-cy],[0,0,1.]])
        patch = cv2.warpPerspective(cv2.imread(str(OUTPUT/row['image'])),translation@matrix,(80,80))
        reference = cv2.getRectSubPix(background,(80,80),(float(cx),float(cy)))
        gray = cv2.cvtColor(patch,cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(patch,cv2.COLOR_BGR2HSV)
        delta = reference.astype(np.int16).mean(axis=2)-patch.astype(np.int16).mean(axis=2)
        mask = ((gray<125)&(hsv[:,:,1]<165)&(delta>25)).astype(np.uint8)*255
        mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
        count,labels,stats,centers = cv2.connectedComponentsWithStats(mask)
        choices = [i for i in range(1,count) if 600<stats[i,cv2.CC_STAT_AREA]<4000]
        angle = track['body_heading']
        if choices:
            component = max(choices,key=lambda i:stats[i,cv2.CC_STAT_AREA])
            y,x = np.where(labels==component)
            rectangle = cv2.minAreaRect(np.float32(np.column_stack((x,y))))
            observed = -rectangle[2]
            # Choose the rectangle axis consistent with the continuous PCA
            # heading. This removes skew from uneven electronics/wheel mass.
            angle += (observed-angle+45)%90-45
        angles.append(angle)
        if index%900==0:
            print('Refining chassis rectangle',index,'/',len(data['track']),flush=True)
    angles = gaussian_filter1d(np.array(angles),1.5)
    for track,angle in zip(data['track'],angles):
        track['body_heading'] = float(angle)
    data['heading_method'] = 'Minimum enclosing chassis rectangle, axis resolved by continuous silhouette PCA'
    (OUTPUT/'video_track_measurement.json').write_text(json.dumps(data,indent=2)+'\n')
    print('Chassis headings refined',flush=True)


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--heading-only',action='store_true')
    args = parser.parse_args()
    cv2.setNumThreads(2)
    if not args.heading_only:
        refine()
    refine_heading()
