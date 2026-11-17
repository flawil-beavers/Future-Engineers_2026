from pathlib import Path
import argparse,sys,json,cv2,numpy as np
from scipy.ndimage import median_filter,gaussian_filter1d
from render_video_aligned_open import sha256
ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description='Track the DJI overhead Open run using reviewed enclosure calibration');parser.add_argument('--video',required=True,type=Path);V=parser.parse_args().video
digest=sha256(V)
if digest!='c0ff2286157bfc5cbf64d98ecb366f184f5914950e0f18be75fa8e82bed6419f':raise ValueError('This reviewed calibration is specific to the supplied DJI video')
OUT=ROOT/'local_workspace/open-dji-sync';OUT.mkdir(exist_ok=True)
cv2.setNumThreads(1)
cap=cv2.VideoCapture(str(V));fps=cap.get(cv2.CAP_PROP_FPS);n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
bg=[]
if (OUT/'background.jpg').exists():background=cv2.imread(str(OUT/'background.jpg'))
else:
 for t in np.linspace(0,(n-1)/fps,35):
  cap.set(cv2.CAP_PROP_POS_MSEC,t*1000);ok,f=cap.read()
  if ok:bg.append(cv2.resize(f,(960,540)))
 background=np.median(bg,axis=0).astype(np.uint8)
cv2.imwrite(str(OUT/'background.jpg'),background)
# Calibrate the OUTSIDE edges of the enclosure, not the printed poster.
src=np.float32([[1865,365],[2680,365],[2668,1835],[1850,1820]])/4
dst=np.float32([[-500,900],[500,900],[500,-900],[-500,-900]])
H=cv2.getPerspectiveTransform(src,dst)
yy,xx=np.indices((540,960));points=np.stack([xx,yy],axis=-1).astype(np.float32)
world=cv2.perspectiveTransform(points.reshape(-1,1,2),H).reshape(540,960,2)
mask=(np.abs(world[:,:,0])<1450)&(np.abs(world[:,:,1])<1430)&((np.abs(world[:,:,0])>540)|(np.abs(world[:,:,1])>950))
cap.set(cv2.CAP_PROP_POS_FRAMES,0)
rows=[];last=np.array([548.,477.]);sheet=[]
for i in range(n):
 ok=cap.grab()
 if not ok:break
 if i%6:continue
 ok,f=cap.retrieve()
 small=cv2.resize(f,(960,540))
 delta=cv2.absdiff(small,background).max(axis=2)
 hsv=cv2.cvtColor(small,cv2.COLOR_BGR2HSV)
 dark=(cv2.cvtColor(small,cv2.COLOR_BGR2GRAY)<100)&(hsv[:,:,1]<155)
 binary=((delta>35)&dark&mask).astype(np.uint8)*255
 binary=cv2.morphologyEx(binary,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
 num,labels,stats,centroids=cv2.connectedComponentsWithStats(binary)
 choices=[j for j in range(1,num) if 55<stats[j,cv2.CC_STAT_AREA]<1500 and max(stats[j,2:4])<65]
 if choices:
  j=max(choices,key=lambda j:stats[j,cv2.CC_STAT_AREA])
  p=centroids[j];dist=np.linalg.norm(p-last)
  if dist<120 or not rows or i/fps-rows[-1][0]>1:
   last=p;rows.append([i/fps,*p,float(stats[j,cv2.CC_STAT_AREA])])
 if i%300==0:
  cv2.circle(small,tuple(last.astype(int)),15,(0,0,255),2);cv2.putText(small,str(round(i/fps,1)),(10,25),0,.8,(0,0,255),2);sheet.append(small)
 if i%600==0:print('Track',i,n,flush=True)
cap.release()
arr=np.array(rows);p=cv2.perspectiveTransform(arr[:,1:3].astype(np.float32).reshape(-1,1,2),H).reshape(-1,2)
for axis in range(2):p[:,axis]=gaussian_filter1d(median_filter(p[:,axis],size=3),1)
times=arr[:,0]
# Reviewed false detections and their contaminated silhouette edges. Interpolate
# missing positions without compressing time, before uniform-time smoothing.
good=((np.abs(p[:,0])>650)|(np.abs(p[:,1])>980))&(np.abs(p).max(axis=1)<1400)
rejected=times[~good].tolist()
p=np.column_stack([np.interp(times,times[good],p[good,a]) for a in (0,1)])
good=~((times>106.5)&(times<109.8))
p=np.column_stack([np.interp(times,times[good],p[good,a]) for a in (0,1)])
p=gaussian_filter1d(p,.8,axis=0)
uniform=np.arange(0,n/fps,6/fps)
p=np.column_stack([np.interp(uniform,times,p[:,a]) for a in (0,1)])
p=gaussian_filter1d(p,1.5,axis=0)
arr=np.column_stack([uniform,np.zeros((len(uniform),3))])
velocity=np.gradient(p,arr[:,0],axis=0);speed=np.linalg.norm(velocity,axis=1)
guide=np.unwrap(np.arctan2(p[:,1],p[:,0]))*180/np.pi+90
raw=np.rad2deg(np.arctan2(velocity[:,1],velocity[:,0]))
heading=raw+360*np.round((guide-raw)/360)
# Stationary samples get heading from nearest moving sample.
moving=np.where(speed>45)[0]
heading=np.interp(arr[:,0],arr[moving,0],heading[moving])
heading=gaussian_filter1d(heading,.6)
data=dict(video=V.name,fps=fps,duration=n/fps,homography=H.tolist(),calibration_pixels=(src*4).tolist(),track=[dict(t=float(t),x=float(x),y=float(y),heading=float(h),speed=float(s)) for (t,*_), (x,y),h,s in zip(arr,p,heading,speed)])
data.update(source_video_sha256=digest,rejected_off_circuit_samples=rejected,interpolated_detection_gap_seconds=[106.5,109.8],resampling='Uniform0.2002s clock before Gaussian1.5sample smoothing; interpolate missed detections, preserve clock.')
(OUT/'track.json').write_text(json.dumps(data,indent=2))
canvas=np.full((900,900,3),245,np.uint8)
def pt(x,y):return (int(450+x*.27),int(450-y*.27))
cv2.rectangle(canvas,pt(-500,900),pt(500,-900),(160,160,160),2)
for a,b in zip(p[:-1],p[1:]):cv2.line(canvas,pt(*a),pt(*b),(0,160,0),2)
for k in range(0,len(p),25):cv2.putText(canvas,str(round(arr[k,0])),pt(*p[k]),0,.4,(0,0,180),1)
cv2.imwrite(str(OUT/'track.png'),canvas)
contact=np.full((540*((len(sheet)+2)//3),960*3,3),255,np.uint8)
for j,f in enumerate(sheet):contact[j//3*540:(j//3+1)*540,j%3*960:(j%3+1)*960]=f
cv2.imwrite(str(OUT/'tracking_contact.jpg'),contact)
print('samples',len(rows),'heading range',heading.min(),heading.max(),'max speed',speed.max(),'moving',arr[moving[0],0],arr[moving[-1],0],flush=True)
for k in range(12):
 ix=np.where(heading>k*90+10)[0];jx=np.where(heading>k*90+85)[0]
 print(k+1,arr[ix[0],0] if len(ix) else None,arr[jx[0],0] if len(jx) else None)
