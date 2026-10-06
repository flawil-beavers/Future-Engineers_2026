"""Ideal kinematic rollout of logged CW connectors with outgoing-route tangents.

Mirrored cases are synthetic, not replays of the CCW preflight failures.
"""
import argparse, json, math
from pathlib import Path
from analyze_connector_tracking import fields, parse_point, lookahead, steering
from parking_entry_scout_sim import clearance

def wrap(a): return (a+180)%360-180
def geometry(path):
    connector=[]; route=[]
    for line in path.read_text().splitlines():
        if line.startswith('[CONNECTOR_POINT]'):
            f=fields(line)
            (connector if f['kind']=='connector' else route).append(parse_point(f))
    return connector,route
def curve(start,end,n,start_scale=1.25,end_scale=1.5):
    chord=math.hypot(end['x']-start['x'],end['y']-start['y'])
    a=math.radians(start['h']); b=math.radians(end['h'])
    sx,sy=math.cos(a)*min(chord*start_scale,800),math.sin(a)*min(chord*start_scale,800)
    ex,ey=math.cos(b)*min(chord*end_scale,1000),math.sin(b)*min(chord*end_scale,1000)
    out=[]
    for i in range(n):
        t=i/(n-1); t2=t*t;t3=t2*t
        x=(2*t3-3*t2+1)*start['x']+(t3-2*t2+t)*sx+(-2*t3+3*t2)*end['x']+(t3-t2)*ex
        y=(2*t3-3*t2+1)*start['y']+(t3-2*t2+t)*sy+(-2*t3+3*t2)*end['y']+(t3-t2)*ey
        dx=(end['x']-start['x'])*(6*t-6*t2)+sx*(3*t2-4*t+1)+ex*(3*t2-2*t)
        dy=(end['y']-start['y'])*(6*t-6*t2)+sy*(3*t2-4*t+1)+ey*(3*t2-2*t)
        out.append(dict(x=x,y=y,h=math.degrees(math.atan2(dy,dx))))
    return out
def simulate(c,r,L=150,shift=(0,0,0), mirror=False, yaw_gain=1.0,
             start_index=0, check_pink=False, pillar_seats=None,
             continue_route=False, start_pose=None, max_travel_mm=500, trace=None):
    p=dict(c[start_index] if start_pose is None else start_pose)
    p['x']+=shift[0];p['y']+=shift[1];p['h']+=shift[2];idx=start_index
    min_wall=min_pillar=math.inf
    for step in range(int(max_travel_mm / 2) + 1):
        if trace is not None:
            trace.append(dict(p, travel_mm=step*2))
        for rear in (0,70):
            angle=math.radians(p['h'])
            pose=(p['x']-rear*math.cos(angle),p['y']-rear*math.sin(angle),p['h'])
            for seat in (pillar_seats if pillar_seats is not None else
                         ((-500 if mirror else 500,-900),(0,-900))):
                wall,pillar=clearance(pose,seat)
                min_wall=min(min_wall,wall);min_pillar=min(min_pillar,pillar)
        if min_wall<=10 or min_pillar<=10:
            return dict(status='clearance',travel_mm=step*2,wall_mm=min_wall,pillar_mm=min_pillar)
        d=math.hypot(p['x']-c[-1]['x'],p['y']-c[-1]['y']);h=abs(wrap(p['h']-c[-1]['h']))
        if d<=60 and h<=15:return dict(status='pass',travel_mm=step*2,distance_mm=d,heading_deg=h,wall_mm=min_wall,pillar_mm=min_pillar)
        while idx+1<len(c) and math.hypot(p['x']-c[idx+1]['x'],p['y']-c[idx+1]['y'])<math.hypot(p['x']-c[idx]['x'],p['y']-c[idx]['y']):idx+=1
        target=lookahead(c,r,idx,L,extend=continue_route)
        x,y,s=steering(p,target,100)
        if x<=1 or abs(s)>42:return dict(status='guard',travel_mm=step*2,distance_mm=d,heading_deg=h,steering_deg=s)
        if check_pink:
            from parking_exit_swept_search import Pose, collision
            local_pose=Pose(480-p['x'],p['y']+1500,180-p['h'])
            if any(collision(local_pose,-round(s),gap) for gap in (242.5,252.5)):
                return dict(status='pink',travel_mm=step*2)
        curvature=-math.tan(math.radians(s))/100*yaw_gain
        angle=math.radians(p['h']);delta=curvature*2
        if abs(curvature)>1e-6:
            p['x']+=(math.sin(angle+delta)-math.sin(angle))/curvature
            p['y']+=(math.cos(angle)-math.cos(angle+delta))/curvature
        else:
            p['x']+=2*math.cos(angle);p['y']+=2*math.sin(angle)
        p['h']+=math.degrees(delta)
    return dict(status='travel',travel_mm=max_travel_mm,distance_mm=d,heading_deg=h)

def corrected(c,r):
    if len(c)<3 or len(r)<2:raise ValueError('Complete connector and outgoing route required')
    end=dict(c[-1]);end['h']=math.degrees(math.atan2(r[1]['y']-r[0]['y'],r[1]['x']-r[0]['x']))
    return curve(c[0],end,len(c))

def preflight_lookahead(connector, route, green=False, mirror=False):
    desired = 150 * (0.55 if green else 1)
    candidate = desired
    while candidate >= 24.9:
        feasible = True
        for progress, at in enumerate(connector[:-1]):
            if math.hypot(at['x']-connector[-1]['x'],at['y']-connector[-1]['y']) <= 60 and abs(wrap(at['h']-connector[-1]['h'])) <= 15:
                break
            target=lookahead(connector,route,progress,candidate)
            forward,_,command=steering(at,target,100)
            if forward<=1 or abs(command)>42:
                feasible=False;break
        if feasible and simulate(connector,route,L=candidate,mirror=mirror)['status']=='pass':
            return candidate
        candidate-=25
    return None

def reflect(points):
    return [dict(p,x=-p['x'],h=wrap(180-p['h'])) for p in points]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs',nargs='+',type=Path)
    args=parser.parse_args();reports=[]
    for path in args.logs:
        c,r=geometry(path);new=corrected(c,r)
        cases=[]
        for mirror in (False,True):
            points=reflect(new) if mirror else new
            route=reflect(r) if mirror else r
            for x in (-10,0,10):
                for y in (-10,0,10):
                    for h in (-2,0,2):
                        for gain in (0.85,1,1.15):
                            cases.append(simulate(points,route,shift=(x,y,h),mirror=mirror,yaw_gain=gain))
        report=dict(log=path.name,old=simulate(c,r),corrected=simulate(new,r),
                    preflight_lookahead_mm=preflight_lookahead(new,r),
                    cases=len(cases),failures=sum(v['status']!='pass' for v in cases),
                    min_wall_mm=min(v.get('wall_mm',math.inf) for v in cases),
                    min_pillar_mm=min(v.get('pillar_mm',math.inf) for v in cases))
        reports.append(report);print(json.dumps(report))
    destination=Path('local_workspace/connector-transition/rollout.json')
    destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_text(json.dumps(reports,indent=2))
    return int(any(r['failures'] for r in reports))

if __name__=='__main__':raise SystemExit(main())
