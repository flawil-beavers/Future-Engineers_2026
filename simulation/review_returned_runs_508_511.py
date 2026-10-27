"""Reproduce the 2026-10-26 batch review from immutable archived logs.

Source-extracted CAD steering/preflight and physical footprint checks use the
logged onboard poses, which are not independent physical ground truth.
Generated code, executables and reports stay in ignored local_workspace.
No firmware changes or robot movement.
"""
import json
import re
import shutil
import subprocess

from check_cw_start_planner import ROOT, fixture_source
from check_connector_servo_resume import block
from analyze_parking_exit_pose import fields


def main():
    dest = ROOT / 'local_workspace/batch_20261026_508_511'
    dest.mkdir(parents=True, exist_ok=True)
    firmware = (ROOT / 'src/obstacle_path.cpp').read_text()
    fixture = fixture_source().split('int main()')[0]
    fixture = fixture.replace('anchor.heading_deg=180;',
        'anchor.heading_deg=routeTurnSign>0?0:180;')
    preflight = block(firmware, firmware.index('bool preflightCwStartArc('))
    preflight = preflight.replace('preflightCwStartArc(', 'reviewStartArc(', 1)
    rejection = '{std::cout<<" ARC_REJECT @REASON@ "<<headingError<<" "<<travel<<" "<<x<<" "<<y<<" "<<heading*180/PI;return false;}'
    preflight = preflight.replace('return false;', rejection.replace('@REASON@', 'footprint'), 1)
    at = preflight.index('for (uint8_t seat')
    preflight = preflight[:at] + preflight[at:].replace('return false;',
        rejection.replace('@REASON@', 'pillar_or_wall'), 1)
    rollout = block(firmware, firmware.index('bool connectorRolloutFeasible('))
    rollout = rollout.replace('connectorRolloutFeasible(', 'reviewConnector(', 1)
    rollout = rollout.replace('bool continueIntoRoute)\n{', 'bool continueIntoRoute, float factor)\n{', 1)
    rollout = rollout.replace('static_cast<int>(steering));', 'static_cast<int>(steering)) * factor;')
    rollout = rollout.replace('        Serial.print("[PARK ENTRY CONNECTOR] Rollout reject reason=");',
        '        std::cout<<" ROLLOUT_REJECT "<<reason<<" "<<travel<<" "<<steering;\n'
        '        Serial.print("[PARK ENTRY CONNECTOR] Rollout reject reason=");')
    assert '* factor;' in rollout
    fixture += preflight + '\n' + rollout + r'''
int main(){char mode;int id;float x,y,h;
 while(std::cin>>mode>>id>>x>>y>>h){
  if(mode=='T'){
   float tx,ty;std::cin>>tx>>ty;float a=h*PI/180,dx=tx-x,dy=ty-y;
   float forward=dx*cosf(a)+dy*sinf(a),lateral=-dx*sinf(a)+dy*cosf(a);
   float k=2*lateral/fmaxf(1,dx*dx+dy*dy);
   std::cout<<"TRACK "<<id<<" "<<forward<<" "<<lateral<<" "
    <<pursuitServoForCurvature(k)<<"\n";
  }else if(mode=='F'){
   int steer;std::cin>>steer;
   parking_start_footprint::Quad bodies[6];
   parking_start_footprint::robotQuads(x,y,h,(steer>0)-(steer<0),0,bodies);
   for(float reserve:{0.f,5.f})for(float gap:{242.5f,247.5f,252.5f}){
    auto fixed=parking_start_footprint::rectangle(490,-1400,10+reserve,100,0);
    auto moved=parking_start_footprint::rectangle(480-gap-10,-1400,10+reserve,100,0);
    unsigned fixedMask=0,movedMask=0;
    for(int i=0;i<6;++i){
     if(parking_start_footprint::overlaps(bodies[i],fixed))fixedMask|=1u<<i;
     if(parking_start_footprint::overlaps(bodies[i],moved))movedMask|=1u<<i;
    }
    std::cout<<"FOOTPRINT "<<id<<" "<<reserve<<" "<<gap<<" "<<fixedMask<<" "<<movedMask<<"\n";
   }
  }else if(mode=='P'){
   routeTurnSign=1;parkingCwShortStart=false;parkingCcwShortStart=true;
   parkingCcwFirstEdgeStart=true;baseline();PositionEstimate p{x,y,h};
   std::cout<<"PREFLIGHT "<<id;
   bool ok=reviewStartArc(p,parkingEntryScanArcMm()+parkingEntryScoutArcMm());
   std::cout<<" RESULT "<<ok<<"\n";
  }else if(mode=='C'){
   float factor;int n;std::cin>>factor>>n;
   routeTurnSign=-1;parkingCwShortStart=true;parkingCcwShortStart=false;baseline();
   memcpy(livePath,baselinePath,sizeof(PathPoint)*pathLength);
   seats[0].confirmed=true;seats[0].red=false;
   seats[4].confirmed=seats[4].injected=true;seats[4].red=true;
   displaceForSeat(livePath,4,260,false);
   parkingEntryConnectorLength=n;
   for(int i=0;i<n;++i){auto &p=parkingEntryConnector[i];std::cin>>p.x>>p.y>>p.headingDeg;}
   PositionEstimate p{x,y,h};
   std::cout<<"CONNECTOR "<<id<<" "<<factor;
   bool ok=reviewConnector(p,150,2,-1,0,livePath,0,true,factor);
   std::cout<<" RESULT "<<ok<<"\n";
  }
 }
}
'''
    cpp, exe = dest/'review.cpp', dest/'review.exe'
    cpp.write_text(fixture)
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'include'),
        '-I', str(ROOT/'simulation/host'), str(cpp), '-o', str(exe)], check=True)
    inputs, poses, tracks, connectors, preflights = [], [], [], [], []
    run_summary = []
    for number, direction in ((508,'cw'),(509,'ccw'),(510,'cw'),(511,'cw')):
        path = ROOT/f'simulation/evidence/parking_exit_diagnostics/20261026_log_{number}_{direction}.txt'
        lines = path.read_text().splitlines()
        active = False
        points = []
        phases = []
        last_phase = None
        for line_number, line in enumerate(lines, 1):
            if line.startswith('[PARK FIELD START]'): active = True
            if active and line.startswith('[PARK_DIAG]') and 'type=sample ' in line and 'state=segment_' in line:
                record = fields(line)
                for kind in ('pose','nominal'):
                    pose = list(map(float,record[kind].split(',')))
                    index = len(poses)
                    poses.append(dict(log=number,line=line_number,t=int(record['t']),segment=int(record['seg']),
                        state=record['state'],kind=kind,pose=pose,steering=int(float(record['steer'])),
                        speed=float(record['speed']),encoder_mm=float(record['emm'])))
                    inputs.append(' '.join(map(str,['F',index,*pose,record['steer']])))
            if line.startswith('[CONNECTOR_POINT]'):
                record = fields(line)
                if record['kind']=='connector':
                    if record['index']=='0': points=[]
                    points.append([float(record[k]) for k in ('x','y','h')])
            if line.startswith('[CONNECTOR_TRACK]'):
                record = fields(line); index=len(tracks)
                tracks.append(dict(log=number,line=line_number,**record))
                inputs.append(' '.join(map(str,['T',index,record['x'],record['y'],record['h'],record['tx'],record['ty']])))
            if line.startswith('[FINAL_PARK_TRACE]'):
                record=fields(line);phase=record['state/phase']
                if phase!=last_phase: phases.append(dict(line=line_number,**record)); last_phase=phase
        if points:
            for factor in (.8,.9,1.,1.1,1.2):
                index=len(connectors);connectors.append(dict(log=number,factor=factor))
                inputs.append(' '.join(map(str,['C',index,*points[0],factor,len(points),*(v for p in points for v in p)])))
        if number==509:
            match=re.search(r'\[PARK LOCALIZE POSE\] x_y_heading=([^\n]+)', '\n'.join(lines))
            x,y,h=map(float,match.group(1).split('/'))
            for dx,dy,dh in ((0,0,0),(0,1,0),(0,2,0),(0,3,0),(0,4,0),(0,5,0),(0,10,0),(0,15,0),(0,0,1.6)):
                index=len(preflights);pose=[x+dx,y+dy,h+dh];preflights.append(dict(pose=pose))
                inputs.append(' '.join(map(str,['P',index,*pose])))
        run_summary.append(dict(log=number,phase_transitions=phases,
            run_laps=[fields(l) for l in lines if l.startswith('[RUN_LAP]')],
            run_end=[fields(l) for l in lines if l.startswith('[RUN_END]')],
            gyro_restarts=sum('Sensor report timeout;' in l for l in lines)))
    result=subprocess.run([str(exe)],input='\n'.join(inputs)+'\n',text=True,capture_output=True,check=True)
    (dest/'review_input.txt').write_text('\n'.join(inputs)+'\n')
    (dest/'review_output.txt').write_text(result.stdout)
    hits=[]
    for line in result.stdout.splitlines():
        values=line.split()
        if values[0]=='FOOTPRINT' and (int(values[4]) or int(values[5])):
            hits.append(dict(**poses[int(values[1])],reserve=float(values[2]),gap=float(values[3]),
                fixed_mask=int(values[4]),moved_mask=int(values[5])))
        elif values[0]=='TRACK':
            record=tracks[int(values[1])];record['replay_steering']=float(values[4])
            record['replay_error']=abs(record['replay_steering']-float(record['steering']))
        elif values[0]=='PREFLIGHT':preflights[int(values[1])]['result']=line
        elif values[0]=='CONNECTOR':connectors[int(values[1])]['result']=line
    report=dict(runs=run_summary,footprint_hits=hits,tracks=tracks,connectors=connectors,preflights=preflights,
        limitations=['Onboard estimated poses, nominal geometry and extreme-steer polygons; no independent contact measurement.',
            'Connector factors are arbitrary sensitivity probes, not fitted calibration or probability.',
            'Sampled final-parking traces do not expose every ToF frame or scan-line latch.'])
    (dest/'review.json').write_text(json.dumps(report,indent=2)+'\n')
    for n in (508,509,510,511):
        actual=[r for r in hits if r['log']==n and r['kind']=='pose' and r['reserve']==0]
        print('FOOTPRINT',n,'physical model hits',len(actual),'first',actual[:1])
    print('CAD target replay maximum steering discrepancy',max((r['replay_error'] for r in tracks),default=0))
    for r in connectors+preflights: print(r)


if __name__=='__main__':main()
