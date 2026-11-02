"""Parking from the registered capture pose: travel, pauses and executed arcs.

Extracts the current state machine and DriveProfile. Independent radius plant;
20ms loop, ideal speed tracking, imposed 0/1.5mm settled overrun. Computation
does NOT advance simulated time: reported seconds exclude real MCU planning,
USB saving, sensor faults, and the approach/registration before capture.
"""
import json
import shutil
import subprocess
import sys
from check_final_parking_sequence import ROOT, fixture_source
from check_connector_servo_resume import block

fixture = fixture_source()
fixture = fixture.replace(
    'void set_speed(int speed){command=speed;dc_state=DC_ENABLED;current_speed=measured_speed=speed;}',
    'void set_speed(int speed){command=speed;dc_state=DC_ENABLED;}')
code = fixture + r'''
using namespace parking_module;
void durationPlant(PositionEstimate &p,int steer,float ds,float scale){
 const float radius=Ackermann::getTurnRadius(float(steer));
 const float k=abs(steer)<2?0:-1.f/(radius*scale);
 const float h=p.heading_deg*PI/180.f,next=h+k*ds;
 if(fabsf(k)>1e-6f){p.x_mm+=(sinf(next)-sinf(h))/k;p.y_mm+=(cosf(h)-cosf(next))/k;}
 else{p.x_mm+=ds*cosf(h);p.y_mm+=ds*sinf(h);}
 p.heading_deg=parking_module::wrap180(next*180.f/PI);
}
int main(){int id=0,failed=0;
 struct Case{float gap,x,y,h;};
 for(auto c:{Case{232.5f,270.26f,294.6f,0},Case{245.2f,269.2f,295.9f,-.5f},
             Case{247.5f,270.26f,294.6f,0},Case{251.7f,270.26f,294.6f,0},
             Case{256.6f,270.3f,293.7f,-.2f},Case{262.5f,270.26f,294.6f,0}})
 for(int dir:{-1,1})for(float scale:{1.f,1.136f,1.235f})
 for(float yaw:{-1.5f,0.f,1.5f})for(float overrun:{0.f,1.5f}){
  ::routeTurnSign=dir;resetState();baseline();for(auto&s:seats)s.confirmed=s.injected=false;
  final_parking_reset();parking_module::routeTurnSign=dir;scanPhase=5;measuredGapMm=c.gap;
  outerSensor=dir>0?TOF_RIGHT:TOF_LEFT;
  localToField(c.x,c.y,c.h+yaw,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
  truePose=hostPose;hostTime=1000;encoder=0;hostGyroHealthy=true;framePeriod=30;
  entryRadiusScale=1;entryRadiusMeasured=false;
  stop(false);robot_logger.lines.str("");robot_logger.lines.clear();robot_logger.writes=0;
  state=FP_CAPTURE_BRAKE;stateStartMs=hostTime;
  bool collision=false;float travel=0,profile=0,acceleration=0,limit=0;
  unsigned long stateMs[22]={};int strokes=0,arcs=0,reversals=0,previousDirection=0;
  for(int tick=0;tick<6000&&!final_parking_complete()&&!final_parking_aborted();++tick){
   stateMs[int(state)]+=20;
   const bool moving=dc_state==DC_ENABLED;const int oldCommand=command,oldSteer=steeringCommand;
   final_parking_update(dir);
   float distance=0;
   if(dc_state==DC_ENABLED){
    if(!moving){profile=acceleration=0;++strokes;arcs+=steeringCommand!=0;
     int direction=command<0?-1:1;reversals+=previousDirection&&previousDirection!=direction;previousDirection=direction;}
    distance=DriveProfile::advance(profile,acceleration,limit,command,.02f,DEFAULT_ACCELERATION);
    current_speed=measured_speed=profile;
   }else{profile=acceleration=0;if(moving)distance=oldCommand<0?-overrun:overrun;}
   const int steer=dc_state==DC_ENABLED?steeringCommand:oldSteer;
   while(fabsf(distance)>.0001f){float ds=(distance<0?-1:1)*fminf(1,fabsf(distance));
    distance-=ds;encoder+=ds;travel+=fabsf(ds);
    durationPlant(hostPose,steer,ds,scale);truePose=hostPose;
    if(!parkingEntryPoseSafe(truePose,steer,0))collision=true;
   }hostTime+=20;
  }
  bool ok=final_parking_complete()&&finalFootprintContained()&&!collision;
  std::cout<<"TIME "<<id++<<" "<<ok<<" "<<c.gap<<" "<<dir<<" "<<scale<<" "<<yaw<<" "<<overrun
   <<" "<<(hostTime-1000)*.001f<<" "<<travel<<" "<<strokes<<" "<<arcs<<" "<<reversals<<" "<<int(entryReplans)
   <<" "<<stateMs[FP_SEGMENT_DRIVE]*.001f<<" "<<stateMs[FP_SEGMENT_SETTLE]*.001f
   <<" "<<(stateMs[FP_SEGMENT_BRAKE]+stateMs[FP_CAPTURE_BRAKE]+stateMs[FP_REPLAN_BRAKE])*.001f
   <<" "<<stateMs[FP_VERIFY]*.001f<<"\n";
  if(!ok){++failed;std::cout<<"FAIL_LOG "<<id-1<<"\n"<<robot_logger.lines.str();}
  if(c.gap==245.2f&&dir==-1&&scale==1&&yaw==0&&overrun==0)
   std::cout<<"NOMINAL_SHORT_LOG\n"<<robot_logger.lines.str();
 }
 return failed?1:0;
}
'''
if '--legacy-waits' in sys.argv:
    # Negative control: restore only the old waits, keeping the same geometry,
    # sensor adapter and plant for an apples-to-apples duration comparison.
    old=block(code,code.index('void beginEntrySettle()'))
    code=code.replace(old,'void beginEntrySettle(){stateStartMs=millis();state=FP_SEGMENT_SETTLE;}')
    code=code.replace('if (!parkingEntryBrakeReady())',
        'if (!parkingBrakeReady(OBSTACLE_FINAL_PARKING_HOLD_BRAKE_MS))')
    code=code.replace('if (!steering_command_settled(steering,\n            OBSTACLE_FINAL_PARKING_STEER_SETTLE_MS))',
        'if (millis()-stateStartMs<OBSTACLE_FINAL_PARKING_STEER_SETTLE_MS)')
    verify=block(code,code.index('if (state == FP_VERIFY)'))
    legacy=verify.replace('servo_disabled=false;set_steering(0);steer(0);',
        'servo_disabled=false;set_steering(0);steer(0);stateStartMs=millis();return true;')
    legacy=legacy.replace('if (!steering_command_settled(0,OBSTACLE_FINAL_PARKING_STEER_SETTLE_MS))',
        'if (millis()-stateStartMs<OBSTACLE_FINAL_PARKING_STEER_SETTLE_MS)')
    code=code.replace(verify,legacy)
if '--nominal' in sys.argv:
    code=code.replace('  ::routeTurnSign=dir;',
        '  if(scale!=1 || yaw!=0 || overrun!=0)continue;\n  ::routeTurnSign=dir;')
if '--filtered-speed' in sys.argv:
    # Match the real50ms encoder window/alpha, including filtered velocity
    # persisting after stop(). Ideal chassis motion is still an assumption.
    code=code.replace('current_speed=measured_speed=0;servo_disabled=true;',
                      'current_speed=0;servo_disabled=true;')
    code=code.replace('  state=FP_CAPTURE_BRAKE;stateStartMs=hostTime;',
        '  hostSpeedSamples=false;speed_measurement_count=0;measured_speed=0;\n'
        '  unsigned long velocityMs=hostTime;float velocityEncoder=encoder;\n'
        '  state=FP_CAPTURE_BRAKE;stateStartMs=hostTime;')
    code=code.replace('current_speed=measured_speed=profile;', 'current_speed=profile;')
    code=code.replace('   }hostTime+=20;', r'''
   }hostTime+=20;
   if((hostTime-velocityMs)*1000UL>=SPEED_MEASUREMENT_WINDOW_US){
    const float raw=(encoder-velocityEncoder)/((hostTime-velocityMs)*.001f);
    measured_speed+=SPEED_FILTER_ALPHA*(raw-measured_speed);
    ++speed_measurement_count;velocityMs=hostTime;velocityEncoder=encoder;
   }
''')
name='parking-duration'+('-nominal' if '--nominal' in sys.argv else '')+('-filtered' if '--filtered-speed' in sys.argv else '')
if '--legacy-waits' in sys.argv:name+='-legacy'
dest = ROOT/'local_workspace'/name
dest.mkdir(parents=True, exist_ok=True)
cpp, exe = dest/'check.cpp', dest/'check.exe'
cpp.write_text(code)
subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'include'),
                '-I', str(ROOT/'local_workspace/later-laps'), str(cpp), '-o', str(exe)], check=True)
run = subprocess.run([str(exe)], capture_output=True, text=True)
(dest/'output.txt').write_text(run.stdout+run.stderr)
keys = 'id passed gap direction radius yaw overrun seconds travel strokes arcs reversals replans drive_s steer_s brake_s verify_s'.split()
rows = [dict(zip(keys, map(float, line.split()[1:]))) for line in run.stdout.splitlines() if line.startswith('TIME ')]
(dest/'report.json').write_text(json.dumps(rows, indent=2)+'\n')
print('CASES', len(rows), 'FAILED', sum(not r['passed'] for r in rows))
for gap in sorted({r['gap'] for r in rows}):
    group = [r for r in rows if r['gap']==gap]
    nominal = [r for r in group if r['radius']==1 and r['yaw']==0 and r['overrun']==0]
    print('GAP', gap, 'NOMINAL', [{k:r[k] for k in ('direction','seconds','travel','strokes','arcs','reversals','replans','steer_s','brake_s')} for r in nominal],
          'RANGE_S', min(r['seconds'] for r in group), max(r['seconds'] for r in group))
run.check_returncode()
