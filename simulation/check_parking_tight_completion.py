"""Recover inside a tight bay without requiring neutral steering first.

Uses the production state machine with measured yaw from a distinct-radius
plant. Includes 10 mm physical end clearances and 5.5 mm edge poses that pass
final containment but still require repositioning before neutral verification.
No sensor bias, actuator lag or physical reliability claim is modeled here.
"""
import shutil
import subprocess
import sys

from check_final_parking_sequence import ROOT, fixture_source


def main():
    source = fixture_source()
    if '--old-neutral-gate' in sys.argv:
        source = source.replace('// ENTRY_FIRST_STEERING_CHECKED_BY_SWEEP',
                                'if (!parkingEntryPoseSafe(field,0,margin)) continue;')
    if '--old-finish-loop' in sys.argv:
        source = source.replace('finalFootprintContained() && parkingEntryPoseSafe(get_position_struct(),0)',
                                'finalFootprintContained()')
    source += r'''
using namespace parking_module;
void plant(PositionEstimate &p,int steering,float ds,float scale){
 const float r=Ackermann::getTurnRadius(float(steering));
 const float k=fabsf(float(steering))<2 ? 0:-1.f/(r*scale);
 const float h=p.heading_deg*PI/180.f,next=h+k*ds;
 if(fabsf(k)>1e-6f){p.x_mm+=(sinf(next)-sinf(h))/k;p.y_mm+=(cosf(h)-cosf(next))/k;}
 else{p.x_mm+=ds*cosf(h);p.y_mm+=ds*sinf(h);}
 p.heading_deg=parking_module::wrap180(next*180.f/PI);
}
int main(){int total=0,passed=0;
 for(int direction:{-1,1})for(float scale:{.95f,1.f,1.15f})
 for(auto anchor:{PositionEstimate{110,120,35},PositionEstimate{45.5f,120,0},
                  PositionEstimate{117,120,0},PositionEstimate{50,120,0},
                  PositionEstimate{112,120,0}}){
  ::routeTurnSign=direction;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  final_parking_reset();parking_module::routeTurnSign=direction;scanPhase=5;measuredGapMm=247.5f;
  localToField(anchor.x_mm,anchor.y_mm,anchor.heading_deg,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
  truePose=hostPose;hostTime=1000;encoder=0;hostGyroHealthy=true;framePeriod=30;
  stop(false);robot_logger.lines.str("");robot_logger.lines.clear();robot_logger.writes=0;
  steeringCommand=set_degree=anchor.heading_deg ? 50*direction:0;
  if(!parkingEntryPoseSafe(truePose,steeringCommand,1))return 2;
  state=FP_REPLAN_BRAKE;stateStartMs=0;entryReplanReason="tight_pose_regression";
  bool contact=false;int ticks=0;
  for(;ticks<3000&&!final_parking_complete()&&!final_parking_aborted();++ticks){
   final_parking_update(direction);
   // Actual physical polygons, without the production planner's inflation.
   if(!parkingEntryPoseSafe(truePose,steeringCommand,0))contact=true;
   if(dc_state==DC_ENABLED){
    if(servo_disabled)return 3;
    float remaining=fabsf(command*.02f),sign=command<0?-1.f:1.f;
    while(remaining>.0001f){float ds=sign*fminf(1,remaining);remaining-=fabsf(ds);encoder+=ds;
     plant(hostPose,steeringCommand,ds,scale);plant(truePose,steeringCommand,ds,scale);
     if(!parkingEntryPoseSafe(truePose,steeringCommand,0))contact=true;
    }
   }hostTime+=20;
  }
  ++total;const bool ok=final_parking_complete()&&finalFootprintContained()&&!contact&&
                       robot_logger.writes==1&&steeringCommand==0;
  passed+=ok;
  if(!ok)std::cout<<"TIGHT FAIL dir/scale/anchor="<<direction<<"/"<<scale<<"/"<<anchor.x_mm<<"/"<<anchor.heading_deg
    <<" state/ticks/replans/contact="<<int(state)<<"/"<<ticks<<"/"<<int(entryReplans)<<"/"<<contact<<"\n";
 }
 std::cout<<"TIGHT COMPLETION "<<passed<<"/"<<total<<" PASS\n";return passed==total?0:1;
}
'''
    suffix = next((arg[2:] for arg in sys.argv[1:] if arg.startswith('--old-')), 'current')
    dest = ROOT / 'local_workspace' / ('parking-tight-' + suffix)
    dest.mkdir(parents=True, exist_ok=True)
    cpp, exe = dest / 'check.cpp', dest / 'check.exe'
    cpp.write_text(source)
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'include'),
                    '-I', str(ROOT/'local_workspace/later-laps'), str(cpp), '-o', str(exe)], check=True)
    result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=240)
    (dest/'output.txt').write_text(result.stdout + result.stderr)
    print(result.stdout, end='')
    raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
