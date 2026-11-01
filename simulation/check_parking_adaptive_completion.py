"""Production final parking with recorded entry anchors and curvature error.

The plant uses a distinct radius from the planning model and reports its yaw
through the pose estimator. This tests correction of measured motion error;
it does not model an unknown gyro bias or prove physical contact-free parking.
Gap sizes cover the full accepted measured range, including its endpoints.
"""
from pathlib import Path
import shutil
import subprocess
import sys

from check_final_parking_sequence import ROOT, fixture_source


def main():
    source=fixture_source()+r'''
using namespace parking_module;
void plant(PositionEstimate &p,int steering,float ds,float scale){
 const float r=Ackermann::getTurnRadius(float(steering));
 const float k=fabsf(float(steering))<2 ? 0:-1.f/(r*scale);
 const float h=p.heading_deg*PI/180.f,next=h+k*ds;
 if(fabsf(k)>1e-6f){p.x_mm+=(sinf(next)-sinf(h))/k;p.y_mm+=(cosf(h)-cosf(next))/k;}
 else{p.x_mm+=ds*cosf(h);p.y_mm+=ds*sinf(h);}
 p.heading_deg=parking_module::wrap180(next*180.f/PI);
}
int main(){int total=0,parked=0,failed=0;float maxTravel=0;int maxReplans=0;
 for(float gap:{232.5f,242.5f,247.5f,252.5f,262.5f})
 for(int direction:{-1,1})for(float scale:{.95f,1.f,1.05f,1.10f,1.15f})
 for(auto anchor:{PositionEstimate{270.26f,294.6f,0},
                  PositionEstimate{269.1f,296.f,1.4f},
                  PositionEstimate{147.7f,245.4f,57.1f},
                  PositionEstimate{105.f,177.6f,58.5f},
                  PositionEstimate{74.8f,143.f,38.2f},
                  PositionEstimate{81.25f,120.f,8.f},
                  PositionEstimate{81.25f,120.f,-8.f}}){
  ::routeTurnSign=direction;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  final_parking_reset();parking_module::routeTurnSign=direction;scanPhase=5;measuredGapMm=gap;
  localToField(anchor.x_mm,anchor.y_mm,anchor.heading_deg,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
  truePose=hostPose;hostTime=1000;encoder=0;hostGyroHealthy=true;framePeriod=30;
  stop(false);robot_logger.lines.str("");robot_logger.lines.clear();robot_logger.writes=0;
  state=FP_REPLAN_BRAKE;stateStartMs=0;entryReplanReason="recorded_anchor";
  bool collision=false;float travel=0;
  for(int tick=0;tick<6000&&!final_parking_complete()&&!final_parking_aborted();++tick){
   final_parking_update(direction);
   if(dc_state==DC_ENABLED){
    if(servo_disabled)return 2;
    float remaining=fabsf(command*.02f),sign=command<0?-1.f:1.f;
    while(remaining>.0001f){float ds=sign*fminf(1,remaining);remaining-=fabsf(ds);encoder+=ds;travel+=fabsf(ds);
     plant(hostPose,steeringCommand,ds,scale);plant(truePose,steeringCommand,ds,scale);
     if(!parkingEntryPoseSafe(truePose,steeringCommand)){
      if(!collision)std::cout<<"COLLISION state/segment/pose/margin="<<int(state)<<"/"<<int(segmentIndex)<<"/"<<truePose.x_mm<<"/"<<truePose.y_mm<<"/"<<truePose.heading_deg<<"/"<<entrySafetyMargin<<"\n";
      collision=true;
     }
    }
   }hostTime+=20;
  }
  ++total;bool ok=final_parking_complete()&&finalFootprintContained()&&!collision&&robot_logger.writes==1&&steeringCommand==0;
  parked+=ok;failed+=!ok;maxTravel=fmaxf(maxTravel,travel);maxReplans=std::max(maxReplans,int(entryReplans));
  if(!ok)std::cout<<"ADAPTIVE FAIL gap="<<gap<<" dir/scale/anchor="<<direction<<"/"<<scale<<"/"<<anchor.x_mm<<"/"<<anchor.y_mm<<"/"<<anchor.heading_deg<<" collision="<<collision<<"\n"<<robot_logger.lines.str();
 }
 int health=0;
 for(int dir:{-1,1})for(auto phase:{FP_REPLAN_BRAKE,FP_VERIFY}){
  final_parking_reset();parking_module::routeTurnSign=dir;scanPhase=5;measuredGapMm=247.5f;
  localToField(81.25f,120.f,0,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);truePose=hostPose;
  hostTime=1000;stateStartMs=100;state=phase;hostGyroHealthy=false;set_speed(40);
  final_parking_update(dir);++health;
  if(dc_state!=DC_DISABLED||state!=phase||final_parking_aborted()||final_parking_complete())return 3;
  hostTime+=500;hostGyroHealthy=true;final_parking_sensor_resume();
  if(stateStartMs!=600)return 4;
  hostPose.x_mm=NAN;final_parking_update(dir);++health;
  if(!final_parking_aborted()||dc_state!=DC_DISABLED||final_parking_complete())return 5;
 }
 std::cout<<"ADAPTIVE HEALTH "<<health<<" PASS\n";
 std::cout<<"ADAPTIVE "<<total<<" PARKED "<<parked<<" FAILED "<<failed<<" max_travel/replans="<<maxTravel<<"/"<<maxReplans<<"\n";
 return failed?1:0;
}
'''
    if '--quick' in sys.argv:
        source=source.replace('for(float gap:{232.5f,242.5f,247.5f,252.5f,262.5f})',
                              'for(float gap:{247.5f})')
        source=source.replace('for(int direction:{-1,1})for(float scale:{.95f,1.f,1.05f,1.10f,1.15f})',
                              'for(int direction:{1})for(float scale:{1.15f})')
    if '--extended-radius' in sys.argv:
        source=source.replace('for(float gap:{232.5f,242.5f,247.5f,252.5f,262.5f})',
                              'for(float gap:{247.5f})')
        source=source.replace('for(float scale:{.95f,1.f,1.05f,1.10f,1.15f})',
                              'for(float scale:{1.30f,1.45f})')
        # Deep partial entries have already supplied a measured preceding arc.
        # Retain a 3% estimate error instead of granting exact plant knowledge.
        source=source.replace('state=FP_REPLAN_BRAKE;stateStartMs=0;',
            'if(anchor.y_mm<200){entryRadiusScale=scale*.97f;entryRadiusMeasured=true;}\n  state=FP_REPLAN_BRAKE;stateStartMs=0;')
    if '--brake-overrun' in sys.argv:
        # Deliberately pessimistic settled overrun at every drive-to-hold
        # transition. Encoder and gyro both observe the extra motion; this is
        # a host sensitivity test, not a measured robot deceleration model.
        source=source.replace('for(float gap:{232.5f,242.5f,247.5f,252.5f,262.5f})',
                              'for(float overrun:{0.f,1.5f,3.f})for(float gap:{247.5f})')
        source=source.replace('for(float scale:{.95f,1.f,1.05f,1.10f,1.15f})',
                              'for(float scale:{1.f,1.15f})')
        source=source.replace('   final_parking_update(direction);',r'''
   const bool wasMoving=dc_state==DC_ENABLED;
   const int oldCommand=command,oldSteering=steeringCommand;
   final_parking_update(direction);
   if(wasMoving && dc_state!=DC_ENABLED){
    for(float left=overrun;left>.0001f;){float step=fminf(1.f,left);left-=step;
     float ds=oldCommand<0?-step:step;encoder+=ds;travel+=step;
     plant(hostPose,oldSteering,ds,scale);plant(truePose,oldSteering,ds,scale);
     if(!parkingEntryPoseSafe(truePose,oldSteering,0))collision=true;
    }
   }''')
    if '--without-brake-lead' in sys.argv:
        source=source.replace(' ? entryBrakeTravelMm:0.0f;', ' ? 0.0f:0.0f;')
    dest=ROOT/'local_workspace'/('parking-adaptive-quick' if '--quick' in sys.argv else 'parking-adaptive-completion')
    if '--extended-radius' in sys.argv: dest=ROOT/'local_workspace/parking-adaptive-extended'
    if '--brake-overrun' in sys.argv: dest=ROOT/'local_workspace/parking-adaptive-brake-overrun'
    if '--without-brake-lead' in sys.argv: dest=ROOT/'local_workspace/parking-adaptive-brake-negative'
    dest.mkdir(parents=True,exist_ok=True)
    cpp,exe=dest/'check.cpp',dest/'check.exe'
    cpp.write_text(source)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),
                    '-I',str(ROOT/'local_workspace/later-laps'),str(cpp),'-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],capture_output=True,text=True,timeout=240)
    (dest/'output.txt').write_text(result.stdout+result.stderr)
    print('\n'.join(line for line in result.stdout.splitlines()
                    if line.startswith(('ADAPTIVE FAIL','ADAPTIVE ','COLLISION'))))
    raise SystemExit(result.returncode)


if __name__=='__main__':
    main()
