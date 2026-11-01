"""Returned620/621 coverage and mirrored recorded parking anchors (host only)."""
import re
from collections import Counter
from check_final_parking_sequence import ROOT, fixture_source
from check_returned_603_and_parking_practice import run
from check_connector_servo_resume import block


def main():
    for n in (620,621):
        text=(ROOT/f'simulation/evidence/parking_exit_diagnostics/20261031_log_{n}_cw.txt').read_text()
        poses=list(re.finditer(r'\[RUN_POSE\] v=2 t=(\d+) lap=(\d+).* tof=',text))
        times=[int(m[1]) for m in poses]
        print(n,'poses',len(times),'laps',dict(Counter(m[2] for m in poses)),
              'max_gap_ms',max(b-a for a,b in zip(times,times[1:])),
              'obstacles',text.count('[RUN_OBSTACLE]'),'sync',text.count('[RUN_SYNC]'))
        assert '[RUN_END]' in text and 'truncated=0' in text
        assert all(b>a for a,b in zip(times,times[1:]))
    # Reuse the actual production state-machine/independent plant fixture,
    # replacing the generic anchors with this run's stopped recorded poses.
    script=(ROOT/'simulation/check_parking_adaptive_completion.py').read_text()
    code=script.split("source=fixture_source()+r'''",1)[1].split("'''",1)[0]
    code=code.replace('for(float gap:{232.5f,242.5f,247.5f,252.5f,262.5f})',
                      'for(float gap:{249.2f})')
    code=code.replace('for(float scale:{.95f,1.f,1.05f,1.10f,1.15f})',
                      'for(float scale:{1.104f,1.120f})')
    begin=code.index(' for(auto anchor:')
    end=code.index('){',begin)+2
    code=code[:begin]+''' for(auto anchor:{PositionEstimate{269.9f,295.7f,-.5f},
       PositionEstimate{161.4f,247.4f,55.1f},
       PositionEstimate{70.0f,119.0f,39.4f}}){'''+code[end:]
    code=code.replace('state=FP_REPLAN_BRAKE;stateStartMs=0;',
        'if(anchor.y_mm<260){entryRadiusScale=1.104f;entryRadiusMeasured=true;}\n  state=FP_REPLAN_BRAKE;stateStartMs=0;')
    run('620-621-parking',fixture_source()+code)
    code=code.replace('entryReplanReason="recorded_anchor";',
        'entryReplanReason="entry_motion_clearance";entryReplans=2;')
    code=code.replace('int total=0,parked=0,failed=0;', 'int total=0,parked=0,failed=0,rearEscapes=0;')
    code=code.replace('++total;bool ok=',
        'rearEscapes+=robot_logger.lines.str().find("Checked10mm rearward escape")!=std::string::npos;\n  ++total;bool ok=')
    code=code.replace('return failed?1:0;',
        'std::cout<<"AUTOMATIC REAR PREFIX "<<rearEscapes<<"\\n";return (failed||rearEscapes)?1:0;')
    run('621-rearward-escape',fixture_source()+code)
    run('normal-rear-room',fixture_source()+r'''
using namespace parking_module;
int main(){int selected=0;
 for(int dir:{-1,1})for(auto p:{PositionEstimate{161.4f,247.4f,55.1f},PositionEstimate{70.f,119.f,39.4f}}){
  ::routeTurnSign=dir;resetState();baseline();for(auto&s:seats)s.confirmed=s.injected=false;
  final_parking_reset();parking_module::routeTurnSign=dir;scanPhase=5;measuredGapMm=249.2f;
  entryRadiusScale=1.104f;entryRadiusMeasured=true;entryReplans=0;entryReplanReason=nullptr;
  localToField(p.x_mm,p.y_mm,p.heading_deg,hostPose.x_mm,hostPose.y_mm,hostPose.heading_deg);
  robot_logger.lines.str("");robot_logger.lines.clear();
  if(!planParkingEntry(hostPose)){std::cout<<"NORMAL PLAN FAIL "<<dir<<"/"<<p.x_mm<<"/"<<p.y_mm<<"/"<<p.heading_deg<<"\n"<<robot_logger.lines.str();return 1;}
  if(robot_logger.lines.str().find("Checked10mm rearward escape")!=std::string::npos){
   ++selected;std::cout<<"NORMAL FIRST "<<dir<<"/"<<p.y_mm<<" "<<int(adaptiveEntry[0].direction)<<"/"<<int(adaptiveEntry[0].steering)<<"/"<<adaptiveEntry[0].distanceMm<<"\n";
  }
  if(OBSTACLE_FINAL_PARKING_TARGET_LOCAL_X_MM!=60)return 3;
 }
 std::cout<<"NORMAL PARKING "<<selected<<" automatic extra10mm prefixes; final goal unchanged\n";
 return selected?4:0;
}
''')
    path=(ROOT/'src/obstacle_path.cpp').read_text()
    branch=block(path,path.index('    if(cornerViewPhase==CORNER_VIEW_REBASE_BRAKE)'))
    run('620-brake-rebase',r'''
#include <cmath>
#include <iostream>
using std::isfinite;
struct PositionEstimate{float x_mm=0,y_mm=0,heading_deg=0;};
PositionEstimate pose,cornerViewOrigin;
enum{CORNER_VIEW_REBASE_BRAKE,CORNER_VIEW_OBSERVE};
int cornerViewPhase=0,cornerViewStation=3,cornerViewStoppedFrames=0;
float encoder=0,cornerViewOriginEncoder=0,cornerViewDistanceMm=0,cornerViewMeasuredReverseMm=0;
unsigned now=0,cornerViewPhaseMs=0;bool locked=false;
struct S{int clearFrames[2]={};}discoveryStations[12];
float wrap180(float a){while(a>180)a-=360;while(a<-180)a+=360;return a;}
void holdCornerViewCentered(){} unsigned millis(){return now;}
PositionEstimate get_position_struct(){return pose;}float get_distance(){return encoder;}
void lockCornerView(const char*){locked=true;}
void logCornerViewPose(const char*,PositionEstimate,float){}
void tick(){
'''+branch+r'''
}
int main(){
 for(unsigned t:{10U,100U,299U}){now=t;pose={5,0,6};encoder=-6;tick();
  if(locked||cornerViewPhase!=CORNER_VIEW_REBASE_BRAKE)return 1;}
 now=300;tick();if(locked||cornerViewPhase!=CORNER_VIEW_OBSERVE||
   cornerViewOrigin.heading_deg!=6||cornerViewOriginEncoder!=-6)return 2;
 cornerViewPhase=CORNER_VIEW_REBASE_BRAKE;cornerViewOrigin={};pose={21,0,0};tick();
 if(!locked)return 3;
 std::cout<<"620 brake/rebase PASS: settles6deg, then observes; excessive drift stops\n";
}
''')


if __name__=='__main__':
    main()
