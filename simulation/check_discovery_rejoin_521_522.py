"""Replay returned late-discovery poses through production CAD pursuit preflight.

No images or slip reconstructed. Includes measured stop and observation poses,
both directions, bounded pose perturbations and progress-index preservation.
"""
import shutil, subprocess
from pathlib import Path
from check_later_laps import later_lap_fixture
from check_connector_servo_resume import block

ROOT=Path(__file__).resolve().parents[1]
source=(ROOT/'src/obstacle_path.cpp').read_text()
fixture=later_lap_fixture().split('int main()')[0]
fixture=fixture.replace('struct Discovery {bool observedClear=false;};',
                        'struct Discovery {bool observedClear=false;bool seatObservedClear[2]={};};')
fixture+='\nbool earlyMiddleViewActive[OBSTACLE_SEAT_COUNT]={};\n'
fixture+='\nusing std::min; float runtimeSpeedCapMmS=0;\n'
for signature in ('float cappedPathSpeed(', 'bool earlyMiddleViewEligible(',
                  'bool earlyMiddleViewPathSafe(', 'void rebuildLivePath()'):
    fixture+=block(source,source.index(signature))+'\n'
fixture+=block(source,source.index('bool cornerViewSeatMayBeOccupied('))+'\n'
fixture+=block(source,source.index('bool discoveryResumePoseSafe('))+'\n'
fixture+=block(source,source.index('bool discoveryResumeFromViewSafe('))+'\n'
fixture='#include <cstdint>\nstatic uint32_t clearanceCalls=0;\n'+fixture.replace(
    '    sample = ObstacleClearanceSample{};',
    '    ++clearanceCalls;\n    sample = ObstacleClearanceSample{};')
fixture+=r'''
// Frozen525--527 exhaustive calculation: independent wall check for every
// seat. Compare its decisions with the deduplicated wall/pillar calculation.
bool exhaustivePoseSafe(const PositionEstimate &p){
 float h=p.heading_deg*PI/180;
 for(int rear=0;rear<2;++rear){float offset=rear?OBSTACLE_MAX_WHEEL_HALF_WIDTH_MM:0;
  for(uint8_t i=0;i<24;++i){ObstacleClearanceSample c{};
   if(!calculateClearanceAtPose(seats[i],p.x_mm-offset*cosf(h),p.y_mm-offset*sinf(h),p.heading_deg,c)||
      c.wallMm<=30||((seats[i].confirmed||(!seats[i^1U].confirmed&&cornerViewSeatMayBeOccupied(i)))&&c.pillarMm<=30))return false;
  }
 }return true;
}
int main(){int bad=0,n=0,accepted=0;float minBack=1e9;
 for(int dir:{-1,1})for(int lastSeat:{22,23}){
  routeTurnSign=dir;resetState();baseline();
  for(auto &seat:seats)seat.confirmed=seat.injected=false;
  for(auto &d:discoveryStations)for(bool &clear:d.seatObservedClear)clear=true;
  // Confirmed logged CW seats plus BOTH possible last GREEN rows: final
  // GREEN was not mapped in522. Reflection also swaps colours/pass sides.
  for(int i:{4,7,10,15,19,lastSeat}){
   seats[i].confirmed=seats[i].injected=true;
   seats[i].red=(i==4||i==10||i==19)^(dir>0);
   discoveryStations[i/2].seatObservedClear[i%2]=false;
   discoveryStations[i/2].seatObservedClear[(i%2)^1]=false;
  }
  rebuildLivePath();
  for(float x=-1400;x<=1400;x+=280)for(float y=-1400;y<=1400;y+=280)
  for(float h=0;h<360;h+=60){PositionEstimate p{x,y,h};
   bool reference=exhaustivePoseSafe(p);clearanceCalls=0;
   bool actual=discoveryResumePoseSafe(p);++n;
   if(actual!=reference||clearanceCalls>2)++bad;
  }
  for(int scene:{0,1})for(float dx:{-5.f,0.f,5.f})for(float dy:{-5.f,0.f,5.f})
  for(float dh:{-1.f,0.f,1.f}){
   PositionEstimate hold=scene==0?PositionEstimate{-870.5f,-708,142.05f}:PositionEstimate{1008.4f,-188.3f,270.32f};
   PositionEstimate view=scene==0?PositionEstimate{-765.8f,-790.4f,141.15f}:hold;
   if(scene==1){float h=hold.heading_deg*PI/180;view.x_mm-=210*cosf(h);view.y_mm-=210*sinf(h);}
   if(dir>0){hold.x_mm=-hold.x_mm;hold.heading_deg=180-hold.heading_deg;
             view.x_mm=-view.x_mm;view.heading_deg=180-view.heading_deg;}
   hold.x_mm+=dx;hold.y_mm+=dy;hold.heading_deg+=dh;
   view.x_mm+=dx;view.y_mm+=dy;view.heading_deg+=dh;
   progressIndex=nearestPathIndex(livePath,hold.x_mm,hold.y_mm,0,pathLength);
   const uint16_t saved=progressIndex;uint16_t index=0;
   bool oldSafe=discoveryResumeFromViewSafe(hold,index);
   ++n;if(progressIndex!=saved)++bad;
   clearanceCalls=0;bool ok=discoveryResumeFromViewSafe(view,index);
   ++n;if(clearanceCalls>282)++bad;
   ++n;if(progressIndex!=saved)++bad;
   if(ok){++accepted;float back=cyclicDistanceForward(baselinePath[index].distanceMm,baselinePath[saved].distanceMm);
          if(back<minBack)minBack=back;}
   // The nominal reported CW poses must actually recover, rather than merely
   // rejecting both choices and claiming success from a new permanent hold.
   if(dir<0 && dx==0 && dy==0 && dh==0){++n;if(!ok||(scene==0&&oldSafe))++bad;
     std::cout<<"nominal scene/lastSeat "<<scene<<"/"<<lastSeat<<" old/new "<<oldSafe<<"/"<<ok<<"\n";}
  }
 }
 ++n;if(accepted!=216)++bad;
 std::cout<<"REJOIN "<<n<<" checks failures "<<bad<<" accepted_views "<<accepted<<" min_progress_rebase_mm "<<minBack<<"\n";
 return bad?1:0;
}
'''
dest=ROOT/'local_workspace/fix_521_522';dest.mkdir(parents=True,exist_ok=True)
(dest/'rejoin.cpp').write_text(fixture)
subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),
               '-I',str(ROOT/'simulation/host'),str(dest/'rejoin.cpp'),'-o',str(dest/'rejoin.exe')],check=True)
run=subprocess.run([str(dest/'rejoin.exe')],capture_output=True,text=True)
(dest/'rejoin_output.txt').write_text(run.stdout+run.stderr)
print(run.stdout,end='');raise SystemExit(run.returncode)
