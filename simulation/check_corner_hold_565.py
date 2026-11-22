"""565 corner hold: production empty-evidence gates and safe route rejoin.
Synthetic RGB565 scenes are controlled hypotheses, not absent original images.
"""
import shutil,subprocess
from check_connector_servo_resume import ROOT,block
from check_far_obstacle_acquisition import fixture_source
from check_later_laps import later_lap_fixture


def run(name,code,extra=(),expected=0):
 out=ROOT/'local_workspace/corner-hold-565';out.mkdir(exist_ok=True)
 cpp,exe=out/(name+'.cpp'),out/(name+'.exe');cpp.write_text(code)
 subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-include','cstdint','-include','cstring','-I',str(ROOT/'simulation/host'),
  '-I',str(ROOT/'include'),str(cpp),*map(str,extra),'-o',str(exe)],check=True)
 r=subprocess.run([str(exe)],text=True,capture_output=True)
 (out/(name+'.txt')).write_text(r.stdout+r.stderr)
 print(r.stdout,end='')
 if r.returncode!=expected:raise SystemExit(r.returncode or 99)
 if expected:print('Negative control reproduces the unresolved station')


def main():
 path=(ROOT/'src/obstacle_path.cpp').read_text()
 code=fixture_source()+ '\n'+block(path,path.index('struct DiscoveryStation'))+';\n'
 code+=r'''
DiscoveryStation discoveryStations[12];
bool greenSeatCandidateThisFrame[OBSTACLE_SEAT_COUNT]={};
uint32_t greenSeatCandidateLastSeenMs[OBSTACLE_SEAT_COUNT]={};
int parkingEntryTargetStation=-1,clearDispatch=0;
constexpr int CORNER_VIEW_IDLE=0;int cornerViewPhase=1,cornerViewStation=6;
constexpr int COURSE_STATIONS_PER_SECTION=3;
bool stationResolved(uint8_t s){return s!=6 || discoveryStations[s].observedClear;}
void logParkingStartClearImage(uint8_t,const PositionEstimate &){}
void course_map_record_clear_station(int,int,float,float,float,float){++clearDispatch;}
'''
 for sig in ('float wrap180(', 'void seatCameraGeometry(', 'float cameraBearingForImageX(',
             'bool observationAllowsClearAtGeometry(', 'bool confirmedPillarOccludesSeat(', 'bool rejectedGreenBehindSeat(',
             'bool unmappedForegroundBlocksClear(', 'bool rejectedBlobBlocksSeatClear(',
             'bool seatComfortablyVisible(', 'void updateDiscoveryCoverage('):
  code+=block(path,path.rindex(sig))+'\n'
 code+=r'''
static uint8_t pixels[320*240*2];static Vision testVision;
void put(int x,int y,uint16_t c){
 int sx=Vision::rotates180()?319-x:x,sy=Vision::rotates180()?239-y:y;
 int i=(sy*320+sx)*2;pixels[i+(Vision::rgb565MsbFirst()?0:1)]=c>>8;
 pixels[i+(Vision::rgb565MsbFirst()?1:0)]=c&255;
}
int main(){testVision.begin();int total=0;
 // Full upper silhouettes from512/514 retain their bearing-independent veto.
 Blob raw{};raw.found=true;raw.color=ColorType::GREEN;raw.minX=0;raw.maxX=26;
 raw.minY=80;raw.maxY=118;ObstacleObservationResult obs{};
 for(float b:{-25.06f,-6.36f}){if(!unmappedForegroundBlocksClear(&raw,obs,b,619.4f))return 1;++total;}
 raw.maxX=48;raw.maxY=238;
 if(!unmappedForegroundBlocksClear(&raw,obs,-7.79f,547.6f))return 2;
 for(int shifted:{0,1})for(int occupied:{0,1})for(int fullUpper:{0,1}){
  for(auto &d:discoveryStations)d={};discoveryStations[6].seatObservedClear[0]=true;
  for(auto &v:greenSeatCandidateThisFrame)v=false;
  seats[12].x=seats[13].x=-500;seats[12].y=900;seats[13].y=1100;
  hostPose={-782.7f,771.1f,54.29f};float bearing,range;
  seatCameraGeometry(13,hostPose,bearing,range);
  int expectedX=int(lroundf(164.4f-248.9f*tanf(bearing*PI/180)));
  int foot=int(lroundf(52+33600/(range*cosf(bearing*PI/180))));
  raw={};raw.found=true;raw.color=ColorType::GREEN;
  raw.centerX=shifted?expectedX:273;raw.minX=raw.centerX-9;raw.maxX=raw.centerX+9;
  raw.minY=fullUpper?80:148;raw.maxY=192;raw.area=400;
  for(int y=0;y<240;++y)for(int x=0;x<320;++x)put(x,y,0xffff);
  for(int y=raw.minY;y<=raw.maxY;++y)for(int x=raw.minX;x<=raw.maxX;++x)put(x,y,0x2244);
  if(occupied)for(int y=80;y<=foot;++y)for(int x=expectedX-14;x<=expectedX+14;++x)put(x,y,0x18c3);
  GreenSeatCandidate candidate;testVision.findGreenSeatCandidate(pixels,320,240,expectedX,foot,candidate);
  greenSeatCandidateThisFrame[13]=candidate.silhouetteFound;
  if(occupied && !candidate.silhouetteFound)return 3;
  for(int frame=0;frame<3;++frame)updateDiscoveryCoverage(obs,hostPose,&raw);
  bool shouldClear=!shifted && !occupied && !fullUpper;
  if(discoveryStations[6].observedClear!=shouldClear){
   std::cout<<"565 CLEAR FAIL "<<shifted<<" "<<occupied<<" "<<fullUpper<<" bearing/range "<<bearing<<"/"<<range<<"\n";return 4;}
  ++total;
 }
 std::cout<<"565 EVIDENCE "<<total<<" PASS; unrelated lower fragment clears only with empty local scene\n";
}
'''
 code+='\n#include <algorithm>\nusing std::min;using std::max;\nuint32_t micros(){return clockMs*1000;}\n'
 code+=(ROOT/'src/vision.cpp').read_text()
 run('coverage',code)
 frozen=code.replace('if (raw->minY>OBSTACLE_MAX_TOP_Y && isfinite(bearing))',
                     'if (false)')
 run('old_global_veto',frozen,expected=4)
 geo=later_lap_fixture().split('int main()')[0]
 geo=geo.replace('struct Discovery {bool observedClear=false;};',
  'struct Discovery {bool observedClear=false;bool seatObservedClear[2]={};};')
 geo+='\nusing std::min;float runtimeSpeedCapMmS=0;bool earlyMiddleViewActive[OBSTACLE_SEAT_COUNT]={};\n'
 geo+=block(path,path.index('float cappedPathSpeed('))+'\n'
 for sig in ('bool earlyMiddleViewEligible(', 'bool earlyMiddleViewPathSafe(', 'void rebuildLivePath()',
             'bool cornerViewSeatMayBeOccupied(', 'bool discoveryResumePoseSafe(',
             'bool discoveryResumeFromViewSafe('):
  geo+=block(path,path.index(sig))+'\n'
 geo+=r'''
int main(){int count=0;
 for(float dx:{-5.f,0.f,5.f})for(float dy:{-5.f,0.f,5.f})for(float dh:{-.5f,0.f,.5f}){
  routeTurnSign=-1;resetState();baseline();
  for(auto &s:seats)s.confirmed=s.injected=false;
  for(auto &d:discoveryStations){d.observedClear=false;d.seatObservedClear[0]=d.seatObservedClear[1]=false;}
  for(int i:{0,4,7,10}){seats[i].confirmed=seats[i].injected=true;seats[i].red=i==4||i==10;}
  discoveryStations[6].observedClear=true;
  discoveryStations[6].seatObservedClear[0]=discoveryStations[6].seatObservedClear[1]=true;
  rebuildLivePath();PositionEstimate p{-782.7f+dx,771.1f+dy,54.29f+dh};
  progressIndex=nearestPathIndex(livePath,-718.3f,860.6f,0,pathLength);
  uint16_t index=0;
  if(!discoveryResumeFromViewSafe(p,index)){
   std::cout<<"565 REJOIN FAIL "<<dx<<" "<<dy<<" "<<dh<<"\n";return 5;}
  ++count;
 }
 std::cout<<"565 REJOIN "<<count<<" perturbed poses PASS with production700mm CAD check\n";
}
'''
 run('rejoin',geo)


if __name__=='__main__':main()
