"""Production camera selection, logged perception regressions and exit geometry.

No upload. Synthetic components do not reconstruct unrecorded camera frames.
"""
import shutil
import subprocess
from check_connector_servo_resume import block
from check_cw_start_planner import ROOT


def compile_run(name, source, extra=()):
    dest = ROOT / 'local_workspace/fix_512_514'
    dest.mkdir(parents=True, exist_ok=True)
    (dest/'Arduino.h').write_text("""#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <cmath>
using std::min; using std::max; using std::isfinite;
constexpr int A0=0,A1=1,A2=2;
constexpr double PI=3.141592653589793;
inline uint32_t micros(){return 0;}
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
template<class T> T constrain(T x,T a,T b){return x<a?a:(x>b?b:x);}
""")
    cpp, exe = dest / (name + '.cpp'), dest / (name + '.exe')
    cpp.write_text(source)
    run = subprocess.run([shutil.which('g++'), '-std=c++17', '-O2',
        '-I', str(dest),
        '-I', str(ROOT/'include'), str(cpp), *map(str, extra), '-o', str(exe)],
        text=True, capture_output=True)
    if run.returncode:
        print(run.stderr)
        raise SystemExit(run.returncode)
    run = subprocess.run([str(exe)], text=True, capture_output=True)
    (dest/(name+'_output.txt')).write_text(run.stdout + run.stderr)
    print(run.stdout, end='')
    if run.returncode:
        raise SystemExit(run.returncode)


def camera():
    source = (ROOT/'src/obstacle_vision.cpp').read_text()
    code = r'''#include "vision.h"
#include "obstacle_blob_validation.h"
#include <vector>
#include <iostream>
static Vision vision;
bool obstacle_blob_valid_for_acquisition(const Blob*b){return obstacle_blob_shape_valid(b);}
''' + block(source, source.index('const Blob *getLargestValidObstacle()'))
    code += r'''
void rect(std::vector<uint8_t>&b,int x0,int y0,int x1,int y1,uint16_t color){
 for(int y=y0;y<=y1;y++)for(int x=x0;x<=x1;x++){
  int i=((239-y)*320+319-x)*2;b[i]=color>>8;b[i+1]=color&255;
 }
}
int main(){vision.begin();int bad=0,n=0;
 for(bool green:{false,true})for(bool backgroundFirst:{false,true}){
  std::vector<uint8_t>b(320*240*2,255);
  uint16_t c=green?0x2244:0xa082;
  int pillarX=backgroundFirst?230:70;
  rect(b,backgroundFirst?0:180,80,backgroundFirst?130:319,210,c);
  rect(b,pillarX-16,80,pillarX+16,142,c);
  vision.update(b.data(),320,240);auto &r=vision.getResult();
  auto raw=green?r.green:r.red;auto candidate=green?r.greenPillar:r.redPillar;
  const Blob *selected=getLargestValidObstacle();
  ++n;if(!raw.found||obstacle_blob_shape_valid(&raw)||!candidate.found||
      !selected||selected->centerX!=pillarX||selected->color!=(green?ColorType::GREEN:ColorType::RED))++bad;
 }
 std::vector<uint8_t>blank(320*240*2,255);vision.update(blank.data(),320,240);
 ++n;if(getLargestValidObstacle()!=nullptr)++bad;
 vision.update(nullptr,320,240);++n;if(vision.getResult().greenPillar.found)++bad;
 std::cout<<"CAMERA COMPONENTS "<<n<<" failures "<<bad<<"\n";return bad?1:0;
}
'''
    compile_run('camera', code, [ROOT/'src/vision.cpp'])


def perception():
    source = (ROOT/'src/obstacle_path.cpp').read_text()
    code = r'''#include "obstacle_path.h"
#include "config.h"
#include <cmath>
#include <iostream>
''' + block(source, source.index('struct CandidateSeat')) + ';\n'
    code += 'CandidateSeat seats[OBSTACLE_SEAT_COUNT];\n'
    for signature in ('bool distantSeatSideAmbiguous(',
                      'bool unmappedForegroundBlocksClear(',
                      'bool closeUnmappedObstacle('):
        code += block(source, source.index(signature)) + '\n'
    # The independent unassigned-blob stop was superseded by station-aware
    # hold/recovery in521/522. Keep its geometry helper regression below;
    # no longer require that obsolete motor gate to exist.
    code += r'''
int main(){int bad=0,n=0;
 seats[10].x=-900;seats[10].y=500;seats[10].headingDeg=90;
 seats[11].x=-1100;seats[11].y=500;seats[11].headingDeg=90;
 //514: both early RED projections were within140mm of the wrong rail;
 // the old >700mm-only ambiguity gate accepted them at547/522mm.
 for(auto p:{std::initializer_list<float>{-1035,620,547},
             std::initializer_list<float>{-1032,610,522}}){
  auto i=p.begin();float x=*i++,y=*i++,r=*i;++n;
  if(!distantSeatSideAmbiguous(x,y,r,11))++bad;
 }
 ++n;if(distantSeatSideAmbiguous(-900,500,350,10))++bad;
 ObstacleObservationResult obs;obs.status=OBSTACLE_OBSERVATION_NO_BLOB;
 Blob raw;raw.found=true;raw.color=ColorType::GREEN;
 raw.minX=0;raw.maxX=26;raw.minY=80;raw.maxY=118;
 //512 t111401: an edge GREEN must veto both seats' EMPTY evidence.
 for(float bearing:{-25.06f,-6.36f}){
  ++n;if(!unmappedForegroundBlocksClear(&raw,obs,bearing,619.4f))++bad;
 }
 //514 t35521: giant low GREEN region was rejected but must retain UNKNOWN.
 raw.maxX=48;raw.minY=80;raw.maxY=238;
 ++n;if(!unmappedForegroundBlocksClear(&raw,obs,-7.79f,547.6f))++bad;
 // A shallow high wall strip and a truly distant region remain nonblocking.
 raw.maxX=200;raw.maxY=94;
 ++n;if(unmappedForegroundBlocksClear(&raw,obs,0,400))++bad;
 raw.maxX=26;raw.maxY=102;
 ++n;if(unmappedForegroundBlocksClear(&raw,obs,0,300))++bad;
 obs.productionValid=true;obs.status=OBSTACLE_OBSERVATION_NO_SEAT;obs.rangeMm=348;obs.bottom=150;obs.left=70;obs.right=110;
 ++n;if(!closeUnmappedObstacle(obs))++bad;
 obs.rangeMm=170;obs.bottom=104;obs.left=0;obs.right=40;
 ++n;if(closeUnmappedObstacle(obs))++bad;
 obs.bottom=150;obs.left=70;obs.right=110;
 obs.rangeMm=401;++n;if(closeUnmappedObstacle(obs))++bad;
 obs.rangeMm=300;obs.seatId=10;++n;if(closeUnmappedObstacle(obs))++bad;
 std::cout<<"LOGGED PERCEPTION "<<n<<" failures "<<bad<<"\n";return bad?1:0;
}
'''
    compile_run('perception', code)


def exit_geometry():
    source=(ROOT/'src/obstacle.cpp').read_text()
    code=r'''#include <Arduino.h>
#include "config.h"
#include "ackermann_kinematics.h"
#include "parking_start_footprint.h"
#include <iostream>
#include <cmath>
struct PositionEstimate{float x_mm,y_mm,heading_deg;};
''' + block(source,source.index('struct ParkingExitSegment')) + ';\n'
    at=source.index('static constexpr ParkingExitSegment PARKING_EXIT_SEGMENTS[')
    code+=source[at:source.index('};',at)+2]+'\n'
    code+=block(source,source.index('static bool parkingExitMotionSafe('))+'\n'
    code+=r'''
bool run(bool legacy,int turn,int mask,float dead,float gap,float dx,float dy,float dh){
 float x=(turn<0?390.f:570.f-gap)+dx,y=-1365+dy,h=(turn<0?180.f:0.f)+dh;
 for(int seg=0;seg<5;seg++){
  auto s=PARKING_EXIT_SEGMENTS[seg];
  if(legacy){float old[]={28,34,28,80,170};s.distanceMm=old[seg];}
  int steer=s.steeringRelativeToAway*(turn<0?50:-50);
  bool aligned=false;
  for(float travel=0;travel<=220;travel+=2){
   parking_start_footprint::Quad q[6];
   parking_start_footprint::robotQuads(x,y,h,(steer>0)-(steer<0),0,q);
   auto a=parking_start_footprint::rectangle(490,-1400,15,100,0);
   auto b=parking_start_footprint::rectangle(480-gap-10,-1400,15,100,0);
   for(auto &body:q){for(auto &p:body.p)if(p.y<=-1495)return false;
    if(parking_start_footprint::overlaps(body,a)||parking_start_footprint::overlaps(body,b))return false;}
   if(!legacy&&turn<0&&!parkingExitMotionSafe({x,y,h},steer,s.direction,0))return false;
   if(seg<4&&travel>=s.distanceMm)break;
   float error=remainderf(h-(turn<0?180.f:0.f),360.f);
   if(seg==4&&travel>=120&&fabsf(error)<=2){aligned=true;break;}
   if(travel>=220)return false;
   float factor=(mask&(1<<(seg%4)))?1.05f:.8f;
   float k=travel<dead?0:-factor/Ackermann::getTurnRadius(steer);
   float angle=h*PI/180,ds=s.direction*2.f,next=angle+k*ds;
   if(fabsf(k)>1e-6){x+=(sinf(next)-sinf(angle))/k;y+=(cosf(angle)-cosf(next))/k;}
   else{x+=ds*cosf(angle);y+=ds*sinf(angle);}h=next*180/PI;
  }
  if(seg==4&&!aligned)return false;
 }
 return true;
}
int main(){int n=0,bad=0,oldBad=0;
 for(int turn:{-1,1})for(int mask=0;mask<16;mask++)for(float dead:{0.f,4.f})
 for(float gap:{242.5f,252.5f})for(float dx:{-5.f,0.f,5.f})
 for(float dy:{-5.f,5.f,8.f})for(float dh:{-1.f,1.f}){
  ++n;if(!run(false,turn,mask,dead,gap,dx,dy,dh))++bad;
  if(!run(true,turn,mask,dead,gap,dx,dy,dh))++oldBad;
 }
 std::cout<<"EXIT INDEPENDENT "<<n<<" failures "<<bad<<" prior_failures "<<oldBad<<"\n";
 return bad||!oldBad?1:0;
}
'''
    compile_run('exit',code)


def settling():
    source=(ROOT/'src/obstacle.cpp').read_text()
    code=r'''#include <Arduino.h>
#include "config.h"
#include <iostream>
#include <limits>
struct PositionEstimate{float x_mm,y_mm,heading_deg;};
struct NullLog{template<class T>void print(T){}template<class T>void print(T,int){}
 template<class T>void println(T){}template<class T>void println(T,int){}} Serial;
enum {PARKING_EXIT_SEGMENT_BRAKE,PARKING_EXIT_SEGMENT_SETTLE,
 PARKING_EXIT_SEGMENT_DRIVE,PARKING_EXIT_TEST_HOLD,DC_DISABLED,DC_HOLDING};
int oc_parking_exit_state,oc_parking_exit_segment,oc_parking_exit_steering=50;
uint32_t now,oc_parking_exit_brake_start_ms,oc_parking_exit_settle_start_ms;
float measured_speed,oc_parking_exit_state_distance,oc_parking_exit_start_heading;
int dc_state;bool servo_disabled,aborted=false;
float encoder=42;
uint32_t millis(){return now;}
void stop(bool){dc_state=DC_DISABLED;}
void parking_exit_diagnostics_finish(const char*){aborted=true;}
void finishParkingExit(bool){}
float distanceSince(float start){return encoder-start;}
float get_angle(){return 0;}
float get_distance(){return encoder;}
void set_steering(int){}void steer(int){}
PositionEstimate get_position_struct(){return {350,-1150,180};}
// Geometry is tested separately above; isolate brake/settle sequencing here.
bool parkingExitMotionSafe(PositionEstimate,int,int,float){return true;}
''' + block(source,source.index('struct ParkingExitSegment')) + ';\n'
    at=source.index('static constexpr ParkingExitSegment PARKING_EXIT_SEGMENTS[')
    code+=source[at:source.index('};',at)+2]+'\n'
    code+='bool update(){const auto &segment=PARKING_EXIT_SEGMENTS[oc_parking_exit_segment];int segmentSteering=segment.steeringRelativeToAway*oc_parking_exit_steering;\n'
    for state in ('BRAKE','SETTLE'):
        signature='if (oc_parking_exit_state == PARKING_EXIT_SEGMENT_'+state+')'
        code+=block(source,source.index(signature))+'\n'
    code+='return false;}\n'
    code+=r'''
int main(){int bad=0,n=0;
 for(float speed:{9.f,-9.f,std::numeric_limits<float>::quiet_NaN()}){
  oc_parking_exit_state=PARKING_EXIT_SEGMENT_BRAKE;oc_parking_exit_segment=1;
  oc_parking_exit_brake_start_ms=0;now=151;measured_speed=speed;aborted=false;
  update();++n;if(oc_parking_exit_state!=PARKING_EXIT_SEGMENT_BRAKE||aborted)++bad;
  now=2700;update();++n;if(oc_parking_exit_state!=PARKING_EXIT_TEST_HOLD||!aborted)++bad;
 }
 oc_parking_exit_state=PARKING_EXIT_SEGMENT_BRAKE;oc_parking_exit_segment=1;
 oc_parking_exit_brake_start_ms=0;now=151;measured_speed=0;update();
 ++n;if(oc_parking_exit_state!=PARKING_EXIT_SEGMENT_SETTLE||oc_parking_exit_segment!=2)++bad;
 now=400;measured_speed=-57.68f;update();
 ++n;if(oc_parking_exit_state!=PARKING_EXIT_SEGMENT_SETTLE)++bad;
 now=500;measured_speed=0;encoder=37;update();
 ++n;if(oc_parking_exit_state!=PARKING_EXIT_SEGMENT_SETTLE)++bad;
 now=601;update();
 ++n;if(oc_parking_exit_state!=PARKING_EXIT_SEGMENT_DRIVE||oc_parking_exit_state_distance!=37)++bad;
 std::cout<<"EXIT BRAKE/STEERING "<<n<<" failures "<<bad<<"\n";return bad?1:0;
}
'''
    compile_run('settling',code)


if __name__=='__main__':
    camera()
    perception()
    exit_geometry()
    settling()
