"""Run production scan classification against independent planar cone geometry.

Mixed edge returns reproduce the gap overestimate in logs 531/533/534.
This is a source-level mechanism test, not a replay of unavailable ToF frames.
"""
import shutil
import subprocess

from check_cw_start_planner import ROOT
from check_final_parking_sequence import fixture_source


def main():
    fixture = fixture_source()
    fixture += r'''
// Independent sensor model: intersect rays with the front face of both
// physical pieces; do not call either production footprint function.
void physicalCone(int dir, float &lo, float &hi) {
 const double h=hostPose.heading_deg*3.141592653589793/180;
 const double lx=40, ly=dir>0?-35:35;
 const double sx=hostPose.x_mm+cos(h)*lx-sin(h)*ly;
 const double sy=hostPose.y_mm+sin(h)*lx+cos(h)*ly;
 const double a=h+(dir>0?-1:1)*3.141592653589793/2;
 double x[2];int i=0;
 for(double offset:{-11.,11.}) {
  const double angle=a+offset*3.141592653589793/180;
  x[i++]=sx+(-1300-sy)*cos(angle)/sin(angle);
 }
 lo=std::min(x[0],x[1]);hi=std::max(x[0],x[1]);
}
struct Result {bool accepted;float gap,legacyGap,xCorrection;int phase;};
Result scan(int dir,float actualGap,float heading,float y,float bias,float mixed) {
 parking_module::final_parking_reset();
 parking_module::routeTurnSign=dir;
 parking_module::outerSensor=dir>0?TOF_RIGHT:TOF_LEFT;
 parking_module::scanPhase=0;hostTime=0;
 float legacyFixed=0,legacyMoving=0;
 for(float x=800;x>100;x-=.6f) {
  hostPose={x,y,(dir>0?0.f:180.f)+heading};
  float lo,hi;physicalCone(dir,lo,hi);
  bool fixed=hi>=480&&lo<=500;
  bool moving=hi>=480-actualGap-20&&lo<=480-actualGap;
  bool marker=fixed||moving;
  float range=parking_module::expectedOuterWallRange(parking_module::outerSensor,hostPose);
  if(marker) {
   // A 2-mm boundary region yields a valid but mixed 170-mm reading,
   // whereas the middle of a piece returns its actual ~60-mm distance.
   bool edge=fixed?(hi-480<2||500-lo<2):(hi-(480-actualGap-20)<2||480-actualGap-lo<2);
   range=edge&&mixed>0?mixed:range-200;
  }
  hostPose.x_mm+=bias;
  TofDiagnosticSnapshot s{};s.sequence=++hostTime;s.sampled_ms=hostTime;
  s.stored_object_count=1;s.objects[0].distance_mm=(int16_t)range;
  const int phase=parking_module::scanPhase;
  parking_module::processScanFrame(s);
  if(phase==2&&parking_module::scanPhase==3)
   legacyFixed=parking_module::beamFootprint(parking_module::outerSensor,
    parking_module::lastFixedMarkerRange,parking_module::lastFixedMarkerPose).maximumX;
  if(phase==3&&parking_module::scanPhase==4)
   legacyMoving=parking_module::beamFootprint(parking_module::outerSensor,
    parking_module::firstMovingMarkerRange,parking_module::firstMovingMarkerPose).minimumX;
  if(parking_module::scanPhase==5)break;
 }
 float before=hostPose.x_mm;
 bool ok=parking_module::applyScanLocalization();
 return {ok,parking_module::measuredGapMm,legacyFixed-legacyMoving,hostPose.x_mm-before,parking_module::scanPhase};
}
int main(){int total=0,failed=0,negative=0;
 for(int dir:{-1,1})for(float gap:{242.5f,247.5f,252.5f,280.f})
 for(float heading:{-3.f,0.f,3.f})for(float y:{-1215.f,-1205.4f,-1195.f})
 for(float bias:{-180.f,0.f,180.f})for(float mixed:{0.f,150.f,170.f,180.f}) {
  auto r=scan(dir,gap,heading,y,bias,mixed);++total;
  const bool expect=gap<260;
  bool safe=r.phase==5&&r.accepted==expect&&fabsf(r.gap-gap)<2;
  if(expect)safe=safe&&fabsf(r.xCorrection+bias)<2;
  if(!safe){++failed;if(failed<8)std::cout<<"FAIL "<<dir<<" "<<gap<<" "<<heading<<" "<<y<<" "<<bias<<" "<<mixed<<" result "<<r.accepted<<" "<<r.gap<<" "<<r.phase<<"\n";}
  if(expect&&mixed==170&&fabsf(r.legacyGap-247.5f)>15)++negative;
 }
 // Bad beam direction and nonfinite pose must never become an edge.
 hostPose={0,-1205.4f,NAN};auto bad=parking_module::markerEdgeFootprint(TOF_LEFT,hostPose);
 if(std::isfinite(bad.minimumX)||std::isfinite(bad.maximumX))++failed;
 hostPose={0,-1205.4f,0};bad=parking_module::markerEdgeFootprint(TOF_LEFT,hostPose);
 if(std::isfinite(bad.minimumX)||std::isfinite(bad.maximumX))++failed;
 std::cout<<"MIXED EDGE "<<total<<" FAILED "<<failed<<" LEGACY REJECTIONS "<<negative<<"\n";
 return failed||!negative?1:0;
}
'''
    dest = ROOT/'local_workspace/parking-mixed-edges'
    dest.mkdir(exist_ok=True)
    cpp, exe = dest/'check.cpp', dest/'check.exe'
    cpp.write_text(fixture)
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', '-I', str(ROOT/'include'),
                    '-I', str(ROOT/'local_workspace/later-laps'), str(cpp), '-o', str(exe)], check=True)
    result = subprocess.run([str(exe)], capture_output=True, text=True)
    (dest/'output.txt').write_text(result.stdout)
    print(result.stdout)
    raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
