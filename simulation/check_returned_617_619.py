"""617 inward tracking vs618, plus rotations/travel directions; host only."""
from check_connector_servo_resume import block
from check_final_parking_sequence import ROOT
from check_returned_603_and_parking_practice import run


def main():
    source=(ROOT/'src/obstacle_path.cpp').read_text()
    funcs='\n'.join(block(source,source.index(sig)) for sig in
        ('float connectorRouteHeading(', 'float laterLapInwardTrackingSpeed('))
    run('617-inward-speed',r'''
#include <cmath>
#include <cstdint>
#include <iostream>
using std::isfinite;
constexpr float PI=3.14159265358979323846f;
constexpr float OBSTACLE_LATER_LAP_RESPONSE_S=.40f;
struct PositionEstimate{float x_mm,y_mm,heading_deg;};
struct PathPoint{float x,y;};uint16_t pathLength=3;
''' +funcs+r'''
PositionEstimate rotated(PositionEstimate p,float angle){
 float h=angle*PI/180;return {p.x_mm*cosf(h)-p.y_mm*sinf(h),
 p.x_mm*sinf(h)+p.y_mm*cosf(h),p.heading_deg+angle};}
int main(){int checks=0;
 for(float angle:{0.f,90.f,180.f,270.f})for(bool reverse:{false,true}){
  PathPoint route[3];
  for(int i=0;i<3;++i){auto p=rotated({-690.f,200.f+(reverse?-1:1)*i*200.f,0},angle);route[i]={p.x_mm,p.y_mm};}
  for(float speed:{100.f,260.f,436.f,520.f}){
   const float aligned=reverse?-90.f:90.f;
   auto good=rotated({-690,200,aligned},angle);
   if(fabsf(laterLapInwardTrackingSpeed(speed,good,route,0)-speed)>.01f)return 1;
   auto bad=rotated({-651.3f,318.2f,reverse?-54.6f:54.6f},angle);
   float limited=laterLapInwardTrackingSpeed(speed,bad,route,0);
   if(limited>speed || limited<fminf(speed,180.f))return 2;
   if(speed>=436 && limited>260)return 3;
   ++checks;
  }
 }
 PathPoint route[3]={{-690,200},{-690,400},{-690,600}};
 float bad=laterLapInwardTrackingSpeed(436,{-651.3f,318.2f,54.6f},route,0);
 float good=laterLapInwardTrackingSpeed(436,{-707.3f,339.7f,68.8f},route,0);
 if(!(bad<good && good>380))return 4;
 if(laterLapInwardTrackingSpeed(436,{NAN,0,0},route,0)!=0)return 5;
 std::cout<<"617 TRACKING "<<checks<<" PASS; recorded-leg illustrative bad/good="<<bad<<"/"<<good<<" mm/s\n";
}
''')


if __name__=='__main__':
    main()
