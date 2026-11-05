"""Actual production speed selection; confined strokes retain their speed."""
from check_cw_start_planner import ROOT
from check_returned_603_and_parking_practice import run

source = (ROOT/'src/obstacle.cpp').read_text()
at = source.index('const ParkingExitSegment &segment =')
at = source.index('    set_speed(', at)
command = source[at:source.index(';', at)+1]
config = (ROOT/'include/config.h').read_text()
constants = '\n'.join(line for line in config.splitlines() if line.startswith((
    'constexpr auto OBSTACLE_PARKING_EXIT_SPEED =',
    'constexpr auto OBSTACLE_PARKING_EXIT_LONG_ARC_SPEED =')))
run('exit-long-arc-speed', r'''
#include <cassert>
#include <iostream>
'''+constants+r'''
int commanded=0,oc_parking_exit_segment=0;
struct Segment {int direction;} segment;
void set_speed(int value){commanded=value;}
void update(){
'''+command+r'''
}
int main(){int n=0;
 for(int dir:{-1,1})for(int i=0;i<5;++i){
  oc_parking_exit_segment=i;segment.direction=dir;update();
  assert(commanded==dir*(i<3?80:100));++n;
 }
 std::cout<<"EXIT SPEED "<<n<<" PASS; first3 unchanged, long arcs100\n";
}
''')
