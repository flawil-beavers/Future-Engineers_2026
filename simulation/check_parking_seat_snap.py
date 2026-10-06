"""Host regression of the actual rule-aware seat snap function."""
import shutil
import subprocess
from pathlib import Path

from check_connector_servo_resume import block

ROOT = Path(__file__).resolve().parents[1]


def main():
    source = (ROOT / 'src/obstacle_path.cpp').read_text()
    function = block(source, source.index('int nearestSeatIndex('))
    fixture = r'''
#include <cmath>
#include <cstdint>
#include <iostream>
constexpr int OBSTACLE_SEAT_COUNT=24;
constexpr float OBSTACLE_SEAT_SNAP_RADIUS_MM=180;
constexpr int OBSTACLE_SECTION_LAYOUT_OFFICIAL=0, OBSTACLE_SECTION_LAYOUT_CHECK_ALL=1;
bool parkingSectionInnerSeatsOnly=true;
int sectionLayoutMode=OBSTACLE_SECTION_LAYOUT_OFFICIAL;
struct Seat {float x,y;}; Seat seats[24];
float distanceSquared(float x,float y,float a,float b){return (x-a)*(x-a)+(y-b)*(y-b);}
FUNCTION
int main(){
 for(int i=0;i<24;++i) seats[i]={10000.f+i*500.f,10000.f};
 for(int station=0;station<3;++station){
  seats[2*station]={500.f-station*500.f,-900.f};
  seats[2*station+1]={500.f-station*500.f,-1100.f};
 }
 for(int station=0;station<3;++station){
  auto a=seats[2*station],b=seats[2*station+1];
  float error=0;
  if(nearestSeatIndex(a.x,a.y,&error)!=2*station || error!=0) return 1;
  if(nearestSeatIndex(b.x,b.y)!=-1) return 2;
 }
 // Rule filter does not affect another section, CHECK_ALL, or nonparking tests.
 if(nearestSeatIndex(seats[6].x,seats[6].y)!=6) return 3;
 sectionLayoutMode=OBSTACLE_SECTION_LAYOUT_CHECK_ALL;
 if(nearestSeatIndex(500,-1100)!=1) return 4;
 sectionLayoutMode=OBSTACLE_SECTION_LAYOUT_OFFICIAL;
 parkingSectionInnerSeatsOnly=false;
 if(nearestSeatIndex(500,-1100)!=1) return 5;
 std::cout << "PASS: legal parking seats retained; impossible outer projections rejected without forced remap\n";
}
'''.replace('FUNCTION', function)
    destination = ROOT / 'local_workspace/parking-seat-snap'
    destination.mkdir(parents=True, exist_ok=True)
    cpp = destination / 'check.cpp'
    cpp.write_text(fixture)
    compiler = shutil.which('g++') or shutil.which('clang++')
    if not compiler:
        raise RuntimeError('A host C++ compiler on PATH is required')
    executable = destination / 'check.exe'
    subprocess.run([compiler, '-std=c++17', str(cpp), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    main()
