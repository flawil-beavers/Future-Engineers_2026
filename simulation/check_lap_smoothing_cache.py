"""Bitwise geometry comparison with the pre-cache historical smoother."""
import subprocess
from check_later_laps import later_lap_fixture
from check_connector_servo_resume import block
from check_returned_603_and_parking_practice import run

old = subprocess.check_output(
    ['git', 'show', 'c19b0b0:src/obstacle_path.cpp'], text=True)
legacy = block(old, old.index('void smoothKnownLaterLapRoute(')).replace(
    'void smoothKnownLaterLapRoute(', 'void legacySmoothKnownLaterLapRoute(', 1)
run('lap-smoothing-cache', later_lap_fixture().split('int main()')[0]+legacy+r'''
#include <cassert>
int main(){int n=0;
 const int layouts[11][3]={{0,0,0},{1,0,0},{2,0,0},{0,1,0},{0,2,0},
  {0,0,1},{0,0,2},{1,0,1},{1,0,2},{2,0,1},{2,0,2}};
 static PathPoint a[OBSTACLE_MAX_PATH_WAYPOINTS],b[OBSTACLE_MAX_PATH_WAYPOINTS];
 static PathPoint scratch[OBSTACLE_MAX_PATH_WAYPOINTS];
 for(int dir:{-1,1})for(int layout=0;layout<11;++layout){
  routeTurnSign=dir;resetState();baseline();
  for(auto&s:seats)s.confirmed=s.injected=false;
  for(int section=0;section<4;++section)for(int station=0;station<3;++station){
   int color=layouts[(layout+section)%11][station];
   int i=section*6+station*2+((section+station)%2);
   seats[i].confirmed=seats[i].injected=color!=0;seats[i].red=color==2;
  }
  learned();joinSameColourStraightEnds(livePath,true);roundKnownCornerPairs(livePath,true);
  memcpy(a,livePath,sizeof(PathPoint)*pathLength);memcpy(b,a,sizeof(PathPoint)*pathLength);
  float wall,pillar;laterLapRouteSafe(a,&wall,&pillar);
  smoothKnownLaterLapRoute(a,scratch,wall,pillar);
  legacySmoothKnownLaterLapRoute(b,scratch,wall,pillar);
  assert(memcmp(a,b,sizeof(PathPoint)*pathLength)==0);++n;
 }
 std::cout<<"SMOOTH CACHE "<<n<<" BITWISE IDENTICAL PASS\n";
}
''')
