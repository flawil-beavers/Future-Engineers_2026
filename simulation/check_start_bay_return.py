"""First-lap return to fixed pink bay; actual source functions, ideal bicycle.
Known maps, no ToF feedback/servo lag. Includes all official start colour
layouts and both sides/colours of the preceding section's last station.
"""
import itertools,subprocess,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    fixture=(ROOT/'local_workspace/later-laps/check.cpp').read_text().split('int main()')[0]
    fixture+=r'''
int main(){int total=0,bad=0;
 for(int dir:{-1,1})for(int layout=0;layout<11;++layout)for(int side=0;side<2;++side)for(int color=1;color<=2;++color){
  routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  const int configs[11][3]={{0,0,0},{1,0,0},{2,0,0},{0,1,0},{0,2,0},{0,0,1},{0,0,2},{1,0,1},{1,0,2},{2,0,1},{2,0,2}};
  for(int station=0;station<3;++station){int i=station*2+(dir>0);int c=configs[layout][station];seats[i].confirmed=seats[i].injected=c;seats[i].red=c==2;}
  seats[22+side].confirmed=seats[22+side].injected=true;seats[22+side].red=color==2;
  learned();roundKnownCornerPairs(livePath,true);recomputeSpeedProfile(livePath);
  progressIndex=0;for(int i=0;i<pathLength;++i)if(baselinePath[i].distanceMm<=corners[3].pathStartMm-100)progressIndex=i;
  float x=livePath[progressIndex].x,y=livePath[progressIndex].y,h=connectorRouteHeading(livePath,progressIndex)*PI/180;
  bool ok=false;
  for(int step=0;step<1400;++step){
   if(!parking_start_footprint::safe(x,y,h*180/PI,-1,false,5)||!parking_start_footprint::safe(x,y,h*180/PI,1,false,5))break;
   bool collision=false;for(int seat=0;seat<24;++seat){if(!seats[seat].confirmed)continue;ObstacleClearanceSample c{};calculateClearanceAtPose(seats[seat],x,y,h*180/PI,c);if(c.wallMm<=0||c.pillarMm<=0)collision=true;}if(collision)break;
   if(dir<0?x<50&&y<-800:x>550&&y<-500){ok=true;break;}
   progressIndex=nearestPathIndex(livePath,x,y,progressIndex,OBSTACLE_PATH_PROGRESS_WINDOW);
   PositionEstimate p;p.x_mm=x;p.y_mm=y;p.heading_deg=h*180/PI;
   float look=adaptiveLookahead(livePath[progressIndex].speedMmS);if(nearCorner(baselinePath[progressIndex].distanceMm))look*=OBSTACLE_LOOKAHEAD_CORNER_SCALE;
   auto t=findLookahead(livePath,p,look);float dx=t.x-x,dy=t.y-y;
   float k=2*(-dx*sinf(h)+dy*cosf(h))/fmaxf(1,dx*dx+dy*dy);
   int servo=static_cast<int>(clampFloat(-atanf(100*k)*180/PI,-42,42));if(!parkingReturnMotionSafe(p,servo))break;k=-tanf(servo*PI/180)/100;
   float next=h+k*2;if(fabsf(k)>1e-6){x+=(sinf(next)-sinf(h))/k;y+=(cosf(h)-cosf(next))/k;}else{x+=2*cosf(h);y+=2*sinf(h);}h=next;
  }
  ++total;if(!ok){++bad;std::cout<<"FAIL "<<dir<<" "<<layout<<" "<<side<<" "<<color<<" pose "<<x<<" "<<y<<" "<<h*180/PI<<"\n";}
 }
 std::cout<<"BAY RETURN "<<total<<" FAILED "<<bad<<"\n";return bad?1:0;
}
'''
    p=ROOT/'local_workspace/bay-return';p.mkdir(exist_ok=True);(p/'check.cpp').write_text(fixture)
    # Ackermann's header consumes Arduino.h from the standard generated fixture.
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),'-I',str(ROOT/'local_workspace/later-laps'),str(p/'check.cpp'),'-o',str(p/'check.exe')],check=True)
    r=subprocess.run([str(p/'check.exe')],capture_output=True,text=True);(p/'output.txt').write_text(r.stdout);print(r.stdout);raise SystemExit(r.returncode)
if __name__=='__main__':main()
