"""CW log3's visible RED18: full colour horizon through next corner.
Archived numeric projections only; images and colour correctness are not inferred.
"""
from check_far_obstacle_acquisition import fixture_source
from check_returned_603_and_parking_practice import run
source=fixture_source()+r'''
int main(){int total=0;
 for(float shift:{0.f,50.f}){
  for(auto &s:seats){s={};s.x=s.y=9000;}
  seats[18].x=900;seats[19].x=1100;seats[18].y=seats[19].y=500;
  seats[18].headingDeg=seats[19].headingDeg=-90;
  seats[18].pathDistanceMm=seats[19].pathDistanceMm=4856.19f;
  baselinePath[0].distanceMm=3520.8f-shift;
  hostPose={-115,873,340.5f};clockMs+=1000;
  Blob b{};b.found=true;b.color=ColorType::RED;b.centerX=171;
  b.minX=163;b.maxX=179;b.minY=90;b.maxY=118;b.area=400;
  auto a=obstacle_path_observe(&b);clockMs+=76;auto c=obstacle_path_observe(&b);
  ++total;if(a.seatId!=18||!c.confirmed||c.seatId!=18){std::cout<<"FAIL "<<a.status<<" "<<a.seatId<<" "<<c.status<<" "<<c.seatId<<"\n";return 1;}
  seats[18].confirmed=seats[18].injected=false;seats[18].redVotes=0;clockMs+=1000;
  seats[18].pathDistanceMm=seats[19].pathDistanceMm=7000;
  if(obstacle_path_observe(&b).seatId>=0)return 2;
 }
 std::cout<<"CW LOG3 FAR CORNER "<<total<<" PASS; remote next-lap seat rejected\n";return 0;
}
'''
run('cw-log3-far-corner',source)
# Prove the archived gate actually caused rejection before the change.
old=source.replace('result.rangeMm <= CAMERA_DRIVE_CAL_MAX_RANGE_MM &&','result.rangeMm <= OBSTACLE_DISCOVERY_VIEW_MAX_MM &&')
run('cw-log3-far-corner-negative',old,1)
