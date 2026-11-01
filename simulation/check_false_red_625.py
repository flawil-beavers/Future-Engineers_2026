"""Replay rounded625 false-RED projection, with synthetic valid blob dimensions."""
from check_far_obstacle_acquisition import fixture_source
from check_returned_603_and_parking_practice import run


def main():
    code=fixture_source()+r'''
int main(){
 for(auto&s:seats){s={};s.x=s.y=9000;}
 seats[22].x=900;seats[22].y=-500;seats[23].x=1100;seats[23].y=-500;
 seats[22].headingDeg=seats[23].headingDeg=270;
 seats[22].pathDistanceMm=seats[23].pathDistanceMm=500;
 hostPose={944,117,281.5f};
 Blob red{};red.found=true;red.color=ColorType::RED;
 red.minX=270;red.maxX=290;red.centerX=280;red.minY=80;red.maxY=112;red.area=600;
 for(int i=0;i<3;++i){clockMs+=80;obstacle_path_observe(&red);}
 std::cout<<"625 FALSE RED confirmed="<<seats[22].confirmed<<"\n";
 return seats[22].confirmed?1:0;
}
'''
    run('625-false-red-new',code)
    # Negative control: before this gate the same rounded projection commits RED.
    old=code.replace('const bool distantRedMismatch=result.color==ColorType::RED &&',
                     'const bool distantRedMismatch=false && result.color==ColorType::RED &&')
    run('625-false-red-old',old,expected=1)


if __name__=='__main__':
    main()
