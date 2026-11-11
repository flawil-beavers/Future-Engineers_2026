"""Production mapped GREEN acquisition range/votes, with controlled vision output."""
from check_far_obstacle_acquisition import fixture_source
from check_connector_servo_resume import ROOT, block
from check_returned_603_and_parking_practice import run


def main():
    path=(ROOT/'src/obstacle_path.cpp').read_text()
    code=fixture_source()+r'''
#include <cstring>
unsigned micros(){return 0;}unsigned lastGreenSeatProcessingUs=0;
bool greenSeatCandidateThisFrame[24]={};unsigned greenSeatCandidateLastSeenMs[24]={};
ObstacleObservationResult lastDiscoveryObservation;
struct Camera{unsigned char pixel=0;unsigned char*getBuffer(){return &pixel;}int getWidth(){return 320;}int getHeight(){return 240;}}camera;
bool candidateEnabled=true;int requested=0;
struct FakeVision{
 void measurePillarHeight(unsigned char*,int,int,Blob&){}
 bool findGreenSeatCandidate(unsigned char*,int,int,int x,int foot,GreenSeatCandidate&c){
  ++requested;if(!candidateEnabled||abs(x-164)>3)return false;
  c.silhouetteFound=true;c.greenSamples=30;c.brightnessContrast=80;
  c.blob.found=true;c.blob.color=ColorType::GREEN;c.blob.centerX=164;
  c.blob.minX=154;c.blob.maxX=174;c.blob.minY=80;c.blob.maxY=foot;return true;
 }
}vision;
void logParkingGreenCheck(uint8_t,const PositionEstimate&,const char*,const GreenSeatCandidate* = nullptr){}
'''
    for signature in ('float wrap180(', 'void seatCameraGeometry(', 'bool seatComfortablyVisible('):
        code+=block(path,path.rindex(signature))+'\n'
    code+=block(path,path.rindex('void processGreenSeatCandidates('))+r'''
void reset(float range,int seat){
 for(auto&s:seats){s={};s.x=9000;s.y=9000;}
 seats[seat].x=800;seats[seat].y=0;seats[seat^1].x=800;seats[seat^1].y=200;
 hostPose={800-range-OBSTACLE_CAMERA_LOCAL_X_MM,0,0};clockMs+=1000;requested=0;
}
void frames(int n,ObstacleObservationResult o={}){
 for(int i=0;i<n;++i){clockMs+=80;processGreenSeatCandidates(hostPose,o);}
}
int main(){int cases=0;
 for(float range:{650.f,700.f,800.f}){
  reset(range,6);if(seatComfortablyVisible(6,hostPose))return 1;
  frames(2);if(seats[6].confirmed)return 2;
  frames(1);if(!seats[6].confirmed||seats[6].red)return 3;++cases;
 }
 reset(810,6);frames(5);if(requested||seats[6].confirmed)return 4;++cases;
 reset(700,0);frames(5);if(requested||seats[0].confirmed)return 5;++cases;
 reset(700,6);candidateEnabled=false;frames(5);if(seats[6].confirmed)return 6;++cases;
 candidateEnabled=true;reset(700,6);ObstacleObservationResult red{};
 red.productionValid=true;red.color=ColorType::RED;red.seatId=6;
 frames(5,red);if(seats[6].confirmed)return 7;++cases;
 std::cout<<"FAR GREEN "<<cases<<" PASS; 3-frame650..800, no CLEAR widening, parking/red/empty limits\n";
}
'''
    run('624-627-far-green',code)


if __name__=='__main__':
    main()
