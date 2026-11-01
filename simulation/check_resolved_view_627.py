"""627: a known colour with an unsafe join must retry checked viewpoints."""
from check_connector_servo_resume import ROOT, block
from check_returned_603_and_parking_practice import run


def main():
    source=(ROOT/'src/obstacle_path.cpp').read_text()
    start=source.index('        if (cornerViewCanResume)',source.index('void updateCornerView(bool'))
    branch=block(source,start)
    run('627-resolved-view',r'''
#include <cstdint>
#include <iostream>
struct PositionEstimate{}pose;
enum{OBSERVE,EXTRA,CORNER_VIEW_REVERSE};
int cornerViewPhase=OBSERVE;float cornerViewDistanceMm=210,cornerViewMeasuredReverseMm=211;
bool cornerViewCanResume=true,extraSafe=false,extraCalled=false;
unsigned cornerViewPhaseMs=0,cornerViewLastNoSafeTraceMs=0;
struct Log{void println(const char*){}}Serial;
unsigned millis(){return 1000;}
bool resumeDiscoveryFromView(PositionEstimate){return false;}
bool cornerViewSweepSafe(PositionEstimate,float){return false;}
bool beginCornerViewExtra(PositionEstimate){extraCalled=true;if(extraSafe)cornerViewPhase=EXTRA;return extraSafe;}
void update(){
'''+branch+r'''
}
int main(){
 extraSafe=true;update();if(!extraCalled||cornerViewPhase!=EXTRA)return 1;
 extraSafe=false;extraCalled=false;cornerViewPhase=OBSERVE;update();
 if(!extraCalled||cornerViewPhase!=OBSERVE)return 2;
 std::cout<<"627 RESOLVED VIEW PASS: checked parallax or continued observation, no unsafe join\n";
}
''')


if __name__=='__main__':
    main()
