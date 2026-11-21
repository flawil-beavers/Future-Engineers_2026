"""Actual RGB565 vision, ground-plane height and seat voting for the extra task.

Images are independent pinhole projections with a square50mm base. This is a
software/geometry regression, not physical acceptance of the camera mounting.
"""
import shutil
import subprocess
import hashlib
import json
from check_far_obstacle_acquisition import ROOT, fixture_source


def main():
    source=fixture_source(surprise=True)
    source+='\n#include <algorithm>\n#include <cstring>\nusing std::max;using std::min;\nunsigned long micros(){return clockMs*1000;}\n'
    source+=(ROOT/'src/vision.cpp').read_text()
    source+=r'''
#include <cassert>
#include <vector>
#include <algorithm>
#include <fstream>
static Vision visionTest;
static std::vector<uint8_t> pixels(320*240*2);
void pixel(int x,int y,uint16_t color){
 if(x<0||x>=320||y<0||y>=240)return;
 int offset=((239-y)*320+(319-x))*2;
 pixels[offset]=color>>8;pixels[offset+1]=color&255;
}
float vertical(float distance,float z,float cameraHeight=85.5f,float fy=259,float pitch=5.62f){
 const float angle=pitch*PI/180,dy=cameraHeight-z;
 return 119.5f+fy*(dy*cosf(angle)-distance*sinf(angle))/
                   (distance*cosf(angle)+dy*sinf(angle));
}
Blob image(ColorType color,float mm,float distance,int center=160,float yaw=0,float cameraHeight=85.5f){
 std::fill(pixels.begin(),pixels.end(),0);
 const int foot=int(std::round(vertical(distance,0,cameraHeight)));
 const int top=int(std::round(vertical(distance,mm,cameraHeight)));
 const float base=50*(cosf(yaw*PI/180)+sinf(yaw*PI/180));
 const int width=int(std::round(249*base/distance));
 const uint16_t rgb=color==ColorType::RED ? 0xf800:0x1a23; // measured dark-green HSV band
 for(int y=std::max(0,top);y<=std::min(239,foot);++y)
  for(int x=center-width/2;x<center+(width+1)/2;++x)pixel(x,y,rgb);
 clockMs+=40;assert(visionTest.update(pixels.data(),320,240));
 const auto &r=visionTest.getResult();
 return color==ColorType::RED ? r.redPillar:r.greenPillar;
}
void resetSeats(){
 surpriseSeatIndex=-1;completedLaps=0;injectionCount=0;clockMs+=2000;
 for(int i=0;i<OBSTACLE_SEAT_COUNT;++i){seats[i]={};seats[i].x=seats[i].y=9000;
  seats[i].pathDistanceMm=float((i/2+1)*500);}
 for(auto&s:discoveryStations)s.observedClear=false;
 for(auto &v:oppositeSeatConflictReported)v=false;
}
bool vote(uint8_t id,Blob &blob){
 prepareConsecutiveVote(id,blob.color);return recordObservedSeatVote(id,blob);
}
int main(int argc,char **argv){visionTest.begin();int rendered=0,policy=0;
 if(argc>4 && std::string(argv[1])=="--sequence"){
  resetSeats();seats[0].confirmed=seats[0].injected=true;
  const auto color=ColorType(std::stoi(argv[2]));const int height=std::stoi(argv[3]);
  Blob b;const int needed=height==100 ? 2:3;
  for(int i=4;i<argc;++i){
   std::ifstream f(argv[i],std::ios::binary);f.read(reinterpret_cast<char*>(pixels.data()),pixels.size());
   assert(f.gcount()==int(pixels.size()));clockMs+=40;
   assert(visionTest.update(pixels.data(),320,240));
   b=color==ColorType::RED ? visionTest.getResult().redPillar:visionTest.getResult().greenPillar;
   assert(b.found && int(obstacle_sign_height::classify(b))==height);
   vote(6,b);assert(seats[6].confirmed==(i-3>=needed));
  }
  if(argc-4>=needed){
   const bool originalRed=(color==ColorType::RED)!=(height!=100);
   assert(seats[6].red==originalRed && seats[6].visibleRed==(color==ColorType::RED));
   assert(seats[6].modifiedSign==(height!=100));
   for(int lap:{1,2}){
    completedLaps=lap;b.color=color==ColorType::RED ? ColorType::GREEN:ColorType::RED;
    b.frameSequence+=100;obstacle_path_observe(&b);
    assert(seats[6].red==originalRed && seats[6].visibleRed==(color==ColorType::RED));
    assert(seats[6].modifiedSign==(height!=100));
   }
  }
  std::cout<<"APPROVED SEQUENCE frames="<<argc-4<<" height="<<height<<" confirmed="<<seats[6].confirmed<<" PASS\n";
  return 0;
 }
 if(argc>1){
  std::ifstream f(argv[1],std::ios::binary);f.read(reinterpret_cast<char*>(pixels.data()),pixels.size());
  if(f.gcount()!=int(pixels.size()))return 2;
  visionTest.update(pixels.data(),320,240);
  for(auto b:{visionTest.getResult().redPillar,visionTest.getResult().greenPillar})
   std::cout<<"REPLAY color="<<int(b.color)<<" found="<<b.found<<" box="<<b.minX<<"/"<<b.minY<<"/"<<b.maxX<<"/"<<b.maxY
    <<" top="<<b.heightTopY<<" mm="<<obstacle_sign_height::estimate(b)<<" class="<<int(obstacle_sign_height::classify(b))
    <<" range="<<obstacle_estimate_camera_forward_mm(&b)<<" x="<<b.centerX<<"\n";
  return 0;
 }
 for(auto color:{ColorType::RED,ColorType::GREEN})for(float mm:{50.f,100.f,150.f})
 for(float distance:{300.f,400.f,500.f,600.f})for(int center:{80,160,240})
 for(float yaw:{0.f,30.f,45.f})for(float cameraHeight:{81.f,85.5f,90.f}){
  auto b=image(color,mm,distance,center,yaw,cameraHeight);
  auto h=obstacle_sign_height::classify(b);
  if(!b.found||!obstacle_blob_shape_valid(&b)||int(h)!=int(mm)){
   std::cout<<"IMAGE FAIL color/mm/dist/x/yaw/cam="<<int(color)<<"/"<<mm<<"/"<<distance<<"/"<<center<<"/"<<yaw<<"/"<<cameraHeight
    <<" box="<<b.minX<<"/"<<b.minY<<"/"<<b.maxX<<"/"<<b.maxY<<" top/valid/height="<<b.heightTopY<<"/"<<b.heightTopMeasured<<"/"<<obstacle_sign_height::estimate(b)<<" class="<<int(h)<<"\n";
   return 1;
  }++rendered;
 }
 // Full production association/voting/dispatch, from rendered pixels.
 int associated=0;
 for(int dir:{-1,1})for(auto color:{ColorType::RED,ColorType::GREEN})
 for(float height:{50.f,100.f,150.f})for(float range:{350.f,400.f,500.f,600.f}){
  resetSeats();routeTurnSign=dir;seats[0].confirmed=seats[0].injected=true;
  seats[0].pathDistanceMm=200;
  seats[6].x=seats[7].x=800;seats[6].y=0;seats[7].y=dir*200;
  seats[6].headingDeg=seats[7].headingDeg=dir>0 ? 0:180;
  seats[6].pathDistanceMm=seats[7].pathDistanceMm=775;
  hostPose={800-dir*(range+OBSTACLE_CAMERA_LOCAL_X_MM),0,dir>0?0.f:180.f};
  ObstacleObservationResult r;
  for(int n=0;n<(height==100?2:3);++n){auto blob=image(color,height,range);r=obstacle_path_observe(&blob);}
  const bool expectedRed=(color==ColorType::RED)!=(height!=100);
  if(!seats[6].confirmed||seats[6].red!=expectedRed||r.passSide!=(expectedRed?'R':'L')){
   std::cout<<"ASSOCIATION FAIL dir/color/h/range="<<dir<<"/"<<int(color)<<"/"<<height<<"/"<<range
    <<" status/seat="<<int(r.status)<<"/"<<int(r.seatId)<<"\n";return 1;
  }++associated;
 }
 for(int direction:{-1,1})for(auto visible:{ColorType::RED,ColorType::GREEN})for(float mm:{50.f,150.f}){
  resetSeats();routeTurnSign=direction;
  // A real earlier normal sign establishes encounter order, not observation order.
  seats[0].confirmed=seats[0].injected=true;seats[0].red=true;
  Blob b;
  for(int n=0;n<3;++n){b=image(visible,mm,400);bool done=vote(6,b);assert(done==(n==2));}
  assert(seats[6].confirmed&&seats[6].modifiedSign&&seats[6].heightMm==mm);
  assert(seats[6].visibleRed==(visible==ColorType::RED));
  assert(seats[6].red!=(visible==ColorType::RED)&&surpriseSeatIndex==6);
  for(int n=0;n<5;++n){b=image(visible,mm,400);assert(!vote(6,b));}
  assert(seats[6].red!=(visible==ColorType::RED));
  // Exactly one reversal; another unusual-looking object cannot consume a second slot.
  for(int n=0;n<2;++n){b=image(visible,mm,400);vote(10,b);}
  assert(seats[10].confirmed&&!seats[10].modifiedSign&&surpriseSeatIndex==6);
  assert(seats[10].red==(visible==ColorType::RED));
  completedLaps=1;b=image(visible,mm,400);auto r=obstacle_path_observe(&b);
  assert(!r.confirmed&&surpriseSeatIndex==6&&seats[6].modifiedSign);++policy;
 }
 // Far-first observation does not consume the one modification or bypass the first guard.
 resetSeats();auto b=image(ColorType::GREEN,150,400);
 for(int n=0;n<5;++n){b=image(ColorType::GREEN,150,400);vote(6,b);}
 assert(!seats[6].confirmed&&surpriseSeatIndex<0);
 // All earlier positions measured empty: this is the first encountered sign.
 for(int station=0;station<3;++station)discoveryStations[station].observedClear=true;
 b=image(ColorType::GREEN,150,400);vote(6,b);
 assert(seats[6].confirmed&&!seats[6].modifiedSign&&!seats[6].red&&surpriseSeatIndex<0);++policy;
 // A second sign in the starting straight remains eligible.
 resetSeats();seats[0].confirmed=seats[0].injected=true;
 for(int n=0;n<3;++n){b=image(ColorType::RED,50,400);vote(2,b);}
 assert(seats[2].modifiedSign&&!seats[2].red&&surpriseSeatIndex==2);++policy;
 // A behind-start cached seat is not proof of an earlier encounter.
 resetSeats();seats[0].pathDistanceMm=seats[1].pathDistanceMm=0;
 seats[0].confirmed=true;assert(surpriseOrderForSeat(2)==SURPRISE_FIRST);
 seats[0].injected=true;assert(surpriseOrderForSeat(2)==SURPRISE_LATER);++policy;
 // Duplicate calls for one camera frame cannot provide three height votes.
 resetSeats();seats[0].confirmed=true;b=image(ColorType::GREEN,50,400);
 for(int n=0;n<8;++n)vote(6,b);
 assert(!seats[6].confirmed&&seats[6].heightFrames==1);++policy;
 // Unknown/occluded height cannot be guessed as a normal sign or clear position.
 b.heightTopMeasured=false;b.frameSequence+=100;vote(6,b);
 assert(!seats[6].confirmed&&surpriseSeatIndex<0);++policy;
 // An unusable primary top may be refined by the mapped fallback from the
 // same camera frame. That upgrade contributes exactly one vote, not zero
 // forever and not two votes from one exposure.
 for(float mm:{50.f,100.f,150.f}){
  resetSeats();seats[0].confirmed=true;
  const int needed=mm==100 ? 2:3;
  for(int n=0;n<needed;++n){
   b=image(ColorType::GREEN,mm,400);auto unknown=b;unknown.heightTopMeasured=false;
   assert(!vote(6,unknown));
   assert(vote(6,b)==(n==needed-1));
   assert(!vote(6,b));
  }
  assert(seats[6].confirmed&&seats[6].modifiedSign==(mm!=100));++policy;
 }
 // Expiry, changing height and changing colour must restart the consensus.
 resetSeats();routeTurnSign=1;seats[0].confirmed=true;
 for(int n=0;n<4;++n){
  b=image(ColorType::GREEN,50,400,54);
  assert(b.found&&obstacle_sign_height::classify(b)==obstacle_sign_height::SHORT);
  assert(!vote(9,b));
 }
 assert(!seats[9].confirmed&&surpriseSeatIndex<0);
 for(int n=0;n<2;++n){b=image(ColorType::GREEN,100,400);assert(vote(9,b)==(n==1));}
 assert(!seats[9].red&&surpriseSeatIndex<0);
 for(int n=0;n<3;++n){b=image(ColorType::GREEN,150,500);assert(vote(19,b)==(n==2));}
 assert(seats[19].modifiedSign&&seats[19].red&&surpriseSeatIndex==19);++policy;

 resetSeats();routeTurnSign=1;seats[0].confirmed=true;
 for(int n=0;n<3;++n){b=image(ColorType::GREEN,50,400);assert(vote(9,b)==(n==2));}
 assert(seats[9].modifiedSign&&surpriseSeatIndex==9);++policy;

 // A short/partial RED on the stationary CW middle view must not
 // occupy that station or consume the surprise identity as in log 46.
 resetSeats();routeTurnSign=-1;parkingEntryObserving=true;
 discoveryStations[0].observedClear=true;
 assert(surpriseOrderForSeat(2)==SURPRISE_FIRST);
 for(int n=0;n<4;++n){
  b=image(ColorType::RED,50,300);
  assert(obstacle_sign_height::classify(b)==obstacle_sign_height::SHORT);
  assert(!vote(2,b));
 }
 assert(!seats[2].confirmed&&surpriseSeatIndex<0);
 for(int n=0;n<3;++n){b=image(ColorType::RED,100,300);assert(vote(2,b)==(n==2));}
 assert(seats[2].red&&!seats[2].modifiedSign&&surpriseSeatIndex<0);
 parkingEntryObserving=false;++policy;

 resetSeats();seats[0].confirmed=true;
 b=image(ColorType::GREEN,50,400);vote(6,b);
 clockMs+=OBSTACLE_SEAT_VOTE_WINDOW_MS+1;
 for(int n=0;n<3;++n){b=image(ColorType::GREEN,50,400);assert(vote(6,b)==(n==2));}
 assert(seats[6].modifiedSign);++policy;
 resetSeats();seats[0].confirmed=true;
 b=image(ColorType::GREEN,50,400);vote(6,b);
 b=image(ColorType::GREEN,150,400);vote(6,b);
 assert(!seats[6].confirmed&&seats[6].heightFrames==1);
 b=image(ColorType::RED,150,400);vote(6,b);
 assert(!seats[6].confirmed&&seats[6].heightFrames==1);
 for(int n=0;n<2;++n){b=image(ColorType::RED,150,400);vote(6,b);}
 assert(seats[6].modifiedSign&&!seats[6].red);++policy;
 // The protected first sign retains ordinary two-frame colour recognition
 // even if its height is occluded. No surprise identity is consumed.
 resetSeats();
 for(int n=0;n<2;++n){b=image(ColorType::GREEN,100,400);b.heightTopMeasured=false;
  assert(vote(0,b)==(n==1));}
 assert(!seats[0].modifiedSign&&!seats[0].red&&surpriseSeatIndex<0);++policy;
 // A thin connected same-colour horizontal background strip may widen the
 // lower component. Its independent narrow stem still has a normal top.
 b=image(ColorType::GREEN,100,400);
 for(int y=84;y<=88;++y)for(int x=132;x<=188;++x)pixel(x,y,0x1a23);
 visionTest.update(pixels.data(),320,240);b=visionTest.getResult().greenPillar;
 assert(b.found&&obstacle_sign_height::classify(b)==obstacle_sign_height::NORMAL);++policy;
 // Wide upper room regions cannot turn the lower ROI pillar into a tall sign.
 b=image(ColorType::GREEN,100,400);
 for(int y=0;y<80;++y)for(int x=0;x<320;++x)pixel(x,y,0x1a23);
 visionTest.update(pixels.data(),320,240);
 b=visionTest.getResult().greenPillar;
 assert(b.found&&!b.heightTopMeasured&&obstacle_sign_height::classify(b)==obstacle_sign_height::UNKNOWN);++policy;
 // Foot/range do not vary with height; clipped feet/edges never classify heights.
 float depth=0;
 for(float mm:{50.f,100.f,150.f}){b=image(ColorType::RED,mm,400);
  if(depth)assert(fabsf(depth-obstacle_estimate_camera_forward_mm(&b))<.001f);
  depth=obstacle_estimate_camera_forward_mm(&b);
  auto clipped=b;clipped.maxY=239;assert(obstacle_sign_height::classify(clipped)==obstacle_sign_height::UNKNOWN);
  clipped=b;clipped.minX=0;assert(obstacle_sign_height::classify(clipped)==obstacle_sign_height::UNKNOWN);++policy;
 }
 std::cout<<"SURPRISE RENDERED "<<rendered<<" ASSOCIATED "<<associated<<" POLICY "<<policy<<" PASS\n";
}
'''
    dest=ROOT/'local_workspace/surprise-rule';dest.mkdir(parents=True,exist_ok=True)
    cpp,exe=dest/'check.cpp',dest/'check.exe';cpp.write_text(source)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'simulation/host'),
                    '-I',str(ROOT/'include'),str(cpp),'-o',str(exe)],check=True)
    run=subprocess.run([str(exe)],capture_output=True,text=True)
    (dest/'output.txt').write_text(run.stdout+run.stderr);print(run.stdout,end='')
    if not run.returncode:
        replay=[]; archived=0; absent=0; failures=[]; measured=[]; sequences={}
        evidence=ROOT/'simulation/evidence/camera_diagnostics'
        registry=json.loads((evidence/'validation_status.json').read_text())
        for stem,entry in registry['captures'].items():
            if entry['status']!='approved' or not entry.get('ground_truth'):
                continue
            for name,digest in entry['sha256'].items():
                assert hashlib.sha256((evidence/name).read_bytes()).hexdigest()==digest,name
            file=evidence/(stem+'.rgb565')
            truth=entry['ground_truth']
            key=(truth['color'],truth['height_mm'],truth['lens_to_near_face_mm'])
            sequences.setdefault(key,[]).append(file)
            result=subprocess.run([str(exe),str(file)],capture_output=True,text=True)
            assert result.returncode==0,(file.name,result.stderr)
            replay.append(file.name+'\n'+result.stdout)
            truth=entry.get('ground_truth')
            target_found=False
            for line in result.stdout.splitlines():
                if 'found=1' in line:
                    fields=dict(token.split('=',1) for token in line.split()[1:])
                    if truth:
                        expected_color={'RED':1,'GREEN':2}[truth['color']]
                        if int(fields['color'])!=expected_color:
                            failures.append((file.name,'unexpected colour component',line))
                            continue
                        target_found=True
                        measured.append(dict(file=file.name,ground_truth=truth,observed=fields))
                        if int(fields['class'])!=truth['height_mm']:
                            failures.append((file.name,'height mismatch',line))
                        if abs(float(fields['range'])-truth['lens_to_near_face_mm'])>30:
                            failures.append((file.name,'range error exceeds30mm',line))
                        if not 140<=int(fields['x'])<=188:
                            failures.append((file.name,'centred target replaced by background',line))
                    elif int(fields['class'])!=100:
                        failures.append((file.name,'historical normal reference mismatch',line))
                    archived+=1
            if truth and not target_found:
                failures.append((file.name,'target not detected',result.stdout))
            absent+=int('found=1' not in result.stdout)
        (dest/'archive_replay.txt').write_text(''.join(replay))
        (dest/'measured_replay.json').write_text(json.dumps(dict(measured=measured,failures=failures),indent=2)+'\n')
        assert archived>0,'Approved normal reference must actually be detected'
        print('USER-APPROVED DETECTIONS',archived,'UNDETECTED FRAMES',absent,
              'GROUND-TRUTH FAILURES',len(failures),'; unapproved/rejected images excluded')
        if failures:
            for failure in failures: print('REFERENCE FAIL',*failure)
            raise SystemExit(1)
        for (color,height,distance),files in sorted(sequences.items()):
            result=subprocess.run([str(exe),'--sequence',str({'RED':1,'GREEN':2}[color]),
                                   str(height),*map(str,files)],capture_output=True,text=True)
            assert result.returncode==0,(color,height,distance,result.stdout,result.stderr)
            print(color,distance,result.stdout.strip())
    raise SystemExit(run.returncode)


if __name__=='__main__':main()
