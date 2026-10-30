"""Compare direct staging with the former learned bypass at logged arrivals.

Arrival poses/maps are immutable evidence. Subsequent ToF and motion are an
ideal synthetic plant; travel savings are model results, not measured times.
"""
import re
import shutil
import subprocess
from check_final_parking_sequence import ROOT, fixture_source


def main():
    inputs=[]
    evidence=ROOT/'simulation/evidence/parking_exit_diagnostics'
    for number in (567,577,579,582,590,593):
        paths=list(evidence.glob(f'*_log_{number}_*.txt'))
        assert len(paths)==1
        log=paths[0].read_text()
        match=re.search(r'\[FINAL_PARK_TRACE\].*?state/phase=1/0 xyh=([-\d.]+)/([-\d.]+)/([-\d.]+)',log)
        assert match
        colors=[0]*24
        for seat,color in re.findall(r'\[PATH\] Live avoidance injected seat=(\d+) color=(RED|GREEN)',log):
            colors[int(seat)]=2 if color=='RED' else 1
        direction=1 if '_ccw' in paths[0].stem else -1
        inputs.append(' '.join(map(str,[number,direction,*match.groups(),*colors])))
    fixture=fixture_source().replace('namespace parking_module {','bool legacyApproach=false;\nnamespace parking_module {',1)
    assert fixture.count('for (bool direct:{true,false})')==1
    fixture=fixture.replace('for (bool direct:{true,false})','for (bool direct:{!legacyApproach,false})')
    fixture+=r'''
struct ArrivalResult{bool ready=false,collision=false;float travel=0,peakX=-1e9;};
ArrivalResult run(int direction,float x,float y,float h,bool legacy){
 using namespace parking_module;
 final_parking_reset();legacyApproach=legacy;
 hostPose=truePose=PositionEstimate{x,y,h};encoder=0;hostTime=0;hostGyroHealthy=true;framePeriod=30;
 stop(true);robot_logger.lines.str("");robot_logger.lines.clear();
 ArrivalResult r;
 for(int tick=0;tick<5000&&!final_parking_aborted()&&state!=FP_SCAN_SETTLE;++tick){
  final_parking_update(direction);r.peakX=fmaxf(r.peakX,hostPose.x_mm);
  if(dc_state==DC_ENABLED){float left=fabsf(command*.02f),sign=command<0?-1.f:1.f;
   while(left>.0001f){float ds=sign*fminf(1,left);left-=fabsf(ds);encoder+=ds;r.travel+=fabsf(ds);
    advanceParkingPose(hostPose,steeringCommand,ds);advanceParkingPose(truePose,steeringCommand,ds);
    r.collision|=!parkingEntryPoseSafe(truePose,steeringCommand);
   }
  }hostTime+=20;
 }r.ready=state==FP_SCAN_SETTLE;return r;
}
int main(){int number,direction,colors[24],count=0,bad=0;float x,y,h;
 while(std::cin>>number>>direction>>x>>y>>h){
  for(int &c:colors)std::cin>>c;
  ::routeTurnSign=direction;resetState();baseline();
  for(int i=0;i<24;++i){seats[i].confirmed=seats[i].injected=colors[i]!=0;seats[i].red=colors[i]==2;}
  learned();joinSameColourStraightEnds(livePath,true);roundKnownCornerPairs(livePath,true);
  if(!buildOptimizedPath())return 1;
  auto old=run(direction,x,y,h,true),now=run(direction,x,y,h,false);
  bool ok=now.ready&&!now.collision&&(!old.ready||now.travel<=old.travel+1);
  ++count;bad+=!ok;
  std::cout<<"ARRIVAL "<<number<<" ready="<<now.ready<<" collision="<<now.collision
    <<" old/new/saved_mm="<<old.travel<<"/"<<now.travel<<"/"<<old.travel-now.travel
    <<" peak_x="<<now.peakX<<"\n";
 }
 std::cout<<"LOGGED ARRIVALS "<<count<<" FAILED "<<bad<<"\n";
 int references=0;
 for(int dir:{-1,1})for(float residual:{-40.f,40.f,107.9f,129.f}){
  ::routeTurnSign=dir;resetState();baseline();for(auto &s:seats)s.confirmed=s.injected=false;
  using namespace parking_module;final_parking_reset();parking_module::routeTurnSign=dir;
  outerSensor=dir>0?TOF_RIGHT:TOF_LEFT;hostPose={1055,-1206.5f,dir>0?0.f:180.f};truePose=hostPose;truePose.y_mm+=residual;
  hostTime=1000;encoder=0;hostGyroHealthy=true;framePeriod=30;stop(false);
  state=FP_APPROACH_BRAKE;stateStartMs=0;legacyApproach=false;
  bool collision=false;
  for(int tick=0;tick<3000&&!final_parking_aborted()&&state!=FP_SCAN_SETTLE;++tick){
   final_parking_update(dir);
   if(dc_state==DC_ENABLED){float ds=command*.02f;encoder+=ds;
    advanceParkingPose(hostPose,steeringCommand,ds);advanceParkingPose(truePose,steeringCommand,ds);
    collision|=!parkingEntryPoseSafe(truePose,steeringCommand);
   }hostTime+=20;
  }
  bool ok=state==FP_SCAN_SETTLE&&!collision;++references;bad+=!ok;
  std::cout<<"WALL ALIGN "<<dir<<" residual="<<residual<<" ready="<<ok<<"\n";
 }
 std::cout<<"WALL ALIGNMENTS "<<references<<" TOTAL_FAILED "<<bad<<"\n";return bad?1:0;
}
'''
    dest=ROOT/'local_workspace/parking-final-arrivals-20261030'
    dest.mkdir(parents=True,exist_ok=True)
    cpp,exe=dest/'check.cpp',dest/'check.exe'
    cpp.write_text(fixture)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),
                    '-I',str(ROOT/'local_workspace/later-laps'),str(cpp),'-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],input='\n'.join(inputs)+'\n',capture_output=True,text=True,timeout=180)
    (dest/'output.txt').write_text(result.stdout+result.stderr)
    print(result.stdout,end='')
    raise SystemExit(result.returncode)


if __name__=='__main__':main()
