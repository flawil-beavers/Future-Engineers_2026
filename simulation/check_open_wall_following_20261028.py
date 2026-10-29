"""Compare unchanged wall control and exercise actual ToF filtering plus Open.

No firmware edits/upload. Baseline navigation is frozen at 861ed8e; both sides
use current configuration, actuator code and sensor filtering to isolate the
navigation change. Hardware observations are injected, not physical evidence.
"""
from pathlib import Path
import shutil
import subprocess

from check_connector_servo_resume import block
import review_open_challenge_20261027 as audit

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'local_workspace/open_wall_following_20261028'


def extra_checks(prefix, nav, historical, helpers):
    OUT.mkdir(parents=True, exist_ok=True)
    compiler = shutil.which('g++') or shutil.which('clang++')
    sensor_source = (ROOT / 'src/sensors.cpp').read_text()
    sensor_fixture = r'''
constexpr int VL53L4CX_ERROR_NONE=0,VL53L4CX_RANGESTATUS_RANGE_VALID=0;
constexpr int VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE=1;
struct Candidate {uint8_t RangeStatus=0;int16_t RangeMilliMeter=300;
 uint32_t SignalRateRtnMegaCps=65536,SigmaMilliMeter=65536;};
struct VL53L4CX_MultiRangingData_t {uint8_t NumberOfObjectsFound=1;Candidate RangeData[4];};
struct VL53L4CX {VL53L4CX_MultiRangingData_t data;
 int VL53L4CX_GetMeasurementDataReady(uint8_t *r){*r=1;return 0;}
 int VL53L4CX_GetMultiRangingData(VL53L4CX_MultiRangingData_t *d){*d=data;return 0;}
 int VL53L4CX_ClearInterruptAndStartMeasurement(){return 0;}
} sensor_left,sensor_right;
'''+block(sensor_source, sensor_source.index('static void read_single_tof('))
    driver = r'''
int main(){
 int cases=0;
 for(int sign:{-1,1})for(float distance:{200.f,250.f,300.f,350.f,550.f})
 for(float heading:{-12.f,-5.f,-1.f,0.f,1.f,5.f,12.f})
 for(float dt:{.001f,.01f,.03f,.05f})for(bool searching:{false,true}){
  reset();selectSide(sign);current_distance=100;nav_searching_for_wall=searching;
  last_loop_time=dt;angle=heading;
  for(int step=0;step<20;++step){
   knownFrames(sign,distance);tick(uint32_t(dt*1000+.5));
   assert(nav_state==NAV_FOLLOWING&&target_speed==300&&!servo_disabled);
   std::cout<<"CONTROL "<<cases<<" "<<step<<" "<<set_degree<<" "<<servo.angle<<" "
    <<target_speed<<" "<<nav_searching_for_wall<<"\n";
  }
  // Once parallel, either side steers away when too close and towards when far.
  if(heading==0&&!searching){
   if(distance<300)assert(sign*set_degree>0);
   if(distance>300)assert(sign*set_degree<0);
   if(distance==300)assert(set_degree==0);
  }
  ++cases;
 }
 // Noisy isolated gaps: transport-valid but poor-quality reports become gaps.
 // Count only frame changes here; the audit separately covers cached-loop votes.
 for(int sign:{-1,1})for(int scenario:{0,1,2}){
  reset();last_loop_time=.001f;current_distance=301;
  frames(300,300);tick(1); // Activate existing 300 ms discovery budget.
  sensor_left.data={};sensor_right.data={};
  auto &inner=sign>0?sensor_left:sensor_right;
  int framesUntilTurn=0;uint32_t firstFrameMs=0;const uint32_t arrivalOrigin=nowMs;
  for(int f=1;f<=20&&nav_state!=NAV_TURNING;++f){
   nowMs=arrivalOrigin+300*f;
   if(scenario==0)inner.data.NumberOfObjectsFound=0;
   else {inner.data.NumberOfObjectsFound=1;inner.data.RangeData[0].RangeMilliMeter=2000;
    inner.data.RangeData[0].SignalRateRtnMegaCps=scenario==1?65536:6553;}
   read_single_tof(sensor_left,tof_distances[TOF_LEFT]);
   read_single_tof(sensor_right,tof_distances[TOF_RIGHT]);
   if(f==1)firstFrameMs=nowMs;
   // Five fast loop passes between actual sensor arrivals (old code counts them).
   for(int loop=0;loop<5&&nav_state!=NAV_TURNING;++loop)tick(1);
   framesUntilTurn=f;
  }
  assert(nav_state==NAV_TURNING&&nav_turn_angle==90*sign);
  std::cout<<"EDGE "<<sign<<" "<<scenario<<" "<<framesUntilTurn<<" "
   <<nowMs-firstFrameMs<<" "<<get_tof_distance(sign>0?TOF_LEFT:TOF_RIGHT)<<"\n";
 }
}
'''

    def run(name, navigation):
        source, binary = OUT / f'{name}.cpp', OUT / f'{name}.exe'
        source.write_text(prefix + audit.without_includes(navigation) +
            helpers + sensor_fixture + driver)
        built = subprocess.run([compiler, '-std=c++17', '-O2', '-I', str(ROOT/'include'),
            str(source), '-o', str(binary)], text=True, capture_output=True)
        (OUT/f'{name}_build.txt').write_text(built.stdout+built.stderr)
        if built.returncode:
            raise RuntimeError(built.stderr)
        result = subprocess.run([str(binary)], text=True, capture_output=True)
        (OUT/f'{name}_output.txt').write_text(result.stdout+result.stderr)
        if result.returncode:
            raise RuntimeError(result.stdout+result.stderr)
        return result.stdout.splitlines()

    current = run('current', nav)
    previous = run('baseline_861ed8e', historical)
    controls = [line for line in current if line.startswith('CONTROL ')]
    assert controls == [line for line in previous if line.startswith('CONTROL ')]
    assert len(controls) == 11200
    print('PASS 560 wall-control scenarios / 11200 commands: identical steering/speed/reacquisition; correct CW/CCW correction signs')
    edges = {}
    for label, lines in [('current', current), ('baseline', previous)]:
        edges[label] = [line for line in lines if line.startswith('EDGE ')]
        print(label + ': ' + '; '.join(edges[label]))
    assert [int(line.split()[3]) for line in edges['current']] == [3,15,3]*2
    assert [int(line.split()[3]) for line in edges['baseline']] == [1,13,1]*2
    print('FINDING current 3 vs old 1 acquisition for empty/rejected frames; valid 300->2000mm transition 15 vs 13 due to existing 100mm/frame clamp')


if __name__ == '__main__':
    audit.main(extra_checks)
