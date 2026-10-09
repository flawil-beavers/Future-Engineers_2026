"""Compile the real M7 telemetry writer with a bounded logger and cached pose."""
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    folder = ROOT / 'local_workspace/run-telemetry-regression'
    folder.mkdir(parents=True, exist_ok=True)
    (folder / 'Arduino.h').write_text('#pragma once\n#include <stdint.h>\nuint32_t millis();\n')
    (folder / 'position_estimator.h').write_text(
        '#pragma once\nstruct PositionEstimate {float x_mm,y_mm,heading_deg;};\n'
        'PositionEstimate get_position_struct();\n')
    (folder / 'logger.h').write_text(r'''
#pragma once
#include <string>
#include <stdint.h>
struct Logger {
 std::string text; size_t reserve=0;
 size_t remaining() const {return 192*1024-1-text.size();}
 void reserve_tail(size_t n){reserve=n;}
 void write(const uint8_t *p,size_t n){
  for(size_t i=0;i<n&&remaining()>reserve;++i)text+=(char)p[i];
 }
};
extern Logger robot_logger;
''')
    (folder / 'fixture.cpp').write_text(r'''
#include "run_telemetry.h"
#include "logger.h"
#include "position_estimator.h"
#include <cassert>
#include <iostream>
Logger robot_logger;uint32_t now=1000;unsigned reads=0;
uint32_t millis(){return now;}
PositionEstimate get_position_struct(){++reads;return {100,-1200,90};}
unsigned occurrences(const std::string &s){unsigned n=0;size_t at=0;while((at=robot_logger.text.find(s,at))!=std::string::npos){++n;at+=s.size();}return n;}
int main(){
 run_telemetry_start();run_telemetry_tick();assert(reads==0);
 now+=249;run_telemetry_tick();assert(reads==0);
 ++now;run_telemetry_tick();assert(reads==1);
 run_telemetry_phase("connector",1);run_telemetry_tick();assert(reads==2);
 run_telemetry_pose_change("rebase",0,0,0,100,-1200,90);
 now+=250;run_telemetry_tick();assert(robot_logger.text.find("frame=1 space=field")!=std::string::npos);
 float points[2][3]={{100,-1200,90},{200,-1200,90}};
 run_telemetry_route("connector",points,2,sizeof(points[0]),false);
 assert(occurrences("[RUN_ROUTE_POINT]")==2);
 run_telemetry_route("connector",points,2,sizeof(points[0]),false);
 assert(occurrences("[RUN_ROUTE]")==1);
 points[1][0]=250;run_telemetry_route("connector",points,2,sizeof(points[0]),false);
 assert(occurrences("[RUN_ROUTE_POINT]")==3);
 assert(robot_logger.text.find("route=2 base=1")!=std::string::npos);
 now+=1000;run_telemetry_motion("hold");run_telemetry_motion("hold");
 assert(occurrences("kind=motor_hold")==1);
 now+=5000;run_telemetry_lap(1);
 assert(robot_logger.text.find("lap_ms=6500")!=std::string::npos);
 run_telemetry_finish("completed","final_parking_stop");
 assert(robot_logger.text.find("elapsed_ms=6500 outcome=completed")!=std::string::npos);
 run_telemetry_finish("failed","duplicate");assert(occurrences("[RUN_END]")==1);
 // Long run reaches the sample budget, preserving events and completion time.
 robot_logger.text.clear();now=0;run_telemetry_start();
 for(unsigned i=0;i<4000;++i){now+=250;run_telemetry_tick();}
 assert(occurrences("[RUN_TRUNCATED]")==1);
 auto sampleEnd=robot_logger.text.size();now+=250;run_telemetry_tick();
 assert(robot_logger.text.size()==sampleEnd);
 run_telemetry_lap(1);assert(occurrences("[RUN_LAP]")==1);
 // Ordinary diagnostics fill the buffer but cannot consume the footer reserve.
 const std::string filler(192*1024,'x');robot_logger.write((const uint8_t*)filler.data(),filler.size());
 run_telemetry_finish("stopped","operator_stop");
 assert(robot_logger.text.find("outcome=stopped reason=operator_stop truncated=1")!=std::string::npos);
 assert(robot_logger.text.size()<192*1024);
 // Route transactions preflight their entire output; no partial version is emitted.
 robot_logger.text.clear();now=0;run_telemetry_start();
 float large[192][3]={};for(unsigned i=0;i<192;++i)large[i][0]=i;
 for(unsigned n=0;n<10;++n){for(auto &p:large)p[1]=n;run_telemetry_route("learned",large,192,sizeof(large[0]),true);}
 assert(occurrences("[RUN_ROUTE]")==occurrences("[RUN_ROUTE_END]"));
 assert(occurrences("[RUN_TRUNCATED]")==1);run_telemetry_finish("aborted","test");
 // millis() wrap preserves durations because the writer subtracts unsigned times.
 robot_logger.text.clear();now=UINT32_MAX-100;run_telemetry_start();now+=500;
 run_telemetry_finish("completed","test");
 assert(robot_logger.text.find("elapsed_ms=500 outcome=completed")!=std::string::npos);
 std::cout<<"PASS: cadence, cached pose, route deltas, holds, lap/completion time, budgets, footer, wrap\n";
}
''')
    compiler = shutil.which('g++') or shutil.which('clang++')
    if not compiler:
        raise RuntimeError('A host C++ compiler is required')
    binary = folder / 'fixture.exe'
    subprocess.run([compiler, '-std=c++17', '-DARDUINO', '-I' + str(folder),
                    '-I' + str(ROOT / 'include'), str(ROOT / 'src/run_telemetry.cpp'),
                    str(folder / 'fixture.cpp'), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
