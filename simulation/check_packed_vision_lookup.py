"""Exhaustively compare the production packed LUT with its original classifier."""
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    dest = ROOT / 'local_workspace/packed-vision-lookup'
    dest.mkdir(parents=True, exist_ok=True)
    (dest / 'Arduino.h').write_text('''#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <cmath>
using std::min;using std::max;
constexpr int A0=0,A1=1,A2=2;
constexpr double PI=3.141592653589793;
inline uint32_t micros(){return 0;}
''')
    cpp, exe = dest / 'check.cpp', dest / 'check.exe'
    cpp.write_text(r'''
#define private public
#include "vision.h"
#undef private
#include <cassert>
#include <iostream>
#include <algorithm>
#include <cstring>
static Vision vision;
int main(){
 static_assert(sizeof(vision.colorLookup)==32768,"Lookup must release32KiB");
 for(int initialization=0;initialization<2;++initialization){
  vision.begin();
  for(unsigned raw=0;raw<65536;++raw){
   unsigned r=(raw>>11)&31,g=(raw>>5)&63,b=raw&31;
   RGB rgb{uint8_t((r<<3)|(r>>2)),uint8_t((g<<2)|(g>>4)),uint8_t((b<<3)|(b>>2))};
   auto expected=vision.classifyColor(vision.rgbToHSV(rgb));
   assert(vision.lookupColor(raw)==static_cast<unsigned>(expected));
  }
 }
 std::cout<<"PACKED LUT 131072 exact colour comparisons PASS;32768B released\n";
}
''')
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2',
                    '-I', str(dest), '-I', str(ROOT / 'include'),
                    str(ROOT / 'src/vision.cpp'), str(cpp), '-o', str(exe)], check=True)
    result = subprocess.run([str(exe)], capture_output=True, text=True)
    (dest / 'output.txt').write_text(result.stdout + result.stderr)
    print(result.stdout + result.stderr, end='')
    raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
