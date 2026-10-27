"""Run the generated vendor I/O wrappers with a faulty TwoWire adapter."""
import importlib.util
import shutil
import subprocess
from check_connector_servo_resume import ROOT,block


def main():
    spec=importlib.util.spec_from_file_location('bounded_driver',ROOT/'scripts/bounded_tof_transport.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    dest=ROOT/'local_workspace/bounded-tof-transport';dest.mkdir(parents=True,exist_ok=True)
    wrappers=[]
    for core in ('giga_r1_m7','giga_r1_m4'):
        vendor=ROOT/'.pio/libdeps'/core/'STM32duino VL53L4CX/src/vl53l4cx_class.cpp'
        source=module.replace_transport(vendor.read_text())
        (dest/f'{core}_vendor.cpp').write_text(source)
        bodies='\n'.join(block(source,source.index('VL53L4CX_Error VL53L4CX::VL53L4CX_I2C'+op+'('))
                         for op in ('Read','Write'))
        wrappers.append(bodies)
    assert wrappers[0]==wrappers[1]
    fixture=r'''
#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>
#include "bounded_tof_transport.h"
using VL53L4CX_Error=int;
constexpr int DEFAULT_I2C_BUFFER_LEN=32;
struct Bus{
 int ends=0,requests=0,emptyAt=-1,endFailAt=-1,shortAt=-1,writeCalls=0;
 int pending=0,value=0,missingAt=-1;std::vector<std::vector<uint8_t>> tx;
 bool alwaysNack=false,alwaysShort=false;
 void beginTransmission(uint8_t a){assert(a==0x29);tx.push_back({});}
 size_t write(const uint8_t *p,size_t n){++writeCalls;
  if(alwaysShort||writeCalls==shortAt)return n-1;
  tx.back().insert(tx.back().end(),p,p+n);return n;}
 int endTransmission(bool){++ends;return alwaysNack||ends==endFailAt?4:0;}
 size_t requestFrom(uint8_t a,size_t n){assert(a==0x29);++requests;
  if(requests==emptyAt){pending=0;return 0;}pending=n;return n;}
 int available(){return value==missingAt?0:pending;}
 int read(){--pending;return value++&255;}
};
class VL53L4CX{public:Bus *dev_i2c;
 VL53L4CX_Error VL53L4CX_I2CRead(uint8_t,uint16_t,uint8_t*,uint16_t);
 VL53L4CX_Error VL53L4CX_I2CWrite(uint8_t,uint16_t,uint8_t*,uint16_t);
};
'''+wrappers[0]+r'''
int main(){uint8_t data[96]={};Bus b;VL53L4CX sensor{&b};
 assert(sensor.VL53L4CX_I2CRead(0x52,0x100,data,96)==0);
 assert(b.ends==1&&b.requests==3&&b.tx[0]==std::vector<uint8_t>({1,0}));
 for(int i=0;i<96;++i)assert(data[i]==i);
 b=Bus{};b.alwaysNack=true;assert(sensor.VL53L4CX_I2CRead(0x52,0x100,data,96)!=0);
 assert(b.ends==2&&b.requests==0); // bounded register-address retry
 b=Bus{};b.endFailAt=1;assert(sensor.VL53L4CX_I2CRead(0x52,0x100,data,96)==0);
 assert(b.ends==2&&b.requests==3); // retain recovery from one transient NACK
 for(int fail=1;fail<=3;++fail){b=Bus{};b.emptyAt=fail;
  assert(sensor.VL53L4CX_I2CRead(0x52,0x100,data,96)!=0);assert(b.requests==fail);
 }
 b=Bus{};b.missingAt=12;assert(sensor.VL53L4CX_I2CRead(0x52,0x100,data,96)!=0);
 b=Bus{};b.alwaysShort=true;assert(sensor.VL53L4CX_I2CRead(0x52,0x100,data,96)!=0);
 assert(b.ends==2&&b.requests==0);
 b=Bus{};assert(sensor.VL53L4CX_I2CWrite(0x52,0x100,data,96)==0);
 assert(b.ends==3&&b.tx.size()==3);
 for(int i=0;i<3;++i){assert(b.tx[i].size()==34&&b.tx[i][0]==1&&b.tx[i][1]==32*i);}
 for(int fail=1;fail<=3;++fail){b=Bus{};b.endFailAt=fail;
  assert(sensor.VL53L4CX_I2CWrite(0x52,0x100,data,96)!=0);assert(b.ends==fail);
 }
 for(int fail=1;fail<=6;++fail){b=Bus{};b.shortAt=fail;
  assert(sensor.VL53L4CX_I2CWrite(0x52,0x100,data,96)!=0);
  assert(b.writeCalls==fail);
 }
 b=Bus{};assert(sensor.VL53L4CX_I2CRead(0x52,0x100,data,96)==0);
 std::cout<<"PASS both vendor wrappers: 96-byte read/write, address NACK, zero/short chunks, every write error, next request recovery\n";
}
'''
    cpp,exe=dest/'check.cpp',dest/'check.exe';cpp.write_text(fixture)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(ROOT/'include'),str(cpp),'-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],capture_output=True,text=True)
    (dest/'output.txt').write_text(result.stdout+result.stderr)
    print(result.stdout,end='');assert result.returncode==0,result.stderr


if __name__=='__main__':main()
