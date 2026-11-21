"""Run injected error cases against the installed USB-host source corrections."""
import shutil
import subprocess
import sys

from check_connector_servo_resume import ROOT, block

sys.path.insert(0, str(ROOT))
from scripts.usb_host_save_errors import fix_host, fix_mass_storage


def main():
    library = ROOT/'.pio/libdeps/giga_r1_m7/Arduino_USBHostMbed5/src'
    msd = fix_mass_storage((library/'USBHostMSD/USBHostMSD.cpp').read_text())
    host = fix_host((library/'USBHost/USBHost.cpp').read_text())
    source = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#define USB_DBG(...) ((void)0)
#define USBHOST_OTHER
namespace rtos { struct ThisThread {static void sleep_for(int){}}; }
constexpr unsigned CBW_SIGNATURE=0x43425355,CSW_SIGNATURE=0x53425355;
constexpr int HOST_TO_DEVICE=0,DEVICE_TO_HOST=0x80,BO_MASS_STORAGE_RESET=255;
constexpr int USB_RECIPIENT_INTERFACE=1,USB_HOST_TO_DEVICE=0,USB_REQUEST_TYPE_CLASS=32;
constexpr int USB_RECIPIENT_ENDPOINT=2,USB_REQUEST_TYPE_STANDARD=0,CLEAR_FEATURE=1;
struct Endpoint {int getAddress(){return 1;}};
struct Transport {
 int status=0;
 bool sawTestUnitReady=false;
 int bulkWrite(void*,Endpoint*,uint8_t* data,int n){
  if(n==31 && data[15]==0) sawTestUnitReady=true;
  return 0;
 }
 int bulkRead(void*,Endpoint*,uint8_t* data,int n){
  if(n==13){memcpy(data,&CSW_SIGNATURE,4);data[4]=static_cast<uint8_t>(status);}
  return 0;
 }
 int controlWrite(void*,int,int,int,int,void*,int){return 0;}
};
struct USBHostMSD {
 Transport transport;
 Transport* host=&transport;
 void* dev=nullptr;
 Endpoint ep;
 Endpoint* bulk_in=&ep,*bulk_out=&ep;
 int msd_intf=0,requests=0,capacityStatus=0,blockSize=512,blockCount=100;
 bool disk_init=false;
 struct {unsigned Signature,Tag,DataLength;uint8_t Flags,LUN,CBLength,CB[16];} cbw;
 struct {unsigned Signature;uint8_t Status;} csw;
 int checkResult(int result,Endpoint*){return result;}
 int SCSIRequestSense(){++requests;return 0;}
 int getMaxLun(){return 0;}
 int inquiry(int,int){return 0;}
 int readCapacity(){return capacityStatus;}
 int testUnitReady();
 int init();
 int SCSITransfer(uint8_t*,uint8_t,int,uint8_t*,uint32_t);
};
enum USB_TYPE {USB_TYPE_ERROR,USB_TYPE_PROCESSING};
struct HCTD {uint8_t* currBufPtr;uint32_t size;};
struct USBEndpoint {
 HCTD descriptor{};
 bool available=false;
 volatile HCTD* getNextTD(){return available?&descriptor:nullptr;}
 USB_TYPE queueTransfer(){return USB_TYPE_PROCESSING;}
};
struct USBHost {
 struct Mutex {bool locked=false;void lock(){assert(!locked);locked=true;}
  void unlock(){assert(locked);locked=false;}} td_mutex;
 USB_TYPE addTransfer(USBEndpoint*,uint8_t*,uint32_t);
};
'''
    for text, signature in (
        (msd, 'int USBHostMSD::SCSITransfer('),
        (msd, 'int USBHostMSD::testUnitReady()'),
        (msd, 'int USBHostMSD::init()'),
        (host, 'USB_TYPE USBHost::addTransfer('),
    ):
        source += block(text, text.index(signature))+'\n'
    source += r'''
int main(){
 USBHostMSD storage;
 uint8_t write[10]={0x2a};
 storage.transport.status=1;
 assert(storage.SCSITransfer(write,10,HOST_TO_DEVICE,nullptr,0)!=0);
 assert(storage.requests==1);
 assert(storage.testUnitReady()!=0&&storage.transport.sawTestUnitReady);
 storage.transport.status=0;
 storage.capacityStatus=-1;
 assert(storage.init()!=0&&!storage.disk_init);
 storage.capacityStatus=0;storage.blockSize=0;
 assert(storage.init()!=0&&!storage.disk_init);
 storage.blockSize=512;
 assert(storage.init()==0&&storage.disk_init);
 USBHost host;USBEndpoint endpoint;
 assert(host.addTransfer(&endpoint,nullptr,0)==USB_TYPE_ERROR&&!host.td_mutex.locked);
 endpoint.available=true;
 assert(host.addTransfer(&endpoint,nullptr,0)==USB_TYPE_PROCESSING&&!host.td_mutex.locked);
 std::cout<<"PASS failed SCSI write, not-ready response, invalid capacity and TD-lock recovery\n";
}
'''
    work = ROOT/'local_workspace/usb-host-save-errors'
    work.mkdir(parents=True, exist_ok=True)
    cpp, executable = work/'check.cpp', work/'check.exe'
    cpp.write_text(source, encoding='utf-8')
    subprocess.run([shutil.which('g++'), '-std=c++17', str(cpp), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    main()
