"""Production USBLogger with mocked USB mount/directory and real host file I/O.

Checks large directories, bounded enumeration, 4-KiB durable chunks, complete
recovery after a short write, RAM retention and nonblocking LED feedback.
"""
import shutil
import subprocess

from check_connector_servo_resume import ROOT, block


def main():
    src = (ROOT/'src/logger.cpp').read_text()
    header = (ROOT/'include/logger.h').read_text()
    cls = block(header, header.index('class USBLogger'))
    cls = cls.replace(' : public arduino::Stream', '').replace(' override', '').replace('private:', 'public:')
    code = r'''
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cerrno>
#include <cassert>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <filesystem>
#define LOG_BUFFER_SIZE 196608
constexpr int PA_15=15,LEDR=1,LEDG=2,LOW=0,HIGH=1,OUTPUT=1;
void digitalWrite(int,int){}void pinMode(int,int){}
unsigned long clockMs=1000;unsigned long millis(){return clockMs;}
enum {DC_DISABLED,DC_ENABLED};int dc_state=DC_DISABLED;
struct Printer{operator bool(){return false;}template<class T>void print(T){}template<class T>void println(T){}}terminal;
#define SERIAL_HW terminal
std::vector<std::string> names;int directoryReads=0,opens=0,flushes=0;
std::vector<std::string> saveStages;
bool shortWrite=false,readFault=false;
struct DIR{size_t index=0;};struct dirent{char d_name[256];};
DIR* opendir(const char*){return new DIR;}
dirent* readdir(DIR*d){static dirent e;++directoryReads;
 if(readFault){errno=EIO;return nullptr;}
 if(d->index==names.size())return nullptr;
 snprintf(e.d_name,sizeof(e.d_name),"%s",names[d->index++].c_str());return &e;}
int closedir(DIR*d){delete d;return 0;}
std::string base="local_workspace/logger-directory-save/files/";
FILE* mappedOpen(const char*p,const char*m){++opens;std::string name=p+5;
 FILE*f=std::fopen((base+name).c_str(),m);
 if(f&&m[0]=='a')names.push_back(name);return f;}
int countedFlush(FILE*f){++flushes;return std::fflush(f);}
size_t faultWrite(const void*p,size_t a,size_t n,FILE*f){
 if(shortWrite){shortWrite=false;return std::fwrite(p,a,n/2,f);}
 return std::fwrite(p,a,n,f);}
struct MSD{bool connect(){return true;}}msd;
struct FS{int mount(MSD*){return 0;}int unmount(){return 0;}}usb_fs;
''' + cls + ';\n' + r'''
#define fopen mappedOpen
#define fflush countedFlush
#define fwrite faultWrite
void USBLogger::update_terminal(){}
void USBLogger::log_save_stage(const char*s){
 if(saveStages.empty()||saveStages.back()!=s)saveStages.push_back(s);}
'''
    for sig in ('USBLogger::USBLogger()', 'void USBLogger::clear()', 'bool USBLogger::is_busy() const',
                'void USBLogger::write_to_usb()', 'void USBLogger::begin_attempt()',
                'void USBLogger::remove_written_prefix(', 'void USBLogger::retry_or_fail(',
                'void USBLogger::restart_full_flush(', 'void USBLogger::finish_attempt(', 'void USBLogger::update()'):
        code += block(src, src.index(sig))+'\n'
    code += r'''
#undef fopen
#undef fflush
#undef fwrite
std::string load(const std::string &p){std::ifstream f(p,std::ios::binary);
 return {std::istreambuf_iterator<char>(f),std::istreambuf_iterator<char>()};}
int main(){std::filesystem::create_directories(base);
 for(int i=1;i<=546;++i)names.push_back("log_"+std::to_string(i)+".txt");
 for(auto s:{"notes.txt","log_888.tmp","log_999.txt.bak","log_123garbage","log_-1.txt"})names.push_back(s);
 USBLogger l;std::string original(75852,'x');original.replace(0,9,"RUN_START");original.replace(original.size()-7,7,"RUN_END");
 memcpy(l.log_buffer,original.data(),original.size());l.buffer_head=original.size();l.write_to_usb();
 dc_state=DC_ENABLED;auto stageCount=saveStages.size();l.update();
 assert(directoryReads==0&&opens==0&&saveStages.size()==stageCount);
 dc_state=DC_DISABLED;int updates=0;
 while(l.is_busy()){clockMs+=400;int before=directoryReads;l.update();
  assert(directoryReads-before<=16);assert(++updates<200);}
 assert(l.logger_state==l.LOGGER_SUCCESS_FEEDBACK&&l.buffer_head==0);
 assert(load(base+"log_547.txt")==original);assert(opens==3);
 assert(flushes==20); // 19 chunks plus final flush; previous 149+1.
 assert((saveStages==std::vector<std::string>{"power_settle","connect","mount",
     "directory_open","directory_scan","file_open","write_flush","finalize_verify"}));
 std::cout<<"LOGGER 546 files: "<<directoryReads<<" directory reads, "<<opens<<" opens, "<<flushes<<" flushes, complete snapshot\n";
 l.clear();assert(l.logger_state==l.LOGGER_IDLE);
 // An interrupted write retains the whole RAM prefix and retries to a NEW
 // file. The uncertain partial file is never appended as if it were complete.
 memcpy(l.log_buffer,original.data(),original.size());l.buffer_head=original.size();shortWrite=true;l.write_to_usb();updates=0;
 while(l.is_busy()){clockMs+=400;l.update();assert(++updates<200);}
 assert(load(base+"log_548.txt").size()==2048);
 assert(load(base+"log_549.txt")==original&&l.buffer_head==0);
 // A directory error cannot erase RAM or falsely report successful saving.
 l.clear();memcpy(l.log_buffer,original.data(),original.size());l.buffer_head=original.size();readFault=true;l.write_to_usb();updates=0;
 while(l.is_busy()){clockMs+=400;l.update();assert(++updates<200);}
 assert(l.logger_state==l.LOGGER_ERROR_FEEDBACK&&l.buffer_head==original.size());
 l.clear();assert(l.buffer_head==original.size());
 std::cout<<"PASS short-write complete recovery, directory failure RAM retained, feedback permits start\n";
}
'''
    dest = ROOT/'local_workspace/logger-directory-save'
    dest.mkdir(exist_ok=True)
    # Unique run directory avoids collisions with earlier test output.
    existing = len(list(dest.glob('run_*')))
    folder = dest/f'run_{existing}'
    folder.mkdir()
    code = code.replace('local_workspace/logger-directory-save/files/', folder.relative_to(ROOT).as_posix()+'/files/')
    cpp, exe = dest/'check.cpp', dest/'check.exe'
    cpp.write_text(code)
    subprocess.run([shutil.which('g++'), '-std=c++17', '-O2', str(cpp), '-o', str(exe)], check=True)
    result = subprocess.run([str(exe)], capture_output=True, text=True)
    (dest/'output.txt').write_text(result.stdout+result.stderr)
    print(result.stdout+result.stderr)
    raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
