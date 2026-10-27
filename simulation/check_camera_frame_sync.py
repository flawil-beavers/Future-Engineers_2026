"""Exercise production VSYNC observations and nonblocking camera resynchronization.

Hardware registers/transfer completion are modeled; pixel correctness still
requires real CRC-checked images. Generated fixtures stay in local_workspace.
"""
import shutil
import subprocess
from pathlib import Path
from check_connector_servo_resume import block
ROOT=Path(__file__).resolve().parents[1]

def main():
    dest=ROOT/'local_workspace/camera-frame-sync';dest.mkdir(parents=True,exist_ok=True)
    (dest/'Arduino.h').write_text("""#pragma once
#include <cstdint>
#include <cmath>
using std::isfinite;
constexpr int A0=0,A1=1,A2=2;
constexpr float PI=3.14159265358979323846f;
uint32_t micros();
""")
    driver=(ROOT/'lib/arducam_dvp_async/src/arducam_dvp.cpp').read_text()
    header=(ROOT/'lib/arducam_dvp_async/src/arducam_dvp.h').read_text()
    camera=(ROOT/'src/camera.cpp').read_text()
    code=r'''#include <Arduino.h>
#include "config.h"
#include <iostream>
uint32_t clockUs=1000;uint32_t micros(){return clockUs++;}
uint32_t mask=0;uint32_t __get_PRIMASK(){return mask;}
void __disable_irq(){mask=1;}void __enable_irq(){mask=0;}
constexpr uint32_t DMA_SxCR_CT=1U<<19,DCMI_CR_CAPTURE=1;
struct DMA_Stream_TypeDef{uint32_t NDTR=38400,CR=0;} stream;
struct DCMIRegs{uint32_t CR=1,SR=3;} regs;
struct DMA_HandleTypeDef{void*Instance;} hdma{&stream};
struct DCMI_HandleTypeDef{DCMIRegs*Instance;uint32_t ErrorCode=0;} hdcmi{&regs};
''' + block(header,header.index('struct CameraSyncDiagnostics'))+';\n'
    a=driver.index('static volatile uint32_t dcmi_frame_completed_us')
    b=driver.index('static void dcmiContinuousBufferComplete',a)
    code+=driver[a:b]
    code+=r'''
void resetDriver(){
 dcmi_continuous_active=true;dcmi_continuous_sequence=0;
 dcmi_sync_vsync_count=dcmi_sync_dma_sequence=dcmi_sync_words=0;
 dcmi_sync_min_words=UINT32_MAX;dcmi_sync_max_words=0;
 dcmi_sync_good_phase_streak=dcmi_sync_bad_phase_streak=dcmi_sync_bad_phase_count=0;
 dcmi_continuous_frame_words=38400;dcmi_sync_error_bits=0;
 stream.NDTR=38400;stream.CR=0;regs.CR=1;regs.SR=3;mask=0;
}
struct FrameBuffer{};
class Camera{public:
 int requests=0,stops=0,starts=0,acquires=0;
 FrameBuffer*output=nullptr;
 void continuousSyncDiagnostics(CameraSyncDiagnostics&)const;
 void requestContinuousStop();bool continuousStopReady()const;
 int stopContinuous(){++stops;if(regs.CR&DCMI_CR_CAPTURE)return -1;return 0;}
 int startContinuous(FrameBuffer&,FrameBuffer&){++starts;resetDriver();return 0;}
 bool acquireContinuousFrame(uint32_t consumed,FrameBuffer*&f,uint32_t&s,uint32_t&t){
  ++acquires;if(!dcmi_continuous_sequence||dcmi_continuous_sequence==consumed)return false;
  f=output;s=dcmi_continuous_sequence;t=micros();return true;
 }
};
'''
    for sig in ('void HAL_DCMI_VsyncEventCallback(',
                'void Camera::continuousSyncDiagnostics(',
                'void Camera::requestContinuousStop(',
                'bool Camera::continuousStopReady()'):
        code+=block(driver,driver.index(sig))+'\n'
    code+=r'''
class CameraSystem{public:
 Camera camera;FrameBuffer frameA,frameB;
 bool resyncPending=false;uint32_t autoResyncCount=0,autoResyncLastWords=0,autoResyncMaxServiceUs=0;
 uint8_t readyFrame=0;uint32_t acquiredFrameSequence=0,previousFrameCompletedUs=0,captureStartedUs=0;
 uint32_t lastCaptureTimeUs=0,lastReadyWaitTimeUs=0,lastFrameIntervalUs=0;
 uint32_t minFrameIntervalUs=UINT32_MAX,maxFrameIntervalUs=0,longFrameIntervalCount=0;
 uint32_t lastServiceTimeUs=0,completedFrameCount=0,discardedFrameCount=0;
 bool capture();
};
''' + block(camera,camera.index('bool CameraSystem::capture()'))
    code+=r'''
int main(){int bad=0,n=0;auto check=[&](bool ok){++n;if(!ok){++bad;std::cout<<"FAIL "<<n<<"\n";}};
 resetDriver();CameraSystem c;c.camera.output=&c.frameB;
 check(!c.capture()&&c.camera.acquires==0);
 dcmi_continuous_active=false;HAL_DCMI_VsyncEventCallback(&hdcmi);
 check(dcmi_sync_vsync_count==0);dcmi_continuous_active=true;
 // Reproduce measured eight-word phase error. ISR observes; it never writes
 // DMA pointers/count or capture enable. Main loop keeps returning promptly.
 stream.NDTR=38392;uint32_t cr=stream.CR,control=regs.CR;
 for(int i=1;i<=2;i++){
  HAL_DCMI_VsyncEventCallback(&hdcmi);check(!c.capture()&&!c.resyncPending&&c.camera.stops==0);
 }
 check(stream.NDTR==38392&&stream.CR==cr&&regs.CR==control);
 HAL_DCMI_VsyncEventCallback(&hdcmi);check(!c.capture()&&c.resyncPending&&c.autoResyncLastWords==38392);
 check((regs.CR&DCMI_CR_CAPTURE)==0&&!dcmi_continuous_active&&c.camera.stops==0);
 // Emulate hardware retaining CAPTURE until end of the current sensor frame.
 regs.CR=DCMI_CR_CAPTURE;check(!c.capture()&&c.camera.stops==0&&c.camera.starts==0);
 regs.CR=0;check(!c.capture()&&c.camera.stops==1&&c.camera.starts==1&&c.autoResyncCount==1&&!c.resyncPending);
 HAL_DCMI_VsyncEventCallback(&hdcmi);check(!c.capture()&&c.camera.acquires==0);
 dcmi_continuous_sequence=1;HAL_DCMI_VsyncEventCallback(&hdcmi);
 check(c.capture()&&c.readyFrame==1&&c.completedFrameCount==1&&c.camera.acquires==1);
 check(c.autoResyncMaxServiceUs<10);
 CameraSyncDiagnostics d;mask=1;c.camera.continuousSyncDiagnostics(d);
 check(mask==1&&d.goodPhaseStreak==2&&d.badPhaseStreak==0);mask=0;
 c.camera.continuousSyncDiagnostics(d);check(mask==0);
 // A single delayed/misaligned observation never restarts capture. Two
 // subsequent clean boundaries restore publication of complete new buffers.
 resetDriver();CameraSystem t;t.camera.output=&t.frameA;
 stream.NDTR=38392;HAL_DCMI_VsyncEventCallback(&hdcmi);check(!t.capture()&&!t.resyncPending);
 stream.NDTR=38400;HAL_DCMI_VsyncEventCallback(&hdcmi);check(!t.capture());
 dcmi_continuous_sequence=1;HAL_DCMI_VsyncEventCallback(&hdcmi);
 check(t.capture()&&t.autoResyncCount==0&&t.camera.stops==0);
 std::cout<<"CAMERA FRAME SYNC "<<n<<" failures "<<bad<<"\n";return bad?1:0;
}
'''
    cpp=dest/'check.cpp';exe=dest/'check.exe';cpp.write_text(code)
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-I',str(dest),'-I',str(ROOT/'include'),str(cpp),'-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],text=True,capture_output=True)
    (dest/'output.txt').write_text(result.stdout+result.stderr);print(result.stdout,end='')
    raise SystemExit(result.returncode)

if __name__=='__main__':main()
