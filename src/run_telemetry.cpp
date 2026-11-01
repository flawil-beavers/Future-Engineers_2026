#include "run_telemetry.h"
#ifdef ARDUINO
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "logger.h"
#include "position_estimator.h"
#include "sensors.h"

namespace {
constexpr size_t SAMPLE_BUDGET=RUN_TELEMETRY_POSE_ONCE ? 96*1024:64*1024, ROUTE_BUDGET=24*1024;
constexpr size_t EVENT_BUDGET=RUN_TELEMETRY_POSE_ONCE ? 16*1024:(RUN_TELEMETRY_DETAILED ? 8*1024 : 2*1024);
constexpr size_t TOTAL_BUDGET=RUN_TELEMETRY_POSE_ONCE ? 112*1024:(RUN_TELEMETRY_DETAILED ? 24*1024 : 2*1024);
constexpr unsigned MAX_POINTS=192;
bool active=false, truncated=false, field=false;
bool poseCapture=false,poseCaptureConsumed=false;
bool firstDriveLogged=false;
uint32_t started=0,lastSample=0,lastLap=0,revision=0;
unsigned lapNumber=0,frame=0,routeCount=0;
size_t sampleBytes=0,routeBytes=0,eventBytes=0;
uint32_t hashes[MAX_POINTS]={};
char phase[40]="start",routeKind[24]="none";
char motion[16]="unknown";
bool routeClosed=false;
// The ordinary logger cannot consume this footer reserve while a run is active.
constexpr size_t FOOTER_RESERVE=768, ORDINARY_RESERVE=8192;
void markTruncated() {
    if(truncated)return;
    truncated=true;
    const char *line="[RUN_TRUNCATED] v=1 reason=telemetry_limit\r\n";
    // One bounded marker uses the important-event reserve, even if samples fill it.
    if(robot_logger.remaining()>strlen(line)+FOOTER_RESERVE)
        robot_logger.write(reinterpret_cast<const uint8_t *>(line),strlen(line));
}
bool emit(const char *line,size_t &used,size_t budget) {
    const size_t length=strlen(line);
    if(used+length>budget || sampleBytes+routeBytes+eventBytes+length>TOTAL_BUDGET || robot_logger.remaining()<length+ORDINARY_RESERVE) {
        markTruncated(); return false;
    }
    robot_logger.write(reinterpret_cast<const uint8_t *>(line),length);
    used+=length; return true;
}
uint32_t hashPoint(const float *p) {
    uint32_t h=2166136261u;
    for(unsigned i=0;i<3;++i) {uint32_t v;memcpy(&v,p+i,sizeof(v));h=(h^v)*16777619u;}
    return h;
}
uint32_t hashPointAt(const void *data,size_t stride,unsigned index) {
    float point[3];
    memcpy(point,static_cast<const uint8_t *>(data)+index*stride,sizeof(point));
    return hashPoint(point);
}
}
bool run_telemetry_pose_capture_active(){return active && poseCapture;}
void run_telemetry_start() {
    poseCapture=RUN_TELEMETRY_POSE_ONCE && !poseCaptureConsumed;
    firstDriveLogged=false;
    if(poseCapture)poseCaptureConsumed=true;
    active=true;truncated=false;field=false;frame=revision=routeCount=lapNumber=0;
    sampleBytes=routeBytes=eventBytes=0;started=lastSample=lastLap=millis();
    strcpy(phase,"start");strcpy(routeKind,"none");strcpy(motion,"unknown");if(RUN_TELEMETRY_DETAILED)memset(hashes,0,sizeof(hashes));
    robot_logger.reserve_tail(FOOTER_RESERVE);
    char line[180];snprintf(line,sizeof(line),"[RUN_START] v=1 t=%lu period_ms=%u detailed=%u total_bytes=%u\r\n",(unsigned long)started,RUN_TELEMETRY_DETAILED?250U:0U,unsigned(RUN_TELEMETRY_DETAILED),unsigned(TOTAL_BUDGET));
    emit(line,eventBytes,EVENT_BUDGET);
    if(poseCapture){
        const char *schema="[RUN_CAPTURE] v=2 once_per_boot=1 period_ms=200 adaptive_after_bytes=73728 slow_period_ms=500 final_after_bytes=81920 final_period_ms=1000 pose=estimated units=mm,deg space=L_local_F_field tof=left,right,rear invalid=-1 out_of_range=9999\r\n";
        emit(schema,eventBytes,EVENT_BUDGET);
    }
}
void run_telemetry_phase(const char *value,unsigned lap) {
    if(!RUN_TELEMETRY_DETAILED && !poseCapture)return;
    if(!active)return;
    if(strcmp(value,phase)==0&&lap==lapNumber)return;
    strncpy(phase,value,sizeof(phase)-1);phase[sizeof(phase)-1]=0;lapNumber=lap;
    char line[170];snprintf(line,sizeof(line),"[RUN_EVENT] v=1 t=%lu elapsed_ms=%lu kind=phase phase=%s lap=%u route=%lu\r\n",(unsigned long)millis(),(unsigned long)(millis()-started),phase,lap,(unsigned long)revision);
    emit(line,eventBytes,EVENT_BUDGET);
    // A phase notification must never defeat the dense-capture rate limit.
    //619: conflicting producers otherwise generated hundreds of same-ms poses.
    if(!poseCapture)lastSample=millis()-250;
}
void run_telemetry_tick() {
    if(!RUN_TELEMETRY_DETAILED && !poseCapture)return;
    const uint32_t period=poseCapture ? (sampleBytes<72*1024 ? 200U:(sampleBytes<80*1024 ? 500U:1000U)):250U;
    if(!active||millis()-lastSample<period)return;
    lastSample=millis();const PositionEstimate p=get_position_struct();
    if(!isfinite(p.x_mm)||!isfinite(p.y_mm)||!isfinite(p.heading_deg))return;
    char line[220];
    if(poseCapture){
        const auto tof=[](TofSensor sensor){const float d=get_tof_distance(sensor);
            return isfinite(d)&&d>0 ? int(lroundf(fminf(d,9999.0f))):-1;};
        snprintf(line,sizeof(line),"[RUN_POSE] v=2 t=%lu lap=%u frame=%u space=%c pose=%.1f,%.1f,%.1f tof=%d,%d,%d\r\n",(unsigned long)lastSample,lapNumber,frame,field?'F':'L',p.x_mm,p.y_mm,p.heading_deg,tof(TOF_LEFT),tof(TOF_RIGHT),tof(TOF_REAR));
    }
    else snprintf(line,sizeof(line),"[RUN_POSE] v=1 t=%lu elapsed_ms=%lu phase=%s lap=%u route=%lu frame=%u space=%s pose=%.1f,%.1f,%.1f\r\n",(unsigned long)lastSample,(unsigned long)(lastSample-started),phase,lapNumber,(unsigned long)revision,frame,field?"field":"local",p.x_mm,p.y_mm,p.heading_deg);
    emit(line,sampleBytes,SAMPLE_BUDGET);
}
void run_telemetry_lap(unsigned lap) {
    if(!active)return;
    const uint32_t now=millis();char line[180];
    snprintf(line,sizeof(line),"[RUN_LAP] v=1 t=%lu lap=%u elapsed_ms=%lu lap_ms=%lu\r\n",(unsigned long)now,lap,(unsigned long)(now-started),(unsigned long)(now-lastLap));
    emit(line,eventBytes,EVENT_BUDGET);lastLap=now;
}
void run_telemetry_motion(const char *state) {
    // Dense pose capture records stationary positions too; avoid motor-event
    //chatter consuming the finite buffer during this one visualization run.
    if(poseCapture){
        if(active && !firstDriveLogged && strcmp(state,"drive")==0){
            firstDriveLogged=true;char line[140];
            snprintf(line,sizeof(line),"[RUN_SYNC] v=1 t=%lu elapsed_ms=%lu kind=first_drive_request\r\n",(unsigned long)millis(),(unsigned long)(millis()-started));
            emit(line,eventBytes,EVENT_BUDGET);
        }
        return;
    }
    if(!RUN_TELEMETRY_DETAILED)return;
    if(!active||strcmp(state,motion)==0)return;
    strncpy(motion,state,sizeof(motion)-1);motion[sizeof(motion)-1]=0;
    const PositionEstimate p=get_position_struct();char line[220];
    snprintf(line,sizeof(line),"[RUN_EVENT] v=1 t=%lu elapsed_ms=%lu kind=motor_%s phase=%s lap=%u route=%lu pose=%.1f,%.1f,%.1f\r\n",(unsigned long)millis(),(unsigned long)(millis()-started),motion,phase,lapNumber,(unsigned long)revision,p.x_mm,p.y_mm,p.heading_deg);
    emit(line,eventBytes,EVENT_BUDGET);
    lastSample=millis()-250;
}
void run_telemetry_obstacle(unsigned seat,bool red,float x,float y){
    if(!active || (!RUN_TELEMETRY_DETAILED && !poseCapture))return;
    const PositionEstimate p=get_position_struct();char line[240];
    snprintf(line,sizeof(line),"[RUN_OBSTACLE] v=1 t=%lu elapsed_ms=%lu lap=%u seat=%u color=%s xy=%.1f,%.1f robot=%.1f,%.1f,%.1f\r\n",(unsigned long)millis(),(unsigned long)(millis()-started),lapNumber,seat,red?"RED":"GREEN",x,y,p.x_mm,p.y_mm,p.heading_deg);
    emit(line,eventBytes,EVENT_BUDGET);
}
void run_telemetry_pose_change(const char *kind,float bx,float by,float bh,float ax,float ay,float ah) {
    if(!RUN_TELEMETRY_DETAILED && !poseCapture)return;
    if(!active)return;
    if(poseCapture && strcmp(kind,"rebase")!=0 && hypotf(ax-bx,ay-by)<5.0f && fabsf(ah-bh)<1.0f)return;
    ++frame;if(strcmp(kind,"rebase")==0)field=true;
    char line[260];snprintf(line,sizeof(line),"[RUN_EVENT] v=1 t=%lu kind=%s frame=%u before=%.1f,%.1f,%.1f after=%.1f,%.1f,%.1f\r\n",(unsigned long)millis(),kind,frame,bx,by,bh,ax,ay,ah);
    emit(line,eventBytes,EVENT_BUDGET);
}
void run_telemetry_route(const char *kind,const void *data,unsigned count,size_t stride,bool closed) {
    if(!RUN_TELEMETRY_DETAILED)return;
    if(!active)return;
    if(count>MAX_POINTS||stride<3*sizeof(float)||(count&&!data)){markTruncated();return;}
    for(unsigned i=0;i<count;++i){float p[3];memcpy(p,static_cast<const uint8_t *>(data)+i*stride,sizeof(p));if(!isfinite(p[0])||!isfinite(p[1])||!isfinite(p[2])){markTruncated();return;}}
    const bool full=strcmp(kind,routeKind)!=0||routeClosed!=closed||routeCount!=count;
    bool changed=full;
    for(unsigned i=0;i<count;++i)changed|=hashPointAt(data,stride,i)!=hashes[i];
    if(!changed)return;
    const uint32_t next=revision+1;
    char line[240];size_t required=0;
    // Preflight the complete transaction so partial route versions cannot be drawn.
    for(unsigned i=0;i<count;++i) {
        float p[3];memcpy(p,static_cast<const uint8_t *>(data)+i*stride,sizeof(p));
        if(full||hashPoint(p)!=hashes[i])required+=snprintf(line,sizeof(line),"[RUN_ROUTE_POINT] v=1 route=%lu index=%u pose=%.1f,%.1f,%.1f\r\n",(unsigned long)next,i,p[0],p[1],p[2]);
    }
    required+=snprintf(line,sizeof(line),"[RUN_ROUTE] v=1 t=%lu route=%lu base=%lu kind=%s count=%u closed=%u\r\n",(unsigned long)millis(),(unsigned long)next,(unsigned long)(full?0:revision),kind,count,closed?1:0);
    required+=snprintf(line,sizeof(line),"[RUN_ROUTE_END] v=1 route=%lu\r\n",(unsigned long)next);
    revision=next; // Samples refer to actual route even if its dump is unavailable.
    if(routeBytes+required>ROUTE_BUDGET||robot_logger.remaining()<required+ORDINARY_RESERVE){markTruncated();routeCount=0;strcpy(routeKind,"unlogged");return;}
    snprintf(line,sizeof(line),"[RUN_ROUTE] v=1 t=%lu route=%lu base=%lu kind=%s count=%u closed=%u\r\n",(unsigned long)millis(),(unsigned long)next,(unsigned long)(full?0:next-1),kind,count,closed?1:0);emit(line,routeBytes,ROUTE_BUDGET);
    for(unsigned i=0;i<count;++i){float p[3];memcpy(p,static_cast<const uint8_t *>(data)+i*stride,sizeof(p));const uint32_t h=hashPoint(p);if(full||h!=hashes[i]){snprintf(line,sizeof(line),"[RUN_ROUTE_POINT] v=1 route=%lu index=%u pose=%.1f,%.1f,%.1f\r\n",(unsigned long)next,i,p[0],p[1],p[2]);emit(line,routeBytes,ROUTE_BUDGET);}hashes[i]=h;}
    snprintf(line,sizeof(line),"[RUN_ROUTE_END] v=1 route=%lu\r\n",(unsigned long)next);emit(line,routeBytes,ROUTE_BUDGET);
    routeCount=count;routeClosed=closed;strncpy(routeKind,kind,sizeof(routeKind)-1);
}
void run_telemetry_finish(const char *outcome,const char *reason) {
    if(!active)return;
    lastSample=millis()-1000;run_telemetry_tick();const uint32_t now=millis();char line[260];
    snprintf(line,sizeof(line),"[RUN_END] v=1 t=%lu elapsed_ms=%lu outcome=%s reason=%s truncated=%u sample_bytes=%u route_bytes=%u event_bytes=%u\r\n",(unsigned long)now,(unsigned long)(now-started),outcome,reason,truncated?1:0,(unsigned)sampleBytes,(unsigned)routeBytes,(unsigned)eventBytes);
    robot_logger.reserve_tail(0);robot_logger.write(reinterpret_cast<const uint8_t *>(line),strlen(line));active=false;
}
#endif
