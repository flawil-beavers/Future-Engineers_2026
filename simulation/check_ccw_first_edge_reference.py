"""Execute production ToF first-edge selection with fresh synthetic snapshots.

Tests selection/fallback and frozen edge data; no camera or motor emulation.
Actual scan/connector geometry is checked separately by the CCW planner.
"""
from pathlib import Path
import shutil
import subprocess
from check_connector_servo_resume import block

ROOT = Path(__file__).resolve().parents[1]


def main():
    source = (ROOT/'src/obstacle.cpp').read_text()
    functions = '\n'.join(block(source, source.index(s)) for s in (
        'static ParkingBeamFootprint parkingBeamFootprint(',
        'static float expectedParkingOuterWallRange(',
        'static void processParkingEdgeLocalizationTof()'))
    fixture = r'''
#include <cmath>
#include <cstdint>
#include <iostream>
using std::isfinite;
constexpr int A0=0,A1=1,A2=2;
constexpr float PI=3.14159265358979323846f;
#include "config.h"
enum TofSensor{TOF_LEFT,TOF_RIGHT};
enum ParkingLocalizationPhase{PARKING_LOCALIZE_SEEK_FIRST_MARKER,
 PARKING_LOCALIZE_ON_FIRST_MARKER,PARKING_LOCALIZE_BETWEEN_MARKERS,
 PARKING_LOCALIZE_ON_SECOND_MARKER};
constexpr int OBSTACLE_SECTION_LAYOUT_OFFICIAL=0;
struct PositionEstimate{float x_mm=0,y_mm=0,heading_deg=0;};
struct ParkingBeamFootprint{float centerX,minimumX,maximumX;};
struct TofDiagnosticSnapshot{uint32_t sequence=0;float selected_raw_distance_mm=0;};
struct NullLog{template<class T>void print(T){}template<class T>void println(T){}
 template<class T>void println(T,int){}} Serial;
bool oc_parking_localization_first_edge_reference=false;
bool oc_parking_localization_transition_found=false,oc_parking_localization_piece_seen=false;
bool oc_parking_field_pose_initialized=true;
float oc_parking_localization_first_edge_continue_mm=0;
float oc_parking_localization_last_piece_range=0,oc_parking_localization_wall_range=0;
float oc_parking_localization_start_distance=0;
PositionEstimate oc_parking_localization_last_piece_pose,oc_parking_localization_latest_wall_pose;
TofSensor oc_parking_localization_sensor=TOF_RIGHT;
ParkingLocalizationPhase oc_parking_localization_phase=PARKING_LOCALIZE_SEEK_FIRST_MARKER;
uint32_t oc_parking_localization_tof_sequence=0;
uint8_t oc_parking_localization_marker_frames=0,oc_parking_localization_wall_frames=0;
int oc_parking_exit_steering=-50,layoutMode=0;
PositionEstimate currentPose;
TofDiagnosticSnapshot currentSnapshot;
bool snapshotAvailable=true;
bool get_tof_diagnostic_snapshot(TofSensor,TofDiagnosticSnapshot &s){s=currentSnapshot;return snapshotAvailable;}
PositionEstimate get_position_struct(){return currentPose;}
float distanceSince(float){return 50;}
int obstacle_path_section_layout_mode(){return layoutMode;}
'''+functions+r'''
void reset(){
 oc_parking_localization_first_edge_reference=false;
 oc_parking_localization_transition_found=oc_parking_localization_piece_seen=false;
 oc_parking_localization_phase=PARKING_LOCALIZE_SEEK_FIRST_MARKER;
 oc_parking_localization_marker_frames=oc_parking_localization_wall_frames=0;
 oc_parking_localization_tof_sequence=0;currentSnapshot.sequence=0;
 layoutMode=0;oc_parking_exit_steering=-50;oc_parking_field_pose_initialized=true;
}
void frames(float x,float range,int count){
 currentPose={x,-1200,0};currentSnapshot.selected_raw_distance_mm=range;
 for(int i=0;i<count;++i){++currentSnapshot.sequence;processParkingEdgeLocalizationTof();}
}
bool check(bool eligible,bool badGeometry=false){
 reset();if(!eligible)layoutMode=1;
 frames(460,70,OBSTACLE_PARKING_EXIT_MARKER_CONFIRM_FRAMES);
 if(oc_parking_localization_phase!=PARKING_LOCALIZE_ON_FIRST_MARKER)return false;
 // First-edge maximum footprint is near480 with this fresh marker.
 frames(badGeometry?350:435,70,1);
 frames(420,expectedParkingOuterWallRange(TOF_RIGHT,currentPose),
        OBSTACLE_PARKING_EXIT_WALL_CONFIRM_FRAMES);
 if(eligible&&!badGeometry){
  if(!oc_parking_localization_first_edge_reference||!oc_parking_localization_transition_found)return false;
  const auto frozen=oc_parking_localization_last_piece_pose;
  const auto wall=oc_parking_localization_wall_range;
  const float correction=480-parkingBeamFootprint(TOF_RIGHT,70,frozen).maximumX;
  const float finalX=420-oc_parking_localization_first_edge_continue_mm+correction;
  if(fabsf(finalX-OBSTACLE_PARKING_CCW_FIRST_EDGE_ENTRY_X_MM)>.01f)return false;
  frames(200,65,4);
  return oc_parking_localization_last_piece_pose.x_mm==frozen.x_mm &&
         oc_parking_localization_wall_range==wall;
 }
 if(oc_parking_localization_first_edge_reference||oc_parking_localization_transition_found||
    oc_parking_localization_phase!=PARKING_LOCALIZE_BETWEEN_MARKERS)return false;
 // Ambiguous first edge / O3 must still acquire the second marker normally.
 frames(180,70,OBSTACLE_PARKING_EXIT_MARKER_CONFIRM_FRAMES);
 frames(150,expectedParkingOuterWallRange(TOF_RIGHT,currentPose),
        OBSTACLE_PARKING_EXIT_WALL_CONFIRM_FRAMES);
 return oc_parking_localization_transition_found&&!oc_parking_localization_first_edge_reference;
}
int main(){
 if(!check(true)||!check(false)||!check(true,true))return 1;
 reset();frames(460,70,1);const auto before=oc_parking_localization_marker_frames;
 processParkingEdgeLocalizationTof();if(before!=oc_parking_localization_marker_frames)return 2;
 std::cout<<"FIRST EDGE selection, target, frozen reference, O3/geometry fallback, freshness PASS\n";
}
'''
    destination=ROOT/'local_workspace/ccw-first-edge'
    destination.mkdir(parents=True,exist_ok=True)
    cpp=destination/'check.cpp';cpp.write_text(fixture)
    compiler=shutil.which('g++') or shutil.which('clang++')
    if not compiler:raise RuntimeError('Host C++ compiler on PATH required')
    exe=destination/'check.exe'
    subprocess.run([compiler,'-std=c++17','-O2','-I',str(ROOT/'include'),str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)


if __name__=='__main__':main()
