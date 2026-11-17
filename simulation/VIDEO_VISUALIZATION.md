# Video visualization tools

Run these commands from the repository root with Python 3.10 or newer.
Install the Python dependencies in your chosen environment.

```powershell
python -m pip install numpy scipy Pillow opencv-python matplotlib
```

Text rendering uses installed Arial or DejaVu Sans fonts. Video exports use
OpenCV's MPEG-4 encoder. Generated media, tracking caches and manifests are
written under ignored `local_workspace/`.

## Archived-log replays

The default inputs are the complete tracked originals in
`simulation/evidence/parking_exit_diagnostics/`. The supported documentation
runs are 601, 602, 632, 633 and 634. Run 601 contains an obstacle session despite
its original OPEN filename.

```powershell
python simulation/render_video_runs.py --runs 601 632 633 634
python simulation/render_video_runs.py --continuous --runs 601 632 633 634
python simulation/render_open_video_602.py
python simulation/render_sensor_usage.py
```

The first command preserves pose-coverage gaps. The second fills them with
interpolation for presentation. Run 602's Open sequences have untimed corner
records, so its schematic uses illustrative positions and pacing. The sensor
overlay command consumes the continuous exports and the 602 schematic.

`render_video_runs.py --source DIRECTORY` also accepts working copies with
`log_633.txt`-style names. Each input must match an archived original by SHA-256.

## Latest run 633 map

This pipeline uses the archived CCW log and the original recordings
`DJI_20261031175628_0066_D.MP4` and `DJI_20261031180153_0067_D.MP4`.
The video files are supplied separately.

The preprocessing inputs are the saved camera-transform JSON files from the
one-time stabilization analysis. Place those files at these paths.

```text
local_workspace/video-documentation/stabilized-dji-batch-0066-0068/
  DJI_20261031175628_0066_D/camera_transforms.json
  DJI_20261031180153_0067_D/camera_transforms.json
```

Each JSON contains the source SHA-256, dimensions, frame count, frame rate,
960-pixel working size and frame-indexed homography matrices. The original
stabilization utility and its analysis remain local working material. A fresh
checkout needs these preprocessing inputs alongside the original footage.

Prepare the tracking cache by supplying the folder containing both originals.

```powershell
$videoRoot = Read-Host 'Folder containing the original DJI videos'
python simulation/prepare_run_633_drone.py --video-root $videoRoot
python simulation/refine_run_633_tracking.py
python simulation/run_633_video_alignment.py
python simulation/calibrate_run_633_clock.py
python simulation/render_run_633_map.py --preview
python simulation/render_run_633_map.py
```

`prepare_run_633_drone.py --reuse-cache` rebuilds tracking from cached frames.
`refine_run_633_tracking.py --heading-only` updates the chassis headings from
the refined position track. Both options require their earlier cache stages.

The final export is
`local_workspace/video-documentation/sensor-usage/run-633-drone/map-only/run_633_map_video_synced_large_text_1x.mp4`.
It is a silent 1080-square, 30 fps clip with 42-pixel distance labels. The
manifest and README beside it record source identity, sensor events, timing
calibration and uncertainty.

The clock calibration matches the initial unparking turn, three corresponding
bottom course crossings and the final stop. It applies one uniform conversion
to logged poses and sensor events. The reviewed fit maps one logged second to
about 1.03447 filmed seconds. Source DJI0066 time 4:39.788 aligns with the first
simulation frame. The resulting playback lasts about 196.9 seconds.

Pose gaps follow the tracked footage and meet their surrounding logged anchors.
The final reverse uses its logged 53.2 mm distance and final pose, with estimated
timing inside the missing tail. The top green pillar is drawn at the filmed
outer-middle seat, while the manifest retains the robot's original assignment.
Distance labels show geometric intersections with modeled surfaces. Closing
rear-reference corrections have untimed log evidence and estimated display
windows.

`render_run_633_drone.py` retains the earlier panel and field export workflow.
`render_run_633_map.py` produces the latest map revision.

## Open Challenge renderers

For the archived log 602 schematic and range-ray versions, run these commands.

```powershell
python simulation/render_open_tof_rays.py
python simulation/render_tof_positioned_open.py
```

For the reviewed DJI Open recording, provide
`DJI_20261031112923_0061_D_OPEN_video1.MP4` through the video argument. The tracker
checks its reviewed source hash and writes the track under ignored working
storage.

```powershell
$openVideo = Read-Host 'Path to the original DJI Open recording'
python simulation/track_dji_open.py --video $openVideo
python simulation/render_dji_open.py --video $openVideo --track local_workspace/open-dji-sync/track.json
```

The historical oblique-video renderer accepts an explicit reviewed trajectory
JSON through `--track` and source footage through `--video`.

```powershell
python simulation/render_video_aligned_open.py --help
```

These older Open reconstructions retain their individual geometry and timing
assumptions in their generated manifests. Source video, manually reviewed track
inputs and derived media are supplied or regenerated outside Git.
