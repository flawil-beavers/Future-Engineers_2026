# Whole-run images with pillars

Install the offline plotting dependency with `python -m pip install matplotlib`.
Run from the repository root:

```powershell
python simulation/visualize_robot_run.py simulation/evidence/parking_exit_diagnostics/20261006_log_476_ccw.txt simulation/evidence/parking_exit_diagnostics/20261006_log_481_ccw.txt
```

Each diagnostic session gets one `_whole_run.svg` and `_whole_run.png` under
ignored `local_workspace/parking-exit-analysis-all/`. Use `--output-dir` to choose
another working directory outside the source-log directory. Multi-session logs
are split using the existing diagnostic parser; each image uses only its session.
Original logs are never changed.

For the whole archived batch, including links from the batch report:

```powershell
python simulation/analyze_parking_exit_batch.py --whole-run-images
```

The flag is optional so the standard-library batch workflow remains available
without Matplotlib. Existing exit/ToF outputs and robot behavior are unchanged.
The program and instructions belong in Git; generated images stay ignored.

Pillars are red/green square symbols labeled with their recorded seat index.
Accepted map confirmations, live/later avoidance records and explicitly stored
start pillars supply occupancy and color. Rejected camera projections and CLEAR
records do not create pillars. Unobserved seats are omitted; the recorded map
can be incomplete or incorrect. Coordinates follow `initializeSeats()` in
`src/obstacle_path.cpp`: 500 mm station spacing, centerline at 1000 mm,
100 mm lateral offset, section rotation according to CW/CCW. Symbols are not
scaled obstacle footprints. Historical geometry differing from this model needs
a separately versioned geometry profile before using the image quantitatively.

Solid traces show logged rear-axle estimates. Sparse lap-1 pose records are dots,
not a fabricated complete route. Later tracking samples are joined for viewing,
but may miss motion and include estimator corrections. Parking corrections are
drawn separately; the scan link connects endpoints only. Accepted connector
plans remain dashed and are not claimed as traveled paths. A completed lap is
reported only from explicit completion text, not inferred from a drawn loop.
Physical outcomes remain in the evidence README. Each image includes source
SHA-256 for provenance; no clearance/contact conclusion follows from this plot.

Validation:

```powershell
python scripts/test-robot-run-visualization.py
```
