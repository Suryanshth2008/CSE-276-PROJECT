# Vision-Based Outdoor Navigation for Unmanned Ground Vehicles (UGV)

A complete, multi-module vision-based autonomous ground navigation system and web mission control portal for Unmanned Ground Vehicles (UGV).

### 🚀 Quick Start — Web Dashboard Studio
Launch the complete web application with all 6 interactive inspection modules:
```bash
python serve_website.py
# Or double-click start_website.bat
```
Visit **`http://localhost:8080`** in any web browser.

---

## What this prototype does
- Reads video files or a webcam.
- Applies simple HSV color thresholding to the lower portion of each frame.
- Finds the largest qualifying candidate region.
- Estimates its center and suggests LEFT / STRAIGHT / RIGHT.
- Reports `STOP / PATH UNCERTAIN` when no candidate region is detected.
- Saves frame-by-frame decisions and a processed video.

## Important limitations
This is only a **baseline experiment**, not a complete autonomous vehicle. HSV thresholding does not understand terrain semantically and may fail under changing lighting, shadows, mud, gravel, grass, or similar colors. It does not yet implement robust obstacle avoidance, GPS-denied localization, visual odometry, or physical robot control. Do not connect its steering suggestions directly to motors. A real robot needs independent emergency-stop and safety systems.

## 1. Setup (Windows)
Install Python 3.10 or 3.11, then open a terminal in this folder:

```bash
python -m venv venv
venv\Scripts\activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

## 2. Run with a video
Put an outdoor video in this folder and run:

```bash
python main.py --source "outdoor_video.mp4"
```

Or use the default webcam:

```bash
python main.py --source 0
```

Press `q` in the video window to stop. Results are saved under `outputs/`.

## 3. Tune the candidate-region threshold
The default HSV range is broad and may not match your scene. Use the displayed mask to see which pixels are being selected. Example for a brighter, low-saturation surface:

```bash
python main.py --source "outdoor_video.mp4" --hsv-lower 0 0 60 --hsv-upper 180 110 240
```

Example for green-dominant vegetation/ground (only suitable when that is what you want to test):

```bash
python main.py --source "outdoor_video.mp4" --hsv-lower 25 25 25 --hsv-upper 95 255 255
```

These are examples, not universal thresholds. The target traversable region must be defined for your dataset.

## 4. Headless processing
For a machine without a display:

```bash
python main.py --source "outdoor_video.mp4" --no-display
```

## 5. System Architecture (Perception & Navigation Engine)

The system is organized into modular Python components:

```
vision_ugv_project/
├── main.py                  # CLI pipeline, real-time HUD, and frame-level CSV logging
├── perception.py            # Multi-channel color segmentation, shadow compensation & waypoints
├── navigation.py            # State machine (INIT, PATH_FOUND, PATH_UNCERTAIN, STOP), error & decisions
├── evaluate_runs.py         # Benchmark suite (success rate, RMSE deviation, failure mode, plots)
├── generate_test_clips.py   # Test scenario generator (Sunny, Shadows/Gravel, Switchback)
├── simulate_ugv.py          # Closed-loop 4WD skid-steer autonomous simulator
├── outdoor_video.mp4        # Default realistic outdoor test video
├── test_clips/              # Multiple test scenarios for comparative evaluation
└── robot_model/             # 3D URDF, Xacro, OBJ CAD models and interactive WebGL viewer
```

### Perception Module (`perception.py`)
- **TraversablePathDetector**:
  - Lower ROI ground filtering (`roi_y = int(h * 0.42)`).
  - Multi-space color segmentation (HSV + LAB luminance compensation for outdoor shadows).
  - Bottom-anchored contour selection (ensures path connects directly in front of UGV wheels).
  - Lookahead waypoint & heading angle estimation.
  - Confidence scoring ($0.0$ to $1.0$) and failure categorization.

### Navigation Engine (`navigation.py`)
- **UGVNavigationEngine**:
  - Formal State Machine: `INIT`, `PATH_FOUND`, `PATH_UNCERTAIN`, `STOP_NO_PATH`.
  - Steering Decisions: `FORWARD`, `TURN LEFT`, `TURN RIGHT`, `STOP / PATH UNCERTAIN`.
  - Continuous normalized center error calculation with deadband filtering ($\pm 0.08$).
  - Reference path tracking & cross-track error measurement.

## 6. Run-Level Evaluation & Benchmark Suite

Evaluate any navigation test run or compare multiple runs:
```bash
python evaluate_runs.py
```
This computes:
- **Navigation Success Rate (%)**
- **Path Detection Consistency (%)**
- **Mean Absolute Path Deviation & RMSE**
- **Steering Smoothness / Jitter Index**
- **Processing Speed & Latency (FPS)**
- Generates publication-ready 4-panel visual performance plots under `outputs/evaluation_report_*.png`.

## 7. Generating & Testing Multiple Outdoor Scenarios

To generate 3 repeatable test clips with different outdoor challenges:
```bash
python generate_test_clips.py
```
Test across clips:
```bash
# 1. Sunny curved trail
python main.py --source test_clips/sunny_curved_trail.mp4

# 2. Shadowed gravel path (evaluates shadow compensation)
python main.py --source test_clips/shadowed_gravel_path.mp4

# 3. Sharp switchback (evaluates lookahead steering)
python main.py --source test_clips/sharp_switchback.mp4
```

## 8. Autonomous Closed-Loop Working Model

A complete **closed-loop autonomous simulation working model** is implemented in [`simulate_ugv.py`](simulate_ugv.py):
- Connects the 4WD skid-steer UGV kinematics directly with `main.py`'s computer vision functions (`detect_path` and `decide`).
- Generates synthetic camera frames from the robot's pose, detects traversable paths, computes center errors, and steers the UGV along winding outdoor paths.
- Records real-time multi-panel dashboards (Overhead map, Onboard camera, HSV mask, and Telemetry) to video and CSV logs.

To run the simulation model:
```bash
python simulate_ugv.py
```
Options:
- `--track s_curve`: Choose track layout (`s_curve`, `oval`, `switchback`).
- `--speed 1.2`: Cruise speed in m/s.
- `--duration 30`: Simulation time in seconds.
- `--no-display`: Run headless and save output to `outputs/`.

## 9. 3D Simulation Robot Model & Interactive Web Viewer
A full 3D simulation model (URDF, Xacro, 3D Wavefront OBJ/MTL, and interactive 3D WebGL viewer) is available in [`robot_model/`](robot_model/README.md).

To view and run the interactive 3D working model in your browser:
```bash
python robot_model/view_model.py
```
Or open [`robot_model/ugv_3d_viewer.html`](robot_model/ugv_3d_viewer.html) directly in any browser:
- **Autonomous Vision Mode**: The 3D rover autonomously navigates the circuit using the onboard camera's 42% ROI detection.
- **Manual Mode**: Drive using `W`, `A`, `S`, `D` or arrow keys.
- **Circuit Track Selection**: Circuit Loop or S-Curve Trail.
- **Live Onboard Camera (PIP)**: Real-time camera view with crosshairs, steering vector, and mask toggle.
- **URDF / Xacro**: Ready for ROS 1/2, Gazebo, Webots, and PyBullet.

## 10. Team split
- **Member 1 — Perception:** collect and label test clips; tune the baseline; compare it with a pretrained segmentation model; evaluate path-mask quality.
- **Member 2 — Navigation/evaluation:** implement safer waypoint selection, failure categories, repeatable test runs, metric summaries, and dashboard.
- **Both:** integration, testing, documentation, and honest reporting of failures.

## 11. Recommended next milestones
1. Test on at least 3 different outdoor clips.
2. Record which frames fail and why.
3. Define a reference path or annotations so evaluation is meaningful.
4. Add a semantic segmentation model for traversable terrain.
5. Add obstacle detection and conservative stop rules.
6. Investigate visual odometry for relative motion estimation.
7. If available, validate in a simulator or with a small rover under supervision.

## 12. Metrics already logged
- `status`: candidate path found or not detected.
- `decision`: qualitative direction suggestion or stop/uncertain.
- `normalized_center_error`: horizontal path-center offset in normalized image coordinates.
- `candidate_area_pixels`: detected candidate contour area.
- `processing_fps_average`: average processing rate.

The path-detection percentage printed at the end is **not** a navigation success rate. Report success rate, path deviation, and failures per run only after defining test-run completion and collecting the necessary reference data.
