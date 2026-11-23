# 3D Simulation Robot Model — Vision UGV Rover

This folder contains the complete **3D Robot Simulation Model** (URDF, Xacro, 3D Wavefront OBJ/MTL, and interactive 3D WebGL viewer) designed for the **Vision-Based Outdoor Navigation for Unmanned Ground Vehicles (UGV)** project.

---

## 1. Quick Start: Interactive 3D Viewer

You can launch and view the 3D model, rotate, zoom, inspect coordinate frames, and drive the rover around with live onboard camera streaming:

```bash
python robot_model/view_model.py
```

Or simply double-click or open `robot_model/ugv_3d_viewer.html` directly in any web browser (Chrome, Edge, Firefox).

### Features in the 3D Working Model:
- **Autonomous Vision Navigation Engine**: Toggle ON to watch the 3D UGV Rover autonomously steer itself along winding tracks and circuits using closed-loop visual path tracking and skid-steer kinematics!
- **Circuit Track Selection**: Switch between an endless **Circuit Loop** and an **S-Curve Trail**.
- **Interactive Teleop Drive**: Toggle OFF autonomous mode to drive manually using `W`, `A`, `S`, `D` or arrow keys.
- **Picture-in-Picture (PIP) Onboard Camera**: See the live perspective from the rover's camera lens.
- **Vision Mask View Toggle**: Switch the camera PIP between RGB live view and the computer vision traversable segmentation mask.
- **42% ROI Line & Guidance Vector**: Visually inspect the ground cutoff line from `main.py` (`roi_y = int(h * 0.42)`), the detected centroid crosshair, and heading vector.
- **Live Telemetry HUD**: Real-time display of navigation state, qualitative decision (`TURN LEFT`, `STRAIGHT`, `TURN RIGHT`), normalized center error gauge, forward velocity, angular turn rate, and individual wheel speeds ($v_L, v_R$).
- **Coordinate Frames (TF Tree)**: Toggle standard ROS coordinate frames (`base_footprint`, `base_link`, `camera_link`, wheel joints).

---

## 2. Model Files

| File | Description | Target Environment |
|------|-------------|--------------------|
| `ugv_rover.urdf` | Complete Unified Robot Description Format (URDF) file with kinematics, collision, inertia tensors, and Gazebo plugins | ROS 1, ROS 2, Gazebo, Webots, PyBullet |
| `ugv_rover.xacro` | Parametric XML Macro file for configurable dimensions, masses, camera height, and tilt angle | ROS / ROS 2 Xacro toolchain |
| `ugv_rover.obj` | 3D Wavefront mesh file (672 vertices, 1,248 triangles) with textured parts | Blender, AutoCAD, Proteus, Unity, Unreal |
| `ugv_rover.mtl` | Material definition file with 11 PBR surface properties | Wavefront OBJ renderers |
| `generate_obj.py`| Python script to procedurally regenerate `ugv_rover.obj` and `ugv_rover.mtl` | Python standalone |
| `ugv_3d_viewer.html` | High-fidelity interactive WebGL/Three.js simulation & perception visualizer | Web Browsers |
| `view_model.py` | One-command launcher script with local web server | Python CLI |

---

## 3. Physical Specifications

| Parameter | Specification | Value | Notes |
|-----------|---------------|-------|-------|
| **Drive Architecture** | 4WD Skid-Steer / Diff Drive | 4 driven wheels | High traction on dirt, gravel, and grass |
| **Total Length** | Outer Bumper-to-Bumper | 540 mm (0.54 m) | Compact outdoor exploration class |
| **Chassis Width** | Hull Width | 340 mm (0.34 m) | |
| **Total Track Width** | Wheel Center-to-Center | 440 mm (0.44 m) | Broad stance for slope stability |
| **Wheelbase** | Front Axle to Rear Axle | 360 mm (0.36 m) | |
| **Wheel Diameter / Width** | Rugged Off-road Tires | Ø 250 mm x 80 mm | Deep rubber treads with orange rims |
| **Ground Clearance** | Base Link to Ground | 90 mm (0.09 m) | High clearance for outdoor obstacles |
| **Chassis Mass** | Aluminum chassis + battery | 12.0 kg | Low center of mass (Z = 0.165 m) |
| **Gross Vehicle Weight**| Total dry mass | 19.2 kg | Payload capacity: up to 15 kg |

---

## 4. Vision Camera & Sensor Integration (Aligned with `main.py`)

The camera mount and optical orientation strictly align with the computer vision algorithms in `main.py`:

```
          [ Camera Link ]  (Pitch: -18.0° downward)
                 \
                  \  Camera FOV Frustum (62.2° H-FOV)
                   \
====================\======================= Ground Plane
     Rover Body     | 42% ROI Cutoff Line
                    | (Drivable Ground Path Candidates)
```

- **Mount Height**: 385 mm above the ground (`base_footprint`), positioned 170 mm forward from the chassis center.
- **Downward Pitch**: 18.0° ($0.314\text{ rad}$) — optimized to suppress sky glare while extending forward look-ahead distance.
- **Resolution**: $640 \times 480\text{ px}$ at 30 FPS.
- **Horizontal FOV**: $62.2^\circ$ ($1.085\text{ rad}$), Vertical FOV: $48^\circ$.
- **ROI Ground Cutoff**: Corresponding to `roi_y = int(h * 0.42)` in `main.py`, the path detection begins approximately 0.95 meters in front of the vehicle.

---

## 5. Coordinate Frames (ROS REP-103 & REP-105)

The URDF implements the standard coordinate frames:

1. **`base_footprint`**: 2D ground projection directly below the robot center ($Z = 0.0\text{ m}$).
2. **`base_link`**: Center of the chassis hull ($X\text{ forward (red)}, Y\text{ left (green)}, Z\text{ up (blue)}$).
3. **`front_left_wheel_link`**, **`front_right_wheel_link`**, **`rear_left_wheel_link`**, **`rear_right_wheel_link`**: Continuous rotation joints around the $Y\text{-axis}$.
4. **`sensor_mast_link`**: Rigidly mounted mast for sensor elevation.
5. **`camera_link`**: Body-fixed camera reference frame.
6. **`camera_optical_link`**: Standard ROS camera optical frame ($Z\text{ forward}, X\text{ right}, Y\text{ down}$).

---

## 6. Simulation Integration

### A. Gazebo Simulation
The URDF includes pre-configured Gazebo plugins:
- **Differential / Skid-Steer Controller**: `libgazebo_ros_skid_steer_drive.so`
  - Subscribes to: `/ugv/cmd_vel` (`geometry_msgs/Twist`)
  - Publishes: Odometry and transform to `/tf`
- **Onboard Vision Camera Plugin**: `libgazebo_ros_camera.so`
  - Publishes: `/ugv/front_camera/image_raw` (`sensor_msgs/Image`)
  - Camera info: `/ugv/front_camera/camera_info`

To test directly in ROS / Gazebo:
```bash
rosrun gazebo_ros spawn_model -urdf -file robot_model/ugv_rover.urdf -model vision_ugv_rover
```

### B. Webots / PyBullet
- In **Webots**: Import `ugv_rover.urdf` using `File -> Import -> Import URDF or PROTO`.
- In **PyBullet**:
  ```python
  import pybullet as p
  physicsClient = p.connect(p.GUI)
  rover_id = p.loadURDF("robot_model/ugv_rover.urdf", [0, 0, 0.165])
  ```
