"""
Working Simulation Model for Vision-Based Outdoor UGV Navigation.

Closed-loop autonomous navigation simulator:
1. Simulates 4WD Differential/Skid-Steer UGV rover physics and kinematics.
2. Synthesizes onboard camera frames from the rover's 3D pose looking ahead.
3. Feeds synthetic camera frames into main.detect_path() and main.decide().
4. Uses the vision perception feedback to steer the rover along winding outdoor tracks.
5. Displays a real-time 4-panel dashboard and logs trajectory and telemetry to CSV & video.
"""
import argparse
import csv
import math
import os
import time
from datetime import datetime

import cv2
import numpy as np

# Import vision algorithms directly from main.py
import main


def build_sim_parser():
    p = argparse.ArgumentParser(description="Working closed-loop simulation model for Vision UGV")
    p.add_argument("--track", choices=["s_curve", "oval", "switchback"], default="s_curve",
                   help="Simulation track layout (default: s_curve)")
    p.add_argument("--speed", type=float, default=1.0, help="Forward cruise speed in m/s (default: 1.0)")
    p.add_argument("--kp", type=float, default=1.8, help="Steering proportional gain (default: 1.8)")
    p.add_argument("--duration", type=float, default=25.0, help="Simulation duration in seconds (default: 25.0)")
    p.add_argument("--output-dir", default="outputs", help="Directory for simulation logs and video")
    p.add_argument("--no-display", action="store_true", help="Run without opening GUI windows")
    p.add_argument("--save-video", action="store_true", default=True, help="Record simulation run to MP4")
    return p


class SimulatedEnvironment:
    """Simulates an outdoor world with a ground path, grass/dirt textures, and waypoint splines."""
    def __init__(self, track_type="s_curve"):
        self.world_size = 40.0  # 40 x 40 meters
        self.map_res = 20  # pixels per meter (800x800 pixel world map)
        self.track_type = track_type
        self.waypoints = self._generate_track(track_type)
        self.path_width = 1.8  # meters

    def _generate_track(self, track_type):
        pts = []
        if track_type == "s_curve":
            for s in np.linspace(0, 30, 300):
                x = s
                y = 5.0 * math.sin(s * 0.22)
                pts.append((x, y))
        elif track_type == "oval":
            a, b = 14.0, 7.0
            for theta in np.linspace(0, 2 * math.pi, 300):
                x = a * math.cos(theta) + 16.0
                y = b * math.sin(theta)
                pts.append((x, y))
        else:  # switchback
            for s in np.linspace(0, 30, 300):
                x = s
                y = 4.0 * math.sin(s * 0.35) + 2.0 * math.cos(s * 0.18)
                pts.append((x, y))
        return np.array(pts)

    def render_overhead_map(self, rover_pose, trail):
        """Render top-down tactical overview map."""
        img_size = int(self.world_size * self.map_res)
        overhead = np.full((img_size, img_size, 3), (35, 75, 40), dtype=np.uint8)  # Grass green

        # Offset to center
        origin_x = 80
        origin_y = img_size // 2

        def world_to_map(wx, wy):
            mx = int(origin_x + wx * self.map_res)
            my = int(origin_y - wy * self.map_res)
            return mx, my

        # Draw path ribbon
        for i in range(len(self.waypoints) - 1):
            p1 = world_to_map(self.waypoints[i][0], self.waypoints[i][1])
            p2 = world_to_map(self.waypoints[i + 1][0], self.waypoints[i + 1][1])
            thickness = int(self.path_width * self.map_res)
            cv2.line(overhead, p1, p2, (80, 100, 115), thickness)  # Dirt/asphalt gray

        # Centerline dashes
        for i in range(0, len(self.waypoints) - 1, 4):
            p1 = world_to_map(self.waypoints[i][0], self.waypoints[i][1])
            p2 = world_to_map(self.waypoints[i + 1][0], self.waypoints[i + 1][1])
            cv2.line(overhead, p1, p2, (40, 210, 250), 2)  # Yellow dashes

        # Draw rover trajectory trail
        if len(trail) > 1:
            trail_pts = np.array([world_to_map(tx, ty) for tx, ty in trail], dtype=np.int32)
            cv2.polylines(overhead, [trail_pts], False, (255, 120, 0), 2, cv2.LINE_AA)

        # Draw UGV body
        rx, ry, theta = rover_pose
        rmx, rmy = world_to_map(rx, ry)
        rover_len_px = int(0.7 * self.map_res)
        rover_wid_px = int(0.5 * self.map_res)

        rect = ((rmx, rmy), (rover_len_px, rover_wid_px), -math.degrees(theta))
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        cv2.drawContours(overhead, [box], 0, (230, 230, 230), -1)  # Chassis
        cv2.drawContours(overhead, [box], 0, (30, 30, 30), 2)

        # Heading arrow
        arrow_len = int(1.2 * self.map_res)
        hx = int(rmx + arrow_len * math.cos(theta))
        hy = int(rmy - arrow_len * math.sin(theta))
        cv2.arrowedLine(overhead, (rmx, rmy), (hx, hy), (0, 0, 255), 2, tipLength=0.35)

        return overhead

    def render_synthetic_camera(self, rover_pose, cam_w=640, cam_h=480):
        """
        Synthesize realistic onboard camera frame looking forward from UGV rover.
        Generates ground plane, horizon, traversable path perspective, and textures.
        """
        rx, ry, theta = rover_pose

        frame = np.zeros((cam_h, cam_w, 3), dtype=np.uint8)
        horizon_y = int(cam_h * 0.38)

        # Sky gradient (top region, filtered out by ROI)
        for y in range(horizon_y):
            ratio = y / max(horizon_y, 1)
            b = int(220 + 20 * ratio)
            g = int(170 + 30 * ratio)
            r = int(120 + 40 * ratio)
            frame[y, :] = (b, g, r)

        # Ground base texture (grass / soil)
        ground_h = cam_h - horizon_y
        for y in range(ground_h):
            ratio = y / ground_h
            b = int(25 + 20 * ratio)
            g = int(60 + 25 * ratio)
            r = int(30 + 15 * ratio)
            frame[horizon_y + y, :] = (b, g, r)

        # Add subtle ground noise/texture
        noise = np.random.randint(-10, 10, (cam_h - horizon_y, cam_w, 3), dtype=np.int16)
        ground_region = frame[horizon_y:, :].astype(np.int16) + noise
        frame[horizon_y:, :] = np.clip(ground_region, 0, 255).astype(np.uint8)

        # Perspective projection of the path
        # Look ahead from 1.0m to 14.0m
        cam_height = 0.385
        cam_pitch = 0.314  # ~18 deg downward
        focal_length = cam_w / (2 * math.tan(math.radians(62.2 / 2)))

        path_points_left = []
        path_points_right = []

        distances = np.linspace(1.2, 14.0, 50)
        for dist in distances:
            # Query track waypoint at rover forward distance
            eval_x = rx + dist * math.cos(theta)
            eval_y = ry + dist * math.sin(theta)

            # Find closest track point and track normal
            diffs = self.waypoints - np.array([eval_x, eval_y])
            dist_sq = np.sum(diffs ** 2, axis=1)
            idx = np.argmin(dist_sq)
            nearest = self.waypoints[idx]

            # Vector from evaluated forward point to track center
            lateral_offset = (nearest[0] - rx) * (-math.sin(theta)) + (nearest[1] - ry) * (math.cos(theta))

            # Transform into camera frame (X forward, Y left, Z up)
            # Camera coordinates:
            # X_cam = lateral_offset
            # Y_cam = cam_height
            # Z_cam = dist
            # Pitch rotation:
            z_eff = dist * math.cos(cam_pitch) + cam_height * math.sin(cam_pitch)
            y_eff = -dist * math.sin(cam_pitch) + cam_height * math.cos(cam_pitch)

            if z_eff <= 0.2:
                continue

            # Screen projection
            py = int(cam_h / 2 + (y_eff / z_eff) * focal_length)
            px_center = int(cam_w / 2 - (lateral_offset / z_eff) * focal_length)
            half_w_px = int(((self.path_width / 2.0) / z_eff) * focal_length)

            if horizon_y <= py < cam_h:
                path_points_left.append((px_center - half_w_px, py))
                path_points_right.append((px_center + half_w_px, py))

        if len(path_points_left) >= 2:
            path_polygon = np.array(path_points_left + path_points_right[::-1], dtype=np.int32)
            # Fill traversable path region with candidate dirt/asphalt color
            cv2.fillPoly(frame, [path_polygon], (85, 95, 110))

            # Path edges
            cv2.polylines(frame, [np.array(path_points_left, dtype=np.int32)], False, (60, 70, 80), 2)
            cv2.polylines(frame, [np.array(path_points_right, dtype=np.int32)], False, (60, 70, 80), 2)

            # Center yellow dashes
            center_pts = []
            for (lx, ly), (rx_p, ry_p) in zip(path_points_left, path_points_right):
                center_pts.append(((lx + rx_p) // 2, ly))

            for i in range(0, len(center_pts) - 1, 3):
                p1 = center_pts[i]
                p2 = center_pts[min(i + 1, len(center_pts) - 1)]
                cv2.line(frame, p1, p2, (35, 185, 235), 2)

        return frame


class UGVKinematics:
    """4WD Differential / Skid-steer robot kinematic model."""
    def __init__(self, x=0.0, y=0.0, theta=0.0, wheelbase=0.36, track_width=0.44):
        self.x = x
        self.y = y
        self.theta = theta
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.v = 0.0
        self.omega = 0.0

    def update(self, target_v, target_omega, dt):
        """Update pose with velocity limits and acceleration filtering."""
        # Simple velocity ramp
        self.v += np.clip(target_v - self.v, -1.5 * dt, 1.5 * dt)
        self.omega += np.clip(target_omega - self.omega, -3.0 * dt, 3.0 * dt)

        # Unicycle / Skid-steer forward kinematics
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt

        # Individual wheel speeds
        v_left = self.v - (self.omega * self.track_width / 2.0)
        v_right = self.v + (self.omega * self.track_width / 2.0)
        return self.x, self.y, self.theta, v_left, v_right


def compose_dashboard(overhead_map, cam_annotated, mask_bgr, telemetry):
    """Combine overhead map, vision camera, mask, and telemetry HUD into a single dashboard window."""
    dash_h = 720
    dash_w = 1280
    dashboard = np.full((dash_h, dash_w, 3), (20, 24, 30), dtype=np.uint8)

    # 1. Left side: Tactical Overhead Map (scaled to 600x680)
    map_h, map_w = overhead_map.shape[:2]
    crop_size = min(map_h, map_w)
    map_cropped = overhead_map[0:crop_size, 0:crop_size]
    map_resized = cv2.resize(map_cropped, (560, 560))
    dashboard[70:630, 20:580] = map_resized

    cv2.rectangle(dashboard, (20, 70), (580, 630), (70, 80, 95), 2)
    cv2.putText(dashboard, "TACTICAL MAP & UGV TRAJECTORY", (24, 52),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    # 2. Right Top: Onboard Camera & Path Detection
    cam_resized = cv2.resize(cam_annotated, (380, 270))
    dashboard[70:340, 610:990] = cam_resized
    cv2.rectangle(dashboard, (610, 70), (990, 340), (70, 80, 95), 2)
    cv2.putText(dashboard, "SYNTHETIC ONBOARD CAMERA (PERCEPTION)", (614, 52),
                cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)

    # 3. Right Mid-Top: Vision Segmentation Mask
    mask_resized = cv2.resize(mask_bgr, (250, 180))
    dashboard[70:250, 1010:1260] = mask_resized
    cv2.rectangle(dashboard, (1010, 70), (1260, 250), (70, 80, 95), 2)
    cv2.putText(dashboard, "HSV CANDIDATE MASK", (1014, 52),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 2)

    # 4. Right Bottom: Telemetry, State Machine, and Steering Gauge
    hud_x, hud_y = 610, 365
    cv2.rectangle(dashboard, (hud_x, hud_y), (1260, 690), (30, 38, 48), -1)
    cv2.rectangle(dashboard, (hud_x, hud_y), (1260, 690), (70, 80, 95), 2)

    cv2.putText(dashboard, "CLOSED-LOOP AUTONOMOUS TELEMETRY", (hud_x + 14, hud_y + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (56, 189, 248), 2)

    lines = [
        f"Navigation State: {telemetry['status']}",
        f"Vision Decision:  {telemetry['decision']}",
        f"Steering Error:   {telemetry['error_str']}",
        f"Forward Speed:    {telemetry['speed']:.2f} m/s",
        f"Turn Rate:        {telemetry['omega']:.2f} rad/s",
        f"Wheel Speeds:     L={telemetry['v_left']:.2f} m/s  |  R={telemetry['v_right']:.2f} m/s",
        f"Vision FPS:       {telemetry['fps']:.1f}",
        f"Sim Time:         {telemetry['time']:.1f} s / {telemetry['duration']:.1f} s"
    ]

    for i, line in enumerate(lines):
        color = (255, 255, 255)
        if "Decision" in line:
            color = (0, 255, 0) if "STRAIGHT" in line else ((0, 200, 255) if "TURN" in line else (0, 0, 255))
        elif "State" in line:
            color = (56, 189, 248)
        cv2.putText(dashboard, line, (hud_x + 16, hud_y + 70 + i * 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.56, color, 1, cv2.LINE_AA)

    # Steering Gauge Bar
    bar_x = hud_x + 16
    bar_y = hud_y + 295
    bar_w = 400
    bar_h = 16
    cv2.rectangle(dashboard, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (60, 60, 60), -1)
    center_bar_x = bar_x + bar_w // 2
    cv2.line(dashboard, (center_bar_x, bar_y - 4), (center_bar_x, bar_y + bar_h + 4), (255, 255, 255), 2)

    # Deadband indicator
    deadband_px = int(0.08 * (bar_w // 2))
    cv2.rectangle(dashboard, (center_bar_x - deadband_px, bar_y),
                  (center_bar_x + deadband_px, bar_y + bar_h), (80, 140, 80), -1)

    # Indicator pin
    err_val = telemetry.get("error_val", 0.0)
    pin_x = int(center_bar_x + np.clip(err_val, -1.0, 1.0) * (bar_w // 2))
    cv2.circle(dashboard, (pin_x, bar_y + bar_h // 2), 9, (0, 0, 255), -1)
    cv2.putText(dashboard, "LEFT", (bar_x - 38, bar_y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    cv2.putText(dashboard, "RIGHT", (bar_x + bar_w + 8, bar_y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    return dashboard


def run_simulation():
    args = build_sim_parser().parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.output_dir, f"simulation_log_{stamp}.csv")
    video_path = os.path.join(args.output_dir, f"simulation_run_{stamp}.mp4")

    env = SimulatedEnvironment(track_type=args.track)
    ugv = UGVKinematics(x=0.0, y=0.0, theta=0.0)

    # Candidate ground HSV filter parameters matching main.py
    hsv_lower = [0, 0, 35]
    hsv_upper = [180, 100, 230]
    min_area = 1500
    deadband = 0.08

    dt = 0.05  # 20 Hz simulation loop
    total_steps = int(args.duration / dt)
    trail = []

    writer = None
    if args.save_video:
        writer = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*"mp4v"), 20.0, (1280, 720))

    rows = []
    print("\n" + "=" * 65)
    print("   Vision-Based UGV — Closed-Loop Autonomous Working Model")
    print("=" * 65)
    print(f"Track: {args.track}  |  Speed: {args.speed} m/s  |  Duration: {args.duration} s")
    print("Perception: main.detect_path() + main.decide() -> Skid-Steer Kinematics")
    print(f"Logs: {csv_path}")
    print("Press 'q' in simulation window to stop early.\n")

    start_time = time.time()
    err_integral = 0.0
    last_err = 0.0

    try:
        for step in range(total_steps):
            sim_time = step * dt
            trail.append((ugv.x, ugv.y))

            # 1. Synthesize camera view from robot's 3D pose
            raw_cam_frame = env.render_synthetic_camera((ugv.x, ugv.y, ugv.theta))

            # 2. RUN REAL COMPUTER VISION PIPELINE FROM main.py
            result, mask, roi_y = main.detect_path(raw_cam_frame, hsv_lower, hsv_upper, min_area)
            h, w = raw_cam_frame.shape[:2]

            cam_annotated = raw_cam_frame.copy()
            cv2.line(cam_annotated, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
            cv2.line(cam_annotated, (0, roi_y), (w, roi_y), (255, 255, 0), 1)

            if result is None:
                action, error_float, status = "STOP / PATH UNCERTAIN", 0.0, "PATH_NOT_DETECTED"
                target_v = 0.0
                target_omega = 0.0
                cv2.putText(cam_annotated, action, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                contour_global = result["contour"].copy()
                contour_global[:, 0, 1] += roi_y
                cv2.drawContours(cam_annotated, [contour_global], -1, (0, 255, 0), 2)
                cx, cy = result["center"]
                action, error_float = main.decide(cx, w, deadband)
                status = "AUTONOMOUS_NAVIGATING"

                # Closed-Loop PD steering controller
                err_deriv = (error_float - last_err) / dt
                last_err = error_float

                # Steer toward path centroid
                target_omega = -args.kp * error_float - 0.2 * err_deriv
                target_v = args.speed * max(0.3, 1.0 - 0.6 * abs(error_float))  # Slow down on sharp turns

                cv2.circle(cam_annotated, (cx, cy), 7, (0, 0, 255), -1)
                cv2.line(cam_annotated, (w // 2, h), (cx, cy), (0, 255, 255), 2)
                cv2.putText(cam_annotated, action, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # 3. Update UGV Physics Kinematics
            rx, ry, rtheta, vl, vr = ugv.update(target_v, target_omega, dt)

            # 4. Render Tactical Overhead Map
            overhead = env.render_overhead_map((rx, ry, rtheta), trail)

            # 5. Build Combined Dashboard
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            fps = (step + 1) / max(time.time() - start_time, 1e-5)

            telemetry = {
                "status": status,
                "decision": action,
                "error_str": f"{error_float:+.3f}" if result else "N/A",
                "error_val": error_float,
                "speed": ugv.v,
                "omega": ugv.omega,
                "v_left": vl,
                "v_right": vr,
                "fps": fps,
                "time": sim_time,
                "duration": args.duration
            }

            dashboard = compose_dashboard(overhead, cam_annotated, mask_bgr, telemetry)

            if writer:
                writer.write(dashboard)

            rows.append({
                "sim_time": round(sim_time, 3),
                "ugv_x": round(rx, 3),
                "ugv_y": round(ry, 3),
                "ugv_theta_deg": round(math.degrees(rtheta), 2),
                "status": status,
                "decision": action,
                "normalized_center_error": round(error_float, 4) if result else "",
                "v_linear": round(ugv.v, 3),
                "omega_angular": round(ugv.omega, 3),
                "v_wheel_left": round(vl, 3),
                "v_wheel_right": round(vr, 3),
            })

            if not args.no_display:
                cv2.imshow("Vision UGV — Closed-Loop Working Model Simulator", dashboard)
                step_ms = max(1, int(dt * 1000))
                if cv2.waitKey(step_ms) & 0xFF == ord('q'):
                    print("\nUser requested early stop.")
                    break

        if not args.no_display:
            cv2.putText(dashboard, "SIMULATION COMPLETE - Press 'q' to close",
                        (620, 345), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (34, 197, 94), 2)
            cv2.imshow("Vision UGV — Closed-Loop Working Model Simulator", dashboard)
            cv2.waitKey(0)

    finally:
        if writer:
            writer.release()
        if not args.no_display:
            cv2.destroyAllWindows()

    # Save CSV log
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        fieldnames = ["sim_time", "ugv_x", "ugv_y", "ugv_theta_deg", "status",
                      "decision", "normalized_center_error", "v_linear",
                      "omega_angular", "v_wheel_left", "v_wheel_right"]
        w_csv = csv.DictWriter(f, fieldnames=fieldnames)
        w_csv.writeheader()
        w_csv.writerows(rows)

    total_frames = len(rows)
    nav_frames = sum(1 for r in rows if r["status"] == "AUTONOMOUS_NAVIGATING")
    success_rate = (nav_frames / total_frames * 100) if total_frames else 0.0

    print("\nSimulation Complete!")
    print(f"Total steps simulated: {total_frames}")
    print(f"Autonomous tracking success: {nav_frames}/{total_frames} ({success_rate:.1f}%)")
    print(f"Telemetry Log CSV:  {csv_path}")
    if args.save_video:
        print(f"Recorded Video MP4: {video_path}")


if __name__ == "__main__":
    run_simulation()
