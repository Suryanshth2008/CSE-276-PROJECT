"""
Synthetic Outdoor Test Clip Generator for Vision UGV Evaluation.

Generates 3 representative outdoor test clips matching README Milestone 1:
1. sunny_curved_trail.mp4    - Standard outdoor trail with winding curves.
2. shadowed_gravel_path.mp4  - Tree shadows and variable lighting across gravel.
3. sharp_switchback.mp4      - Rapid turns testing lookahead steering decisions.
"""
import math
import os
import cv2
import numpy as np


def generate_clip(filename, track_type="s_curve", add_shadows=False, total_frames=200):
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(filename, fourcc, 20.0, (640, 480))

    cam_w, cam_h = 640, 480
    horizon_y = int(cam_h * 0.38)
    focal_length = cam_w / (2 * math.tan(math.radians(62.2 / 2)))

    # Trajectory coordinates
    rx, ry, theta = 0.0, 0.0, 0.0
    path_width = 1.8

    for step in range(total_frames):
        frame = np.zeros((cam_h, cam_w, 3), dtype=np.uint8)

        # 1. Sky & Ground Gradients
        for y in range(horizon_y):
            ratio = y / max(horizon_y, 1)
            frame[y, :] = (int(215 + 25 * ratio), int(170 + 30 * ratio), int(115 + 40 * ratio))

        ground_h = cam_h - horizon_y
        for y in range(ground_h):
            ratio = y / ground_h
            frame[horizon_y + y, :] = (int(25 + 20 * ratio), int(60 + 25 * ratio), int(30 + 15 * ratio))

        # 2. Path Projection
        path_pts_l, path_pts_r = [], []
        distances = np.linspace(1.2, 14.0, 50)

        for dist in distances:
            eval_x = rx + dist * math.cos(theta)
            if track_type == "s_curve":
                true_y = 5.0 * math.sin(eval_x * 0.22)
            elif track_type == "switchback":
                true_y = 4.0 * math.sin(eval_x * 0.35) + 2.0 * math.cos(eval_x * 0.18)
            else:
                true_y = 3.0 * math.sin(eval_x * 0.15)

            lat_offset = (true_y - ry) * math.cos(theta) - (eval_x - rx) * math.sin(theta)
            z_eff = dist * math.cos(0.314) + 0.385 * math.sin(0.314)
            y_eff = -dist * math.sin(0.314) + 0.385 * math.cos(0.314)

            if z_eff > 0.2:
                py = int(cam_h / 2 + (y_eff / z_eff) * focal_length)
                px = int(cam_w / 2 - (lat_offset / z_eff) * focal_length)
                half_w = int(((path_width / 2.0) / z_eff) * focal_length)
                if horizon_y <= py < cam_h:
                    path_pts_l.append((px - half_w, py))
                    path_pts_r.append((px + half_w, py))

        if len(path_pts_l) >= 2:
            poly = np.array(path_pts_l + path_pts_r[::-1], dtype=np.int32)
            cv2.fillPoly(frame, [poly], (85, 95, 110))
            cv2.polylines(frame, [np.array(path_pts_l, dtype=np.int32)], False, (60, 70, 80), 2)
            cv2.polylines(frame, [np.array(path_pts_r, dtype=np.int32)], False, (60, 70, 80), 2)

            # Yellow dashed centerline
            for i in range(0, len(path_pts_l) - 1, 3):
                p1 = ((path_pts_l[i][0] + path_pts_r[i][0]) // 2, path_pts_l[i][1])
                nxt = min(i + 1, len(path_pts_l) - 1)
                p2 = ((path_pts_l[nxt][0] + path_pts_r[nxt][0]) // 2, path_pts_l[nxt][1])
                cv2.line(frame, p1, p2, (35, 185, 235), 2)

        # 3. Tree Shadows across path (for shadow test scenario)
        if add_shadows:
            shadow_overlay = frame.copy()
            for s_idx in range(4):
                sy = int(horizon_y + 40 + s_idx * 60)
                angle = 0.4
                pts_shadow = np.array([
                    [0, sy], [cam_w, sy + int(cam_w * math.tan(angle))],
                    [cam_w, sy + 35 + int(cam_w * math.tan(angle))], [0, sy + 35]
                ], dtype=np.int32)
                cv2.fillPoly(shadow_overlay, [pts_shadow], (10, 15, 20))
            frame = cv2.addWeighted(frame, 0.65, shadow_overlay, 0.35, 0)

        writer.write(frame)

        # Autonomous motion
        if track_type == "s_curve":
            target_y = 5.0 * math.sin((rx + 3.0) * 0.22)
        elif track_type == "switchback":
            target_y = 4.0 * math.sin((rx + 3.0) * 0.35) + 2.0 * math.cos((rx + 3.0) * 0.18)
        else:
            target_y = 3.0 * math.sin((rx + 3.0) * 0.15)

        err = (target_y - ry)
        omega = float(np.clip(1.2 * err - 0.8 * theta, -1.2, 1.2))
        rx += 1.0 * math.cos(theta) * 0.05
        ry += 1.0 * math.sin(theta) * 0.05
        theta += omega * 0.05

    writer.release()
    print(f"Generated: {filename} ({total_frames} frames)")


def generate_all():
    out_dir = "test_clips"
    generate_clip(os.path.join(out_dir, "sunny_curved_trail.mp4"), track_type="s_curve", add_shadows=False)
    generate_clip(os.path.join(out_dir, "shadowed_gravel_path.mp4"), track_type="s_curve", add_shadows=True)
    generate_clip(os.path.join(out_dir, "sharp_switchback.mp4"), track_type="switchback", add_shadows=False)


if __name__ == "__main__":
    generate_all()
