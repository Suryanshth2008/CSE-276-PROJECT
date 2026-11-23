"""
Vision-Based Outdoor Navigation for Unmanned Ground Vehicles (UGV)

Core Vision & Navigation System:
1. Video / Camera Frame Acquisition.
2. Multi-space Traversable Path Detection & Shadow Compensation (perception.py).
3. Path Centroid & Lookahead Waypoint Tracking.
4. Formal Navigation State Machine & Steering Decision Generation (navigation.py).
5. Real-Time HUD Visualization & Frame-Level CSV Logging.
"""
import argparse
import csv
import os
import time
from datetime import datetime

os.environ.setdefault("OPENCV_LOG_LEVEL", "OFF")
os.environ.setdefault("OPENCV_VIDEOIO_PRIORITY_MSMF", "0")

import cv2
import numpy as np

from navigation import NavigationDecision, NavigationState, UGVNavigationEngine
from perception import PathDetectionResult, TraversablePathDetector


def build_parser():
    p = argparse.ArgumentParser(description="Vision-based UGV navigation prototype")
    p.add_argument("--source", default=None,
                   help="Video path or camera index (default: outdoor_video.mp4 if present, else 0)")
    p.add_argument("--output-dir", default="outputs", help="Folder for CSV logs and processed video")
    p.add_argument("--no-display", action="store_true", help="Run without opening display windows")
    p.add_argument("--loop", action="store_true", default=True,
                   help="Loop video playback continuously (default: True)")
    p.add_argument("--no-loop", action="store_false", dest="loop",
                   help="Play video once and hold the completed frame")
    p.add_argument("--min-area", type=int, default=1500, help="Minimum candidate path contour area")
    p.add_argument("--hsv-lower", nargs=3, type=int, default=[0, 0, 35], metavar=("H", "S", "V"),
                   help="Lower HSV threshold for candidate ground (default: 0 0 35)")
    p.add_argument("--hsv-upper", nargs=3, type=int, default=[180, 100, 230], metavar=("H", "S", "V"),
                   help="Upper HSV threshold for candidate ground (default: 180 100 230)")
    p.add_argument("--deadband", type=float, default=0.08,
                   help="Normalized center error considered FORWARD (default: 0.08)")
    p.add_argument("--disable-shadow-comp", action="store_true",
                   help="Disable LAB luminance shadow compensation")
    p.add_argument("--serial-port", default=None,
                   help="Serial port to send motor commands to Arduino (e.g. COM3 or /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=9600,
                   help="Baud rate for Arduino serial communication (default: 9600)")
    return p


def parse_source(source):
    return int(source) if source.isdigit() else source


# Backward-compatible standalone helper functions
_default_detector = None
_default_nav_engine = None


def detect_path(frame, lower, upper, min_area):
    """Backward-compatible helper function for detect_path."""
    global _default_detector
    if _default_detector is None or not np.array_equal(_default_detector.hsv_lower, lower):
        _default_detector = TraversablePathDetector(hsv_lower=lower, hsv_upper=upper, min_area=min_area)
    res = _default_detector.process_frame(frame)
    if not res.is_detected:
        return None, res.mask, res.roi_y
    return {"center": res.center, "contour": res.contour, "area": res.area}, res.mask, res.roi_y


def decide(center_x, frame_width, deadband):
    """Backward-compatible helper function for decide."""
    global _default_nav_engine
    if _default_nav_engine is None or _default_nav_engine.deadband != deadband:
        _default_nav_engine = UGVNavigationEngine(deadband=deadband)
    error = (center_x - frame_width / 2.0) / (frame_width / 2.0)
    if error < -deadband:
        action = "TURN LEFT"
    elif error > deadband:
        action = "TURN RIGHT"
    else:
        action = "STRAIGHT"
    return action, float(error)


def draw_hud(frame, result, decision, state, error, reason, fps, roi_y):
    """Draws a clean, publication-ready navigation HUD overlay on the video frame."""
    annotated = frame.copy()
    h, w = frame.shape[:2]

    # 1. Guide lines (Image center vertical reference + Horizon ROI)
    cv2.line(annotated, (w // 2, 0), (w // 2, h), (100, 100, 120), 1, cv2.LINE_AA)
    cv2.line(annotated, (0, roi_y), (w, roi_y), (255, 200, 0), 1, cv2.LINE_AA)
    cv2.putText(annotated, f"ROI HORIZON ({int(roi_y/h*100)}%)", (10, roi_y - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1, cv2.LINE_AA)

    # 2. Path Contour & Corridor Highlight
    if result.is_detected and result.contour is not None:
        overlay = annotated.copy()
        cv2.drawContours(overlay, [result.contour], -1, (0, 220, 100), -1)
        cv2.addWeighted(overlay, 0.25, annotated, 0.75, 0, annotated)
        cv2.drawContours(annotated, [result.contour], -1, (0, 255, 120), 2, cv2.LINE_AA)

        # Centroid
        cx, cy = result.center
        cv2.circle(annotated, (cx, cy), 8, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.circle(annotated, (cx, cy), 12, (255, 255, 255), 2, cv2.LINE_AA)

        # Lookahead Waypoint & Heading Vector
        if result.lookahead_waypoint:
            wx, wy = result.lookahead_waypoint
            cv2.drawMarker(annotated, (wx, wy), (0, 255, 255), cv2.MARKER_CROSS, 16, 2)
            cv2.line(annotated, (cx, cy), (wx, wy), (0, 255, 255), 2, cv2.LINE_AA)

        # Steering Guidance ray from rover base to centroid
        cv2.line(annotated, (w // 2, h), (cx, cy), (0, 220, 255), 2, cv2.LINE_AA)

    # 3. Decision & State Badge (Top Left)
    state_color = (0, 220, 100) if state == NavigationState.PATH_FOUND else \
                  ((0, 180, 255) if state == NavigationState.PATH_UNCERTAIN else (0, 0, 255))

    cv2.rectangle(annotated, (14, 12), (320, 95), (20, 25, 35), -1)
    cv2.rectangle(annotated, (14, 12), (320, 95), state_color, 2)

    cv2.putText(annotated, f"DECISION: {decision}", (24, 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.72, state_color, 2, cv2.LINE_AA)
    cv2.putText(annotated, f"STATE: {state}", (24, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 220, 220), 1, cv2.LINE_AA)

    err_str = f"{error:+.3f}" if result.is_detected else "N/A"
    cv2.putText(annotated, f"Error: {err_str} | Conf: {result.confidence*100:.0f}%", (24, 82),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 190, 205), 1, cv2.LINE_AA)

    # 4. Steering Gauge Bar (Bottom Center)
    gauge_w, gauge_h = 240, 12
    gx = (w - gauge_w) // 2
    gy = h - 24
    cv2.rectangle(annotated, (gx, gy), (gx + gauge_w, gy + gauge_h), (35, 40, 50), -1)
    cv2.rectangle(annotated, (gx, gy), (gx + gauge_w, gy + gauge_h), (120, 130, 145), 1)
    # Deadband box
    center_gx = gx + gauge_w // 2
    db_px = int(0.08 * (gauge_w // 2))
    cv2.rectangle(annotated, (center_gx - db_px, gy), (center_gx + db_px, gy + gauge_h), (40, 100, 50), -1)
    # Pointer pin
    pin_x = int(center_gx + np.clip(error, -1.0, 1.0) * (gauge_w // 2))
    cv2.circle(annotated, (pin_x, gy + gauge_h // 2), 6, (0, 0, 255), -1)
    cv2.putText(annotated, "L", (gx - 14, gy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)
    cv2.putText(annotated, "R", (gx + gauge_w + 5, gy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)

    # 5. FPS & Timestamp (Bottom Right)
    cv2.putText(annotated, f"Processing: {fps:.1f} FPS", (w - 180, h - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

    return annotated


def main():
    args = build_parser().parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    raw_source = args.source
    if raw_source is None:
        raw_source = "outdoor_video.mp4" if os.path.exists("outdoor_video.mp4") else "0"

    source = parse_source(raw_source)
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        if raw_source == "0":
            fallbacks = ["outdoor_video.mp4", os.path.join("test_clips", "sunny_curved_trail.mp4")]
            for fb in fallbacks:
                if os.path.exists(fb):
                    print(f"Notice: Camera index '0' not accessible. Automatically falling back to video source: {fb}")
                    source = fb
                    cap = cv2.VideoCapture(source)
                    if cap.isOpened():
                        break
        if not cap.isOpened():
            raise SystemExit(
                f"Could not open source {raw_source!r}. Check video path or camera index.\n"
                f"Tip: Run with: python main.py --source outdoor_video.mp4"
            )

    fps = cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps < 1 or fps > 120:
        fps = 20.0

    delay_ms = max(1, int(1000.0 / fps)) if isinstance(source, str) else 1

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.output_dir, f"navigation_log_{stamp}.csv")
    video_path = os.path.join(args.output_dir, f"processed_{stamp}.mp4")
    writer = None

    # Initialize Modules
    detector = TraversablePathDetector(
        hsv_lower=args.hsv_lower,
        hsv_upper=args.hsv_upper,
        min_area=args.min_area,
        use_shadow_compensation=not args.disable_shadow_comp
    )
    nav_engine = UGVNavigationEngine(deadband=args.deadband)

    # Optional Hardware Actuation: Arduino Uno Serial Connection
    arduino_serial = None
    if args.serial_port:
        try:
            import serial
            arduino_serial = serial.Serial(args.serial_port, args.baud, timeout=0.1)
            time.sleep(1.5)  # Allow Arduino bootloader to complete reset
            print(f"[HARDWARE] Connected to Arduino Uno on {args.serial_port} @ {args.baud} baud.")
        except Exception as e:
            print(f"[WARNING] Could not open serial port {args.serial_port}: {e}")
            print("          Continuing in software display / video analysis mode.")
            arduino_serial = None

    frame_idx = 0
    start = time.time()
    rows = []
    print("=" * 65)
    print("   Vision-Based Outdoor Navigation for Unmanned Ground Vehicles")
    print("   Student Mini-Project Prototype (Python + OpenCV)")
    print("=" * 65)
    print(f"Source: {raw_source} | Min Area: {args.min_area}px | Deadband: {args.deadband}")
    print(f"Output CSV: {csv_path}")
    if args.serial_port:
        print(f"Hardware Serial: {args.serial_port}")
    print("Press 'q' in the video window to stop early.\n")

    should_loop = args.loop and not args.no_display and isinstance(source, str)

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                if should_loop:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                if not args.no_display and frame_idx > 0:
                    cv2.putText(annotated, "RUN COMPLETE - Press any key in window to exit",
                                (20, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.imshow("Vision UGV Navigation Baseline", annotated)
                    print("\nRun complete. Press any key in the video window to close.")
                    cv2.waitKey(0)
                break

            frame_idx += 1
            h, w = frame.shape[:2]

            if writer is None:
                writer = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))

            # --- STEP 1: Computer Vision Perception Pipeline ---
            result = detector.process_frame(frame)

            # --- STEP 2: Autonomous Navigation Decision Engine ---
            decision, state, error, reason = nav_engine.step(result, w)

            # --- STEP 3: Optional Real-Time Motor Actuation (Arduino) ---
            if arduino_serial is not None:
                cmd_map = {
                    NavigationDecision.FORWARD: b'F',
                    NavigationDecision.TURN_LEFT: b'L',
                    NavigationDecision.TURN_RIGHT: b'R',
                    NavigationDecision.STOP: b'S',
                }
                char_to_send = cmd_map.get(decision, b'S')
                try:
                    arduino_serial.write(char_to_send)
                except Exception:
                    pass

            elapsed = max(time.time() - start, 1e-6)
            current_fps = frame_idx / elapsed

            # --- STEP 4: Real-Time HUD Visualization Overlay ---
            annotated = draw_hud(frame, result, decision, state, error, reason, current_fps, result.roi_y)

            rows.append({
                "frame": frame_idx,
                "timestamp_seconds": round(frame_idx / fps, 3),
                "status": state,
                "decision": decision,
                "normalized_center_error": round(error, 4) if result.is_detected else "",
                "heading_angle_deg": round(result.heading_angle_deg, 2) if result.is_detected else "",
                "confidence": round(result.confidence, 3),
                "candidate_area_pixels": round(result.area, 1) if result.is_detected else "",
                "failure_reason": result.failure_reason,
                "processing_fps_average": round(current_fps, 2),
            })
            writer.write(annotated)

            if not args.no_display:
                cv2.imshow("Vision UGV Navigation Baseline", annotated)
                cv2.imshow("Traversable Path Candidate Mask", result.mask)
                key = cv2.waitKey(delay_ms) & 0xFF
                if key == ord("q") or key == 27:
                    break

    finally:
        if arduino_serial is not None:
            try:
                arduino_serial.write(b'S')  # Command rover to safely STOP on exit
                arduino_serial.close()
                print("[HARDWARE] Safely stopped UGV and closed Arduino serial port.")
            except Exception:
                pass
        cap.release()
        if writer is not None:
            writer.release()
        if not args.no_display:
            cv2.destroyAllWindows()

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        fieldnames = ["frame", "timestamp_seconds", "status", "decision",
                      "normalized_center_error", "heading_angle_deg", "confidence",
                      "candidate_area_pixels", "failure_reason", "processing_fps_average"]
        out = csv.DictWriter(f, fieldnames=fieldnames)
        out.writeheader()
        out.writerows(rows)

    total = len(rows)
    found = sum(1 for row in rows if row["status"] == NavigationState.PATH_FOUND)
    uncertain = total - found
    print("\nFinished Processing.")
    print(f"Total Frames Processed: {total}")
    print(f"Path Successfully Tracked: {found} ({(found / total * 100):.1f}%)" if total else "No frames processed.")
    print(f"Path Uncertain/Lost: {uncertain}")
    print(f"CSV Run Log: {csv_path}")
    print(f"Processed Video: {video_path}")
    print(f"To evaluate run performance: python evaluate_runs.py --log {csv_path}")


if __name__ == "__main__":
    main()
