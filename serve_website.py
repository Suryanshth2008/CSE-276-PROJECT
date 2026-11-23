"""
=============================================================================
  VISION UGV WEB DASHBOARD BACKEND SERVER
  Serves the website, streams live MJPEG video with HUD overlays,
  and provides REST APIs for live telemetry and parameter tuning.
=============================================================================
Usage:
    python serve_website.py [--port 8080] [--source outdoor_video.mp4]
"""
import argparse
import http.server
import json
import os
import socketserver
import sys
import threading
import time
import urllib.parse
import webbrowser

import cv2
import numpy as np

from navigation import NavigationDecision, NavigationState, UGVNavigationEngine
from perception import PathDetectionResult, TraversablePathDetector

# Global State
detector = TraversablePathDetector(hsv_lower=(0, 0, 35), hsv_upper=(180, 100, 230), min_area=1500)
nav_engine = UGVNavigationEngine(deadband=0.08)

current_source_path = "outdoor_video.mp4"
latest_hud_jpeg = None
latest_mask_jpeg = None
latest_raw_jpeg = None
latest_telemetry = {
    "frame": 1,
    "total_frames": 200,
    "decision": "FORWARD",
    "state": "PATH_FOUND",
    "error": 0.0,
    "heading_deg": 0.0,
    "confidence": 1.0,
    "area_px": 45000,
    "fps": "60.0",
}
is_paused = False
frame_lock = threading.Lock()


def draw_web_hud(frame, result, decision, state, error, roi_y, fps):
    annotated = frame.copy()
    h, w = frame.shape[:2]

    # Guide lines
    cv2.line(annotated, (w // 2, 0), (w // 2, h), (100, 100, 120), 1, cv2.LINE_AA)
    cv2.line(annotated, (0, roi_y), (w, roi_y), (255, 200, 0), 1, cv2.LINE_AA)
    cv2.putText(annotated, f"ROI HORIZON ({int(roi_y/h*100)}%)", (10, roi_y - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1, cv2.LINE_AA)

    # Path Corridor Highlight
    if result.is_detected and result.contour is not None:
        overlay = annotated.copy()
        cv2.drawContours(overlay, [result.contour], -1, (0, 220, 100), -1)
        cv2.addWeighted(overlay, 0.25, annotated, 0.75, 0, annotated)
        cv2.drawContours(annotated, [result.contour], -1, (0, 255, 120), 2, cv2.LINE_AA)

        cx, cy = result.center
        cv2.circle(annotated, (cx, cy), 8, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.circle(annotated, (cx, cy), 12, (255, 255, 255), 2, cv2.LINE_AA)

        if result.lookahead_waypoint:
            wx, wy = result.lookahead_waypoint
            cv2.drawMarker(annotated, (wx, wy), (0, 255, 255), cv2.MARKER_CROSS, 16, 2)
            cv2.line(annotated, (cx, cy), (wx, wy), (0, 255, 255), 2, cv2.LINE_AA)

        cv2.line(annotated, (w // 2, h), (cx, cy), (0, 220, 255), 2, cv2.LINE_AA)

    # Status Badge
    color = (0, 220, 100) if state == NavigationState.PATH_FOUND else \
            ((0, 180, 255) if state == NavigationState.PATH_UNCERTAIN else (0, 0, 255))
    cv2.rectangle(annotated, (14, 12), (320, 95), (20, 25, 35), -1)
    cv2.rectangle(annotated, (14, 12), (320, 95), color, 2)

    cv2.putText(annotated, f"DECISION: {decision}", (24, 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.72, color, 2, cv2.LINE_AA)
    cv2.putText(annotated, f"STATE: {state}", (24, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 220, 220), 1, cv2.LINE_AA)

    err_str = f"{error:+.3f}" if result.is_detected else "N/A"
    cv2.putText(annotated, f"Error: {err_str} | Conf: {result.confidence*100:.0f}%", (24, 82),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 190, 205), 1, cv2.LINE_AA)

    # Gauge
    gw, gh = 240, 12
    gx = (w - gw) // 2
    gy = h - 24
    cv2.rectangle(annotated, (gx, gy), (gx + gw, gy + gh), (35, 40, 50), -1)
    cv2.rectangle(annotated, (gx, gy), (gx + gw, gy + gh), (120, 130, 145), 1)
    cgx = gx + gw // 2
    db_px = int(0.08 * (gw // 2))
    cv2.rectangle(annotated, (cgx - db_px, gy), (cgx + db_px, gy + gh), (40, 100, 50), -1)
    pin_x = int(cgx + np.clip(error, -1.0, 1.0) * (gw // 2))
    cv2.circle(annotated, (pin_x, gy + gh // 2), 6, (0, 0, 255), -1)

    cv2.putText(annotated, f"Processing: {fps:.1f} FPS", (w - 180, h - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

    return annotated


def vision_worker():
    """Background thread continuously processing frames and encoding JPEG streams."""
    global latest_hud_jpeg, latest_mask_jpeg, latest_raw_jpeg, latest_telemetry, current_source_path

    while True:
        src = current_source_path
        if not os.path.exists(src):
            time.sleep(0.5)
            continue

        cap = cv2.VideoCapture(src)
        fps = cap.get(cv2.CAP_PROP_FPS) or 20.0
        delay = 1.0 / fps
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 200
        frame_idx = 0
        start = time.time()

        while cap.isOpened() and src == current_source_path:
            if is_paused:
                time.sleep(0.05)
                continue

            ok, frame = cap.read()
            if not ok:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                frame_idx = 0
                continue

            frame_idx += 1
            h, w = frame.shape[:2]

            # 1. Perception
            result = detector.process_frame(frame)

            # 2. Navigation
            decision, state, error, reason = nav_engine.step(result, w)

            elapsed = max(time.time() - start, 1e-6)
            cur_fps = frame_idx / elapsed

            # 3. Draw HUD
            annotated = draw_web_hud(frame, result, decision, state, error, result.roi_y, cur_fps)

            # 4. Encode JPEGs
            _, hud_jpg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
            mask_bgr = cv2.cvtColor(result.mask, cv2.COLOR_GRAY2BGR)
            _, mask_jpg = cv2.imencode(".jpg", mask_bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])
            _, raw_jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])

            with frame_lock:
                latest_hud_jpeg = hud_jpg.tobytes()
                latest_mask_jpeg = mask_jpg.tobytes()
                latest_raw_jpeg = raw_jpg.tobytes()
                latest_telemetry = {
                    "frame": frame_idx,
                    "total_frames": total,
                    "decision": decision,
                    "state": state,
                    "error": round(float(error), 4) if result.is_detected else 0.0,
                    "heading_deg": round(float(result.heading_angle_deg), 2) if result.is_detected else 0.0,
                    "confidence": round(float(result.confidence), 3),
                    "area_px": int(result.area) if result.is_detected else 0,
                    "fps": f"{cur_fps:.1f}",
                }

            time.sleep(delay * 0.9)

        cap.release()


class DashboardRequestHandler(http.server.SimpleHTTPRequestHandler):
    """Handles static files, MJPEG video streaming, and REST APIs."""

    def translate_path(self, path):
        # Serve from website/ folder for root requests
        parsed = urllib.parse.urlparse(path).path
        if parsed == "/" or parsed == "/index.html":
            return os.path.abspath(os.path.join("website", "index.html"))
        if parsed in ("/style.css", "/app.js"):
            return os.path.abspath(os.path.join("website", parsed.lstrip("/")))
        return super().translate_path(path)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        if path == "/stream":
            self.send_mjpeg_stream(parsed.query)
            return

        if path == "/api/telemetry":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            with frame_lock:
                self.wfile.write(json.dumps(latest_telemetry).encode("utf-8"))
            return

        return super().do_GET()

    def do_POST(self):
        global current_source_path, is_paused
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        content_len = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_len).decode("utf-8") if content_len > 0 else ""

        if path == "/api/set_source":
            qs = urllib.parse.parse_qs(parsed.query)
            src = qs.get("source", [None])[0]
            if src and os.path.exists(src):
                current_source_path = src
                self.send_json({"status": "ok", "source": src})
                return

        if path == "/api/set_params":
            try:
                params = json.loads(body)
                if "h_min" in params:
                    detector.hsv_lower[0] = int(params["h_min"])
                if "s_min" in params:
                    detector.hsv_lower[1] = int(params["s_min"])
                if "v_min" in params:
                    detector.hsv_lower[2] = int(params["v_min"])
                if "h_max" in params:
                    detector.hsv_upper[0] = int(params["h_max"])
                if "s_max" in params:
                    detector.hsv_upper[1] = int(params["s_max"])
                if "v_max" in params:
                    detector.hsv_upper[2] = int(params["v_max"])
                if "min_area" in params:
                    detector.min_area = int(params["min_area"])
                if "deadband" in params:
                    nav_engine.deadband = float(params["deadband"])
                if "roi_percent" in params:
                    detector.roi_fraction = float(params["roi_percent"]) / 100.0

                self.send_json({"status": "ok"})
                return
            except Exception as e:
                self.send_json({"status": "error", "message": str(e)}, 400)
                return

        if path == "/api/control":
            qs = urllib.parse.parse_qs(parsed.query)
            act = qs.get("action", [None])[0]
            if act == "toggle_pause":
                is_paused = not is_paused
            elif act == "restart":
                pass
            self.send_json({"status": "ok", "paused": is_paused})
            return

        if path == "/api/serial_send":
            qs = urllib.parse.parse_qs(parsed.query)
            cmd = qs.get("cmd", ["S"])[0]
            print(f"[SERIAL SIMULATOR] Sent command to Arduino: '{cmd}'")
            self.send_json({"status": "ok", "command": cmd})
            return

        self.send_response(404)
        self.end_headers()

    def send_json(self, data, code=200):
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode("utf-8"))

    def send_mjpeg_stream(self, query):
        qs = urllib.parse.parse_qs(query)
        view = qs.get("view", ["hud"])[0]

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        self.end_headers()

        try:
            while True:
                with frame_lock:
                    if view == "mask":
                        frame_data = latest_mask_jpeg
                    elif view == "raw":
                        frame_data = latest_raw_jpeg
                    else:
                        frame_data = latest_hud_jpeg

                if frame_data is not None:
                    header = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Content-Length: " + str(len(frame_data)).encode() + b"\r\n\r\n"
                    )
                    self.wfile.write(header)
                    self.wfile.write(frame_data)
                    self.wfile.write(b"\r\n")

                time.sleep(0.04)  # ~25 FPS stream
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, format, *args):
        # Quiet standard HTTP access log spam
        pass


def main():
    parser = argparse.ArgumentParser(description="Start Vision UGV Web Mission Control Dashboard")
    parser.add_argument("--port", type=int, default=8080, help="Port to host server (default: 8080)")
    parser.add_argument("--source", default="outdoor_video.mp4", help="Video source path")
    args = parser.parse_args()

    global current_source_path
    if os.path.exists(args.source):
        current_source_path = args.source

    # Start vision worker background thread
    t = threading.Thread(target=vision_worker, daemon=True)
    t.start()

    # Find open port
    port = args.port
    server = None
    for test_port in range(port, port + 10):
        try:
            server = http.server.ThreadingHTTPServer(("", test_port), DashboardRequestHandler)
            server.daemon_threads = True
            port = test_port
            break
        except OSError:
            continue

    if server is None:
        print("[ERROR] Could not bind to any port between 8080 and 8090.")
        sys.exit(1)

    url = f"http://localhost:{port}"
    print("=" * 70)
    print("   VISION UGV — AUTONOMOUS GROUND NAVIGATION MISSION CONTROL")
    print("   Web Dashboard & Multi-Module Inspection Studio")
    print("=" * 70)
    print(f"Server running at: {url}")
    print(f"Serving Website:   website/index.html")
    print(f"Live Vision Feed:  {url}/stream?view=hud")
    print(f"REST Telemetry:    {url}/api/telemetry")
    print("=" * 70)
    print("Opening web browser...\nPress Ctrl+C to stop server.\n")

    webbrowser.open(url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[INFO] Server stopped by user.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
