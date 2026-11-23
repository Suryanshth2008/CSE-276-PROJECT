"""
Launch the interactive 3D WebGL / URDF visualizer for the Vision UGV Rover.
Opens the browser automatically to view the rover, drive it, inspect coordinate frames,
and verify the onboard camera's 42% ROI ground projection.
"""
import http.server
import os
import socketserver
import threading
import time
import webbrowser


def run_viewer(port=8080):
    model_dir = os.path.dirname(os.path.abspath(__file__))
    html_file = os.path.join(model_dir, "ugv_3d_viewer.html")

    if not os.path.exists(html_file):
        print(f"Error: {html_file} not found.")
        return

    os.chdir(model_dir)

    class CustomHandler(http.server.SimpleHTTPRequestHandler):
        def log_message(self, format, *args):
            pass  # Quiet logging

    # Try binding to port, fallback if busy
    server = None
    for test_port in range(port, port + 10):
        try:
            server = socketserver.TCPServer(("", test_port), CustomHandler)
            port = test_port
            break
        except OSError:
            continue

    if server is None:
        # Fallback to direct file url
        print("Starting direct file viewer in browser...")
        webbrowser.open(f"file:///{html_file.replace(os.sep, '/')}")
        return

    url = f"http://localhost:{port}/ugv_3d_viewer.html"
    print("=" * 65)
    print("   Vision UGV Rover - 3D Simulation & Perception Visualizer")
    print("=" * 65)
    print(f"Server started at: {url}")
    print("Controls:")
    print("  - Left-click drag: Rotate orbit")
    print("  - Right-click drag: Pan camera")
    print("  - Scroll wheel: Zoom in/out")
    print("  - W / A / S / D or Arrow Keys: Steer and drive the UGV")
    print("  - Bottom-right PIP window: Live Onboard Camera (42% ROI Line)")
    print("Press Ctrl+C in terminal to stop server.\n")

    threading.Thread(target=lambda: (time.sleep(0.6), webbrowser.open(url)), daemon=True).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping 3D viewer server.")
        server.server_close()


if __name__ == "__main__":
    run_viewer()
