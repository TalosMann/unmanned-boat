"""
Live video feed with a bounding box around the detected face — same idea
as main.py's debug overlay for the ball, but viewable from your Mac's
browser instead of a local cv2 window (since the Pi is headless over SSH).

Run on the Pi:
    python3 stream_face_detection.py

Then on your Mac, open a browser to:
    http://talosbot.local:8000

No thrusters involved — camera + detection only. Ctrl+C on the Pi to stop.
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

from face_vision import FaceVision

PORT = 8000

latest_jpeg = None
latest_lock = threading.Lock()
running = True


class PiCameraGrabber:
    def __init__(self, width, height):
        from picamera2 import Picamera2

        self.picam2 = Picamera2()
        cam_config = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            buffer_count=2,  # low buffering so we always process the freshest
                             # frame instead of a growing backlog when
                             # detection can't keep up with capture rate
        )
        self.picam2.configure(cam_config)
        self.picam2.start()
        time.sleep(1.0)

    def get_frame(self):
        return self.picam2.capture_array()

    def stop(self):
        self.picam2.stop()


def capture_loop():
    global latest_jpeg
    camera = PiCameraGrabber(640, 480)
    vision = FaceVision()

    try:
        while running:
            frame = camera.get_frame()
            result = vision.detect(frame)
            face = result["red"]

            if face["bbox"]:
                x, y, w, h = face["bbox"]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            status = (f"found={face['ball_found']} "
                      f"x_offset={face['x_offset']:+.2f} dist={face['distance_est']:.0f}")
            cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            ok, jpeg = cv2.imencode(".jpg", frame)
            if ok:
                with latest_lock:
                    latest_jpeg = jpeg.tobytes()

            time.sleep(0.03)  # ~30fps capture; stream rate is separate, see below
    finally:
        camera.stop()


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(
                b"<html><body style='margin:0;background:#111;"
                b"display:flex;justify-content:center;align-items:center;min-height:100vh'>"
                b"<img src='/stream' style='max-width:100%;image-rendering:pixelated'>"
                b"</body></html>"
            )
            return

        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while running:
                    with latest_lock:
                        frame_bytes = latest_jpeg
                    if frame_bytes is not None:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(frame_bytes)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.066)  # ~15fps stream
            except (BrokenPipeError, ConnectionResetError):
                pass  # client closed the browser tab
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, format, *args):
        pass  # silence per-request access logs


def main():
    global running

    capture_thread = threading.Thread(target=capture_loop, daemon=True)
    capture_thread.start()

    server = ThreadingHTTPServer(("0.0.0.0", PORT), StreamHandler)
    print(f"[stream] Serving at http://talosbot.local:{PORT} — Ctrl+C to stop.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[stream] Stopping...")
    finally:
        running = False
        server.shutdown()


if __name__ == "__main__":
    main()
