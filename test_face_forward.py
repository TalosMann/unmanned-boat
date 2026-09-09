"""
Two-thruster search-and-approach test: no face visible -> rotate in place
(differential thrust) to scan for one; face visible -> steer toward it
and drive forward. Uses MotorController's existing rotate_in_place() and
drive_toward() rather than driving both thrusters equally — same
steering logic fsm.py's SEARCH/APPROACH states use, just without the
ALIGN/SHOOT/RESET states yet.

Also serves a live annotated video feed (bounding box + SEARCHING/
APPROACHING status) so you can watch detection happen on your laptop's
browser and confirm your face is actually being seen.

Run on the Pi:
    python3 test_face_forward.py

Then on your Mac, open a browser to:
    http://talosbot.local:8000

Ctrl+C on the Pi to stop — cleanly stops thrusters and camera on exit.

SAFETY: drives real thrusters as soon as it starts (rotating to search)
and continues once your face is detected. Keep props clear / water-safe
area, be ready to cut battery power.
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

import config
from face_vision import FaceVision
from motor_control import MotorController

FORWARD_SPEED = 0.4       # 0-1, fraction of full speed, when face is found
SEARCH_SPEED = 0.15       # 0-1, differential-turn speed while scanning for a face —
                          # kept low since this just needs to sweep the camera, not
                          # spin fast; lower further if it's still too quick to react to
SEARCH_BURST_SEC = 5.0    # bench-test duty cycle: spin for this long...
SEARCH_REST_SEC = 30.0    # ...then sit idle for this long, repeating until a face is found
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


def control_loop(motors, camera, vision):
    global latest_jpeg

    last_log_at = 0.0
    search_phase_started_at = time.time()
    searching = True  # alternates with resting while no face is found

    try:
        while running:
            frame = camera.get_frame()
            face = vision.detect(frame)["red"]

            if face["ball_found"]:
                motors.drive_toward(face["x_offset"], speed=FORWARD_SPEED)
                status = f"APPROACHING (x_offset={face['x_offset']:+.2f})"
                # reset the duty cycle so if the face is lost again, search
                # resumes with a fresh burst rather than picking up mid-rest
                search_phase_started_at = time.time()
                searching = True
            else:
                elapsed = time.time() - search_phase_started_at
                phase_duration = SEARCH_BURST_SEC if searching else SEARCH_REST_SEC

                if elapsed >= phase_duration:
                    searching = not searching
                    search_phase_started_at = time.time()
                    elapsed = 0.0

                if searching:
                    motors.rotate_in_place(SEARCH_SPEED)
                    status = f"SEARCHING ({elapsed:.0f}/{SEARCH_BURST_SEC:.0f}s)"
                else:
                    motors.stop()
                    status = f"RESTING ({elapsed:.0f}/{SEARCH_REST_SEC:.0f}s)"

            if face["bbox"]:
                x, y, w, h = face["bbox"]
                color = (0, 255, 0) if face["ball_found"] else (0, 0, 255)
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, f"found={face['ball_found']} -> {status}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            ok, jpeg = cv2.imencode(".jpg", frame)
            if ok:
                with latest_lock:
                    latest_jpeg = jpeg.tobytes()

            # Plain newline'd, throttled to 1/sec — under an interactive
            # terminal this scrolls instead of overwriting in place, but
            # under systemd/journald a bare \r with no newline never gets
            # flushed as a line and eventually shows up as a giant opaque
            # "blob data" entry instead of readable log lines.
            now = time.time()
            if now - last_log_at > 1.0:
                print(f"[test] {status}", flush=True)
                last_log_at = now
            time.sleep(0.05)
    finally:
        motors.stop()


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
                    time.sleep(0.066)
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, format, *args):
        pass


def main():
    global running

    print("[test] Starting two-thruster face-forward test. Ctrl+C to stop.")
    motors = MotorController(config)
    camera = PiCameraGrabber(config.FRAME_WIDTH, config.FRAME_HEIGHT)
    vision = FaceVision()

    control_thread = threading.Thread(target=control_loop, args=(motors, camera, vision), daemon=True)
    control_thread.start()

    server = ThreadingHTTPServer(("0.0.0.0", PORT), StreamHandler)
    print(f"[test] Video feed at http://talosbot.local:{PORT} — Ctrl+C to stop.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[test] Stopped by user.")
    finally:
        running = False
        server.shutdown()
        motors.stop()
        camera.stop()
        print("[test] Thrusters and camera stopped.")


if __name__ == "__main__":
    main()
