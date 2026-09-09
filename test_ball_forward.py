"""
Autonomous two-thruster search-and-approach using the RED BALL detector
(vision.BallVision). No ball visible -> continuously rotate in place to
scan, periodically reversing direction (so it sweeps the full area from
both sides and never winds up a trailing tether/cable indefinitely).
Ball visible -> steer toward it and drive forward.

Only the "red" result is used; the colored-ball confusion issue noted in
evaluate_detection.py earlier is a known, accepted limitation for this
bench test (not being scored/tuned against right now).

Camera capture, ball detection/motor control, and video streaming each
run on their own thread/pace:
  - camera_loop grabs frames as fast as the sensor allows.
  - control_loop runs detection (real compute work — HSV+Hough+CSRT) and
    drives the motors at whatever pace that takes; this is what actually
    governs reaction time to the ball.
  - stream_loop redraws the latest camera frame with the latest known
    detection result at a fixed higher rate, so the web feed stays smooth
    even when detection itself is running slower than the camera's fps.

Run on the Pi:
    python3 test_ball_forward.py

Then on your Mac, open a browser to:
    http://talosbot.local:8000

Ctrl+C on the Pi to stop — cleanly stops thrusters and camera on exit.

SAFETY: drives real thrusters as soon as it starts (rotating to search)
and continues once the red ball is detected. Keep props clear / water-safe
area, be ready to cut battery power.
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

import config
from vision import BallVision
from motor_control import MotorController

FORWARD_SPEED = 0.6       # 0-1, fraction of full speed, when ball is found
SEARCH_SPEED = 0.15       # 0-1, differential-turn speed while scanning for the ball
SEARCH_REVERSE_INTERVAL_SEC = 10.0  # flip spin direction this often while searching —
                                    # covers the area from both directions and avoids
                                    # winding up a trailing tether/cable indefinitely
PORT = 8000
STREAM_FPS_INTERVAL = 1 / 20  # redraw/encode rate for the web feed, independent of detection speed

latest_frame_lock = threading.Lock()
latest_frame = None          # freshest raw camera frame (post-rotation), no overlay

latest_detection_lock = threading.Lock()
latest_detection = {"ball_found": False, "bbox": None, "status": "starting..."}

latest_jpeg_lock = threading.Lock()
latest_jpeg = None

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


def camera_loop(camera):
    """Continuously refreshes the shared latest_frame as fast as the
    sensor provides frames — decoupled from how long detection takes."""
    global latest_frame
    while running:
        frame = camera.get_frame()
        frame = cv2.rotate(frame, cv2.ROTATE_180)  # camera is mounted upside down
        with latest_frame_lock:
            latest_frame = frame


def control_loop(motors, vision):
    """Runs ball detection + drives the motors. Paced by however long
    detection actually takes — this governs real reaction time, not the
    video feed's smoothness."""
    global latest_detection

    last_log_at = 0.0
    search_direction = 1
    last_direction_flip_at = time.time()
    last_seen_x_offset = 0.0
    last_ball_seen_at = None  # None until the ball has been seen at least once

    try:
        while running:
            with latest_frame_lock:
                frame = latest_frame
            if frame is None:
                time.sleep(0.01)
                continue

            ball = vision.detect(frame)["red"]

            if ball["ball_found"]:
                motors.drive_toward(ball["x_offset"], speed=FORWARD_SPEED)
                status = f"APPROACHING (x_offset={ball['x_offset']:+.2f})"
                last_seen_x_offset = ball["x_offset"]
                last_ball_seen_at = time.time()
                # reset so if the ball is lost again, search resumes with a
                # fresh direction/timer rather than picking up mid-sweep
                last_direction_flip_at = time.time()
                search_direction = 1
            elif last_ball_seen_at is not None and time.time() - last_ball_seen_at < config.LOST_BALL_TIMEOUT:
                # brief dropout (motion blur mid-turn, momentary occlusion) —
                # keep steering toward where it was last seen instead of
                # immediately giving up and spinning back into SEARCHING
                lost_for = time.time() - last_ball_seen_at
                motors.drive_toward(last_seen_x_offset, speed=FORWARD_SPEED)
                status = f"APPROACHING (lost {lost_for:.1f}/{config.LOST_BALL_TIMEOUT:.1f}s, last x_offset={last_seen_x_offset:+.2f})"
            else:
                elapsed = time.time() - last_direction_flip_at
                if elapsed >= SEARCH_REVERSE_INTERVAL_SEC:
                    search_direction *= -1
                    last_direction_flip_at = time.time()
                    elapsed = 0.0

                motors.rotate_in_place(SEARCH_SPEED * search_direction)
                direction_label = "right" if search_direction > 0 else "left"
                status = f"SEARCHING ({direction_label}, {elapsed:.0f}/{SEARCH_REVERSE_INTERVAL_SEC:.0f}s)"

            with latest_detection_lock:
                latest_detection = {
                    "ball_found": ball["ball_found"],
                    "bbox": ball["bbox"],
                    "status": status,
                }

            now = time.time()
            if now - last_log_at > 1.0:
                print(f"[test] {status}", flush=True)
                last_log_at = now
    finally:
        motors.stop()


def stream_loop():
    """Redraws whatever the freshest camera frame is with the freshest
    known detection result, at a fixed rate — keeps the web feed smooth
    regardless of how fast/slow detection itself is running."""
    global latest_jpeg

    while running:
        with latest_frame_lock:
            frame = latest_frame
        if frame is None:
            time.sleep(0.05)
            continue
        frame = frame.copy()

        with latest_detection_lock:
            detection = dict(latest_detection)

        if detection["bbox"]:
            x, y, w, h = detection["bbox"]
            color = (0, 255, 0) if detection["ball_found"] else (0, 0, 255)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        cv2.putText(frame, f"found={detection['ball_found']} -> {detection['status']}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        ok, jpeg = cv2.imencode(".jpg", frame)
        if ok:
            with latest_jpeg_lock:
                latest_jpeg = jpeg.tobytes()

        time.sleep(STREAM_FPS_INTERVAL)


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
                    with latest_jpeg_lock:
                        frame_bytes = latest_jpeg
                    if frame_bytes is not None:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(frame_bytes)
                        self.wfile.write(b"\r\n")
                    time.sleep(STREAM_FPS_INTERVAL)
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, format, *args):
        pass


def main():
    global running

    print("[test] Starting two-thruster ball-forward test. Ctrl+C to stop.")
    motors = MotorController(config)
    camera = PiCameraGrabber(config.FRAME_WIDTH, config.FRAME_HEIGHT)
    vision = BallVision(config)

    threading.Thread(target=camera_loop, args=(camera,), daemon=True).start()
    threading.Thread(target=control_loop, args=(motors, vision), daemon=True).start()
    threading.Thread(target=stream_loop, daemon=True).start()

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
