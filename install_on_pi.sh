#!/usr/bin/env bash
# Run this ON THE PI. Writes config.py, motor_control.py, and
# test_thrusters.py into ~/ire, then installs gpiozero + lgpio
# (the modern GPIO stack — pigpio is no longer packaged on Debian Trixie
# and doesn't reliably support newer Pi kernels anyway).
#
# Get this file onto the Pi first (from your Mac's own Terminal, not
# through Claude): scp install_on_pi.sh talosgiant@talosbot.local:~/
# Then on the Pi: bash install_on_pi.sh
#
# IMPORTANT: also copy haarcascade_frontalface_default.xml into ~/ire —
# it's not embedded in this script (930KB binary-ish XML, not worth
# inlining) but face_vision.py requires it:
#   scp haarcascade_frontalface_default.xml talosgiant@talosbot.local:~/ire/

set -e

echo "Installing OpenCV + picamera2..."
sudo apt install -y python3-opencv python3-picamera2

mkdir -p ~/ire
cd ~/ire

cat > config.py << 'PYEOF'
"""
Tunable constants. Keep everything you'll adjust at the water's edge here,
not buried in logic files.
"""

# --- HSV thresholds (tune these on-site against actual river lighting) ---
# Derived from auto_calibrate_hsv.py against video_3/video_5's labeled
# ground truth (indoor basketball footage, NOT the Pi's camera/lighting —
# treat as a starting point and re-calibrate against real Pi-camera/water
# footage once available). Red wraps around the HSV hue circle (0 and 180
# are both "red"), so it needs two ranges OR'd together.
RED_HSV_LOWER1 = (0, 165, 109)
RED_HSV_UPPER1 = (9, 255, 255)
RED_HSV_LOWER2 = (170, 165, 109)
RED_HSV_UPPER2 = (180, 255, 255)

COLORED_HSV_LOWER = (20, 100, 100)   # example: yellow-ish ball — retune on-site
COLORED_HSV_UPPER = (35, 255, 255)

MIN_BALL_RADIUS_PX = 8       # ignore detections smaller than this (noise)

# CSRT tracks generic visual appearance, not color/shape — it can happily
# drift onto the wrong object (a vest, background, frame edge) and keep
# reporting success. Each frame we re-check that the tracked box still
# actually matches the ball's HSV color; below this fraction, we treat the
# track as lost and fall back to HSV+Hough re-acquisition.
MIN_COLOR_MATCH_FRACTION = 0.35

# --- FSM tuning ---
SEARCH_ROTATE_SPEED = 0.3       # 0-1 fraction of max rotate speed
APPROACH_SPEED = 0.5
ALIGN_ROTATE_SPEED = 0.25
ALIGN_TOLERANCE = 0.08          # x_offset considered "centered" if abs < this
ALIGN_DISTANCE_THRESHOLD = 60   # ball radius (px) at which to stop approaching, start aligning
LOST_BALL_TIMEOUT = 1.5         # seconds before dropping back to SEARCH
RESET_PAUSE_SEC = 1.0

# --- Safety ---
ESTOP_GPIO_PIN = 17             # hardware e-stop input pin
ESTOP_POLL_HZ = 50               # check e-stop at this frequency, independent of main loop

# --- Camera ---
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# --- Thrusters (motor_control.MotorController, gpiozero + bidirectional ESCs) ---
LEFT_THRUSTER_PIN = 12    # BCM GPIO number, not physical pin. BCM12 = physical pin 32
                          # (matches the single ESC currently wired to physical pin 32/34)
RIGHT_THRUSTER_PIN = 19  # BCM19 = physical pin 35 — not yet wired to a second ESC
ESC_NEUTRAL_US = 1500     # standard RC pulse widths: 1000=full reverse, 1500=stop, 2000=full forward
ESC_MIN_US = 1000
ESC_MAX_US = 2000
ESC_ARM_DELAY_SEC = 2.0   # most ESCs require a sustained neutral signal before they'll respond
STEER_GAIN = 0.5          # how hard x_offset biases the differential during drive_toward
SHOOT_BURST_SEC = 0.5     # duration of full-power burst on shoot()
PYEOF

cat > motor_control.py << 'PYEOF'
"""
Two motor controller implementations:
  - MockMotorController: prints actions, for testing on a laptop webcam
    with no ESCs/thrusters attached.
  - MotorController: real PWM/ESC driver for the Pi, via gpiozero + two
    bidirectional RC-style ESCs (one per thruster, differential drive).

main.py should import MockMotorController for webcam testing and swap to
MotorController once you're wiring real thrusters.

MotorController uses gpiozero.Servo (backed by the lgpio pin factory,
gpiozero's default on modern Raspberry Pi OS / Pi 5). Install with:
    sudo apt install python3-gpiozero python3-lgpio
No daemon needs to be started (unlike pigpio, which Debian Trixie no
longer packages and which doesn't reliably support newer Pi kernels).

Wire the two ESC signal wires to config.LEFT_THRUSTER_PIN /
config.RIGHT_THRUSTER_PIN (BCM numbering). ESC power/ground comes from the
battery, not the Pi — only the PWM signal wire (+ shared ground) goes to
the GPIO pins.

Servo.detach() (used in emergency_stop) stops sending pulses, which most
ESCs treat as a stop condition, but that's a software fail-safe, not a
guaranteed power cut — it does NOT satisfy rulebook §4.4's hardware
e-stop requirement (main.py's EStopMonitor / real GPIO relay is what's
required for that).
"""

import time


class MockMotorController:
    def __init__(self):
        self._last_action = None

    def _log(self, msg):
        if msg != self._last_action:  # avoid spamming identical repeats
            print(f"[motors] {msg}")
            self._last_action = msg

    def rotate_in_place(self, speed):
        direction = "right" if speed > 0 else "left"
        self._log(f"rotate_in_place({direction}, speed={abs(speed):.2f})")

    def drive_toward(self, x_offset, speed):
        self._log(f"drive_toward(x_offset={x_offset:.2f}, speed={speed:.2f})")

    def shoot(self):
        print("[motors] SHOOT")

    def stop(self):
        self._log("stop()")

    def emergency_stop(self):
        print("[motors] *** EMERGENCY STOP ***")


class MotorController:
    """Real hardware driver: two bidirectional ESCs (left/right thruster)
    over gpiozero Servo PWM, matching MockMotorController's interface so
    main.py doesn't need to change when you swap it in."""

    def __init__(self, config):
        from gpiozero import Servo  # imported here so this module still loads on machines without gpiozero (e.g. your laptop)

        self.config = config
        min_pulse_s = config.ESC_MIN_US / 1_000_000
        max_pulse_s = config.ESC_MAX_US / 1_000_000

        self.left = Servo(config.LEFT_THRUSTER_PIN, min_pulse_width=min_pulse_s, max_pulse_width=max_pulse_s)
        self.right = Servo(config.RIGHT_THRUSTER_PIN, min_pulse_width=min_pulse_s, max_pulse_width=max_pulse_s)
        self._arm()

    def _arm(self):
        """Most ESCs require a sustained neutral (stop) signal before they'll
        accept other commands — this is their startup calibration/safety check."""
        print("[motors] Arming ESCs (holding neutral)...")
        self.left.value = 0.0
        self.right.value = 0.0
        time.sleep(self.config.ESC_ARM_DELAY_SEC)
        print("[motors] ESCs armed.")

    def _set_thruster(self, servo, speed):
        """speed: -1.0 (full reverse) to 1.0 (full forward)."""
        servo.value = max(-1.0, min(1.0, speed))

    def rotate_in_place(self, speed):
        # positive speed = rotate right: left thruster forward, right thruster reverse
        speed = max(-1.0, min(1.0, speed))
        self._set_thruster(self.left, speed)
        self._set_thruster(self.right, -speed)

    def drive_toward(self, x_offset, speed):
        # x_offset in [-1, 1] (positive = target is right of center). Matches
        # rotate_in_place's convention (left relatively faster -> turn right):
        # target to the right -> boost left / ease right so the boat curves
        # toward it while still moving forward.
        steer = x_offset * self.config.STEER_GAIN
        self._set_thruster(self.left, speed + steer)
        self._set_thruster(self.right, speed - steer)

    def shoot(self):
        print("[motors] SHOOT")
        self._set_thruster(self.left, 1.0)
        self._set_thruster(self.right, 1.0)
        time.sleep(self.config.SHOOT_BURST_SEC)
        self.stop()

    def stop(self):
        self._set_thruster(self.left, 0.0)
        self._set_thruster(self.right, 0.0)

    def emergency_stop(self):
        print("[motors] *** EMERGENCY STOP ***")
        # detach() = stop sending pulses entirely, rather than commanding
        # neutral — a software fail-safe most ESCs honor, but not a
        # substitute for the hardware e-stop relay required by rulebook §4.4.
        self.left.detach()
        self.right.detach()
PYEOF

cat > test_thrusters.py << 'PYEOF'
"""
Standalone bench test for MotorController — run this on the Pi with props
OFF before ever trusting the FSM/vision loop to drive real thrusters.
Confirms wiring, pin numbers, and direction sense one motion at a time.

Requires gpiozero + lgpio installed:
    sudo apt install python3-gpiozero python3-lgpio

Usage (on the Pi):
    python3 test_thrusters.py

Each step prints what it's about to do and waits for Enter before it does
it, and waits for Enter again before moving on — so you can react/cut
power if anything spins the wrong way or unexpectedly.
"""

import config
from motor_control import MotorController


def step(description, action, motors):
    input(f"\n[test] About to: {description}. Press Enter to run it (Ctrl+C to abort)...")
    action(motors)
    input("[test] Running. Press Enter to stop and continue to the next step...")
    motors.stop()


def main():
    print("[test] Initializing GPIO and arming ESCs...")
    motors = MotorController(config)

    try:
        step("LEFT thruster forward, half speed",
             lambda m: m._set_thruster(m.left, 0.5), motors)
        step("LEFT thruster reverse, half speed",
             lambda m: m._set_thruster(m.left, -0.5), motors)
        step("RIGHT thruster forward, half speed",
             lambda m: m._set_thruster(m.right, 0.5), motors)
        step("RIGHT thruster reverse, half speed",
             lambda m: m._set_thruster(m.right, -0.5), motors)
        step("rotate_in_place(right), both thrusters opposing",
             lambda m: m.rotate_in_place(0.4), motors)
        step("rotate_in_place(left), both thrusters opposing",
             lambda m: m.rotate_in_place(-0.4), motors)
        step("drive_toward(x_offset=0.0, speed=0.4) — straight forward",
             lambda m: m.drive_toward(0.0, 0.4), motors)
        step("drive_toward(x_offset=0.5, speed=0.4) — should curve right",
             lambda m: m.drive_toward(0.5, 0.4), motors)
        step("shoot() — brief full-power burst then stop",
             lambda m: m.shoot(), motors)

        print("\n[test] All steps complete.")

    except KeyboardInterrupt:
        print("\n[test] Aborted — stopping thrusters.")
    finally:
        motors.stop()
        print("[test] Thrusters stopped. Done.")


if __name__ == "__main__":
    main()
PYEOF

echo "Files written to ~/ire"
echo "Installing gpiozero + lgpio..."
sudo apt install -y python3-gpiozero python3-lgpio

cat > face_vision.py << 'PYEOF'
"""
Face detector standing in for the ball detector — lets you test the full
camera -> detection -> FSM -> thrusters pipeline without a physical ball
on hand. Uses OpenCV's built-in Haar cascade (bundled with opencv-python,
no extra download). Not a replacement for vision.py's HSV+Hough ball
detector — swap back to that once you're testing with real balls again.
"""

import glob
import os

import cv2

# Debian's python3-opencv package doesn't ship the cv2.data submodule (unlike
# pip's opencv-python), even though the cascade XML files are still installed
# on disk — just under the system package path instead. Try both.
_CASCADE_SEARCH_PATHS = [
    lambda: cv2.data.haarcascades if hasattr(cv2, "data") else None,
    lambda: "/usr/share/opencv4/haarcascades/",
    lambda: "/usr/share/opencv/haarcascades/",
]


def _find_cascade_path(filename):
    for get_dir in _CASCADE_SEARCH_PATHS:
        directory = get_dir()
        if directory and os.path.isfile(os.path.join(directory, filename)):
            return os.path.join(directory, filename)
    # last resort: search the filesystem for it
    matches = glob.glob(f"/usr/**/{filename}", recursive=True)
    if matches:
        return matches[0]
    return None


class FaceVision:
    def __init__(self):
        cascade_path = _find_cascade_path("haarcascade_frontalface_default.xml")
        if cascade_path is None:
            raise RuntimeError(
                "Could not find haarcascade_frontalface_default.xml anywhere on this system. "
                "Try: sudo apt install --reinstall python3-opencv"
            )
        self.detector = cv2.CascadeClassifier(cascade_path)
        if self.detector.empty():
            raise RuntimeError(f"Found {cascade_path} but OpenCV failed to load it")

    def detect(self, frame):
        """Returns the same {"red": {...}, "colored": {...}} shape BallVision
        does, so it's a drop-in swap for fsm.py — the largest detected face
        goes in "red" (the FSM's default target when nothing colored is
        found), "colored" is always empty."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.detector.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40))

        empty = {"ball_found": False, "x_offset": 0.0, "distance_est": 0, "bbox": None}
        result = dict(empty)

        if len(faces) > 0:
            x, y, w, h = max(faces, key=lambda f: f[2] * f[3])  # largest/closest face
            h_frame, w_frame = frame.shape[:2]
            cx = x + w / 2
            result["ball_found"] = True
            result["x_offset"] = (cx - w_frame / 2) / (w_frame / 2)
            result["distance_est"] = (w + h) / 4
            result["bbox"] = (x, y, w, h)

        return {"red": result, "colored": dict(empty)}
PYEOF

cat > main_face_demo.py << 'PYEOF'
"""
End-to-end chase demo: Pi Camera Module -> face detection -> FSM -> real
thrusters. Stand-in for ball-chasing while testing without a physical
ball — swap face_vision.FaceVision back to vision.BallVision once you
have one again (see main.py for the real entry point).

Run on the Pi:
    python3 main_face_demo.py

Requires the Pi Camera Module + libcamera stack:
    sudo apt install python3-picamera2   (usually preinstalled on Raspberry Pi OS)

Ctrl+C to stop — cleanly stops the thrusters and camera on exit.

SAFETY: this drives real thrusters via MotorController the moment a face
is detected. Keep props clear / in water only, same as any other real
hardware test, and be ready to cut battery power if anything misbehaves —
this demo does not wire up the e-stop.
"""

import time

import config
from fsm import BoatFSM
from face_vision import FaceVision
from motor_control import MotorController


class PiCameraGrabber:
    def __init__(self, width, height):
        from picamera2 import Picamera2

        self.picam2 = Picamera2()
        cam_config = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.picam2.configure(cam_config)
        self.picam2.start()
        time.sleep(1.0)  # let auto-exposure/white-balance settle before the first frame

    def get_frame(self):
        # picamera2's "RGB888" format is actually BGR byte order despite the
        # name — matches what OpenCV expects, no channel swap needed.
        return self.picam2.capture_array()

    def stop(self):
        self.picam2.stop()


def main():
    print("[demo] Starting face-chase demo. Ctrl+C to stop.")
    motors = MotorController(config)
    camera = PiCameraGrabber(config.FRAME_WIDTH, config.FRAME_HEIGHT)
    vision = FaceVision()
    fsm = BoatFSM(motors, config)

    try:
        while True:
            frame = camera.get_frame()
            vision_result = vision.detect(frame)
            fsm.update(vision_result)

            face = vision_result["red"]
            print(f"[demo] state={fsm.state.value:8s} found={face['ball_found']!s:5s} "
                  f"x_offset={face['x_offset']:+.2f} dist={face['distance_est']:.0f}  ",
                  end="\r")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[demo] Stopped by user.")
    finally:
        motors.stop()
        camera.stop()
        print("[demo] Thrusters and camera stopped.")


if __name__ == "__main__":
    main()
PYEOF

cat > test_face_detection.py << 'PYEOF'
"""
Camera + face detection only — no thrusters, no FSM. Confirms the Pi
Camera Module and FaceVision work before wiring anything to motors.

Run on the Pi:
    python3 test_face_detection.py

Prints live detection status to the terminal. Since this typically runs
headless over SSH (no GUI), it also saves an annotated snapshot to
./face_test_snapshot.jpg every 2 seconds so you can pull it down and look
(scp talosgiant@talosbot.local:~/ire/face_test_snapshot.jpg . from your Mac).

Ctrl+C to stop.
"""

import time

import cv2

from face_vision import FaceVision


class PiCameraGrabber:
    def __init__(self, width, height):
        from picamera2 import Picamera2

        self.picam2 = Picamera2()
        cam_config = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.picam2.configure(cam_config)
        self.picam2.start()
        time.sleep(1.0)

    def get_frame(self):
        return self.picam2.capture_array()

    def stop(self):
        self.picam2.stop()


def main():
    print("[test] Starting camera + face detection test. Ctrl+C to stop.")
    camera = PiCameraGrabber(640, 480)
    vision = FaceVision()

    last_snapshot_at = 0.0

    try:
        while True:
            frame = camera.get_frame()
            result = vision.detect(frame)
            face = result["red"]

            status = (f"found={face['ball_found']!s:5s} "
                      f"x_offset={face['x_offset']:+.2f} dist={face['distance_est']:.0f}")
            print(f"[test] {status}      ", end="\r")

            now = time.time()
            if now - last_snapshot_at > 2.0:
                annotated = frame.copy()
                if face["bbox"]:
                    x, y, w, h = face["bbox"]
                    cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(annotated, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imwrite("face_test_snapshot.jpg", annotated)
                last_snapshot_at = now

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[test] Stopped by user.")
    finally:
        camera.stop()


if __name__ == "__main__":
    main()
PYEOF

cat > stream_face_detection.py << 'PYEOF'
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
            main={"size": (width, height), "format": "RGB888"}
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
                b"<html><body style='margin:0;background:#111'>"
                b"<img src='/stream' style='width:100%'></body></html>"
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
PYEOF

cat > vision.py << 'PYEOF'
"""
Vision pipeline: HSV masking -> Hough Circle detection -> CSRT tracking.

detect() is the main entry point called from main.py each frame.
Tracks the red ball and the colored ball independently (both are on the
field simultaneously per the rulebook). Each ball maintains its own CSRT
tracker across frames to smooth out detection gaps, re-acquiring via
HSV+Hough when the tracker is lost or not yet initialized.

If CSRT isn't available in the installed OpenCV build (needs
opencv-contrib-python), tracking is disabled and every frame falls back
to cold-start HSV+Hough detection — slower to re-acquire on partial
occlusion, but never crashes.
"""

import cv2
import numpy as np


def _make_tracker():
    """Returns a fresh CSRT tracker, or None if CSRT isn't available."""
    if hasattr(cv2, "TrackerCSRT_create"):
        return cv2.TrackerCSRT_create()
    if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
        return cv2.legacy.TrackerCSRT_create()
    return None


class _SingleBallTracker:
    """HSV+Hough re-acquisition with an optional CSRT tracker for one ball color."""

    def __init__(self, hsv_ranges, min_radius_px, min_color_match_fraction):
        # hsv_ranges: list of (lower, upper) tuples, OR'd together (red needs two)
        self.hsv_ranges = hsv_ranges
        self.min_radius_px = min_radius_px
        self.min_color_match_fraction = min_color_match_fraction
        self.tracker = None
        self.tracking = False
        self.tracker_available = _make_tracker() is not None

    def _color_mask(self, region):
        hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
        mask = None
        for lower, upper in self.hsv_ranges:
            part = cv2.inRange(hsv, lower, upper)
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        return mask

    def _still_on_target(self, frame, bbox):
        """CSRT tracks generic appearance, not color/shape — it can drift onto
        the wrong object and keep reporting success. Sanity-check that the
        tracked box still actually looks like the ball: a solid contiguous
        colored disc, not just "a similar percentage of reddish pixels"
        (which a wrinkled vest or a partially-colored ball can also produce)."""
        x, y, w, h = [int(v) for v in bbox]
        h_frame, w_frame = frame.shape[:2]
        x0, y0 = max(0, x), max(0, y)
        x1, y1 = min(w_frame, x + w), min(h_frame, y + h)
        if x1 <= x0 or y1 <= y0:
            return False  # box entirely off-frame (e.g. stuck at the edge)

        region = frame[y0:y1, x0:x1]
        mask = self._color_mask(region)
        match_fraction = float(np.count_nonzero(mask)) / mask.size
        if match_fraction < self.min_color_match_fraction:
            return False

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        perimeter = cv2.arcLength(largest, True)
        if perimeter == 0:
            return False
        # circularity: 1.0 for a perfect circle, lower for scattered/irregular
        # blobs (wrinkles, fabric folds, a partial color panel on the wrong ball)
        circularity = 4 * np.pi * area / (perimeter ** 2)
        return circularity >= 0.5

    def _detect_via_hsv_hough(self, frame):
        mask = self._color_mask(frame)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        blurred = cv2.GaussianBlur(mask, (9, 9), 2)

        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=50,
            param1=50,
            param2=25,
            minRadius=self.min_radius_px,
            maxRadius=0,
        )

        if circles is None:
            return None

        circles = np.round(circles[0, :]).astype("int")
        x, y, r = max(circles, key=lambda c: c[2])
        return (x, y, r)

    def _start_tracker(self, frame, bbox):
        if not self.tracker_available:
            return
        x, y, r = bbox
        roi = (x - r, y - r, 2 * r, 2 * r)
        tracker = _make_tracker()
        if tracker is None:
            self.tracker_available = False
            return
        try:
            tracker.init(frame, roi)
            self.tracker = tracker
            self.tracking = True
        except cv2.error:
            # ROI outside frame bounds or similar — just skip tracking this frame
            self.tracker = None
            self.tracking = False

    def detect(self, frame):
        h_frame, w_frame = frame.shape[:2]
        result = {"ball_found": False, "x_offset": 0.0, "distance_est": 0, "bbox": None}

        if self.tracking and self.tracker is not None:
            ok, bbox = self.tracker.update(frame)
            if ok and self._still_on_target(frame, bbox):
                x, y, w, h = bbox
                cx = x + w / 2
                cy = y + h / 2
                result["ball_found"] = True
                result["x_offset"] = (cx - w_frame / 2) / (w_frame / 2)
                result["distance_est"] = (w + h) / 4
                result["bbox"] = (int(x), int(y), int(w), int(h))
                return result
            else:
                self.tracking = False  # lost or drifted off-target — re-acquire

        detected = self._detect_via_hsv_hough(frame)
        if detected is None:
            return result

        x, y, r = detected
        self._start_tracker(frame, (x, y, r))

        result["ball_found"] = True
        result["x_offset"] = (x - w_frame / 2) / (w_frame / 2)
        result["distance_est"] = r
        result["bbox"] = (x - r, y - r, 2 * r, 2 * r)
        return result


class BallVision:
    def __init__(self, config):
        self.config = config
        self._red = _SingleBallTracker(
            hsv_ranges=[
                (config.RED_HSV_LOWER1, config.RED_HSV_UPPER1),
                (config.RED_HSV_LOWER2, config.RED_HSV_UPPER2),
            ],
            min_radius_px=config.MIN_BALL_RADIUS_PX,
            min_color_match_fraction=config.MIN_COLOR_MATCH_FRACTION,
        )
        self._colored = _SingleBallTracker(
            hsv_ranges=[(config.COLORED_HSV_LOWER, config.COLORED_HSV_UPPER)],
            min_radius_px=config.MIN_BALL_RADIUS_PX,
            min_color_match_fraction=config.MIN_COLOR_MATCH_FRACTION,
        )
        if not self._red.tracker_available:
            print("[vision] WARNING: CSRT tracker unavailable (install opencv-contrib-python) "
                  "— falling back to per-frame HSV+Hough re-detection, no tracking smoothing.")

    def detect(self, frame):
        """
        Returns {"red": {...}, "colored": {...}}, each shaped like the
        single-ball result: {"ball_found": bool, "x_offset": float,
        "distance_est": float, "bbox": (x, y, w, h) or None}
        """
        return {
            "red": self._red.detect(frame),
            "colored": self._colored.detect(frame),
        }
PYEOF

cat > fsm.py << 'PYEOF'
"""
Finite State Machine for autonomous ball-chasing behavior.
States: SEARCH -> APPROACH -> ALIGN -> SHOOT -> RESET
"""

from enum import Enum
import time


class State(Enum):
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    ALIGN = "ALIGN"
    SHOOT = "SHOOT"
    RESET = "RESET"


class BoatFSM:
    def __init__(self, motor_controller, config):
        self.state = State.SEARCH
        self.motors = motor_controller
        self.config = config
        self.state_entered_at = time.time()
        self.last_ball_seen_at = None

    def _transition(self, new_state):
        if new_state != self.state:
            print(f"[FSM] {self.state.value} -> {new_state.value}")
            self.state = new_state
            self.state_entered_at = time.time()

    def time_in_state(self):
        return time.time() - self.state_entered_at

    def update(self, vision_result):
        """
        vision_result: {"red": {...}, "colored": {...}}, each shaped
            {"ball_found": bool, "x_offset": float, "distance_est": float}
        x_offset: -1.0 (far left) to 1.0 (far right), 0 = centered
        distance_est: rough proxy (e.g. ball radius in px, bigger = closer)

        Colored ball is worth 2x the red ball (more with full autonomy),
        so it's preferred whenever both are visible.
        """
        vision_result = self._select_target(vision_result)

        if self.state == State.SEARCH:
            self._do_search(vision_result)
        elif self.state == State.APPROACH:
            self._do_approach(vision_result)
        elif self.state == State.ALIGN:
            self._do_align(vision_result)
        elif self.state == State.SHOOT:
            self._do_shoot(vision_result)
        elif self.state == State.RESET:
            self._do_reset(vision_result)

    @staticmethod
    def _select_target(vision_result):
        """Prefer the colored ball (worth more) whenever it's visible."""
        if vision_result["colored"]["ball_found"]:
            return vision_result["colored"]
        return vision_result["red"]

    # ---- state behaviors ----

    def _do_search(self, v):
        if v["ball_found"]:
            self.last_ball_seen_at = time.time()
            self._transition(State.APPROACH)
            return
        # rotate slowly to scan; reverse sweep direction periodically
        self.motors.rotate_in_place(self.config.SEARCH_ROTATE_SPEED)

    def _do_approach(self, v):
        if not v["ball_found"]:
            # lost the ball — give it a short grace period before re-searching
            if time.time() - self.last_ball_seen_at > self.config.LOST_BALL_TIMEOUT:
                self._transition(State.SEARCH)
            return

        self.last_ball_seen_at = time.time()

        if v["distance_est"] > self.config.ALIGN_DISTANCE_THRESHOLD:
            self._transition(State.ALIGN)
            return

        # steer toward ball while moving forward
        self.motors.drive_toward(v["x_offset"], speed=self.config.APPROACH_SPEED)

    def _do_align(self, v):
        if not v["ball_found"]:
            self._transition(State.SEARCH)
            return

        if abs(v["x_offset"]) < self.config.ALIGN_TOLERANCE:
            self._transition(State.SHOOT)
            return

        self.motors.rotate_in_place(
            self.config.ALIGN_ROTATE_SPEED * (1 if v["x_offset"] > 0 else -1)
        )

    def _do_shoot(self, v):
        self.motors.shoot()
        self._transition(State.RESET)

    def _do_reset(self, v):
        self.motors.stop()
        if self.time_in_state() > self.config.RESET_PAUSE_SEC:
            self._transition(State.SEARCH)
PYEOF

cat > main_pi.py << 'PYEOF'
"""
Full autonomous pipeline for the Pi: Pi Camera Module -> BallVision
(HSV+Hough+CSRT, red + colored ball) -> BoatFSM -> real thrusters via
MotorController. This is the real entry point for the boat, as opposed
to main_face_demo.py which stands in a face detector for testing without
a physical ball.

Run on the Pi:
    python3 main_pi.py

Ctrl+C to stop — cleanly stops the thrusters and camera on exit.

SAFETY: drives real thrusters the moment a ball is detected. Keep props
clear / in water only, be ready to cut battery power — this does not
wire up the e-stop (see main.py's EStopMonitor, still a stub).
"""

import time

import config
from fsm import BoatFSM
from vision import BallVision
from motor_control import MotorController


class PiCameraGrabber:
    def __init__(self, width, height):
        from picamera2 import Picamera2

        self.picam2 = Picamera2()
        cam_config = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.picam2.configure(cam_config)
        self.picam2.start()
        time.sleep(1.0)  # let auto-exposure/white-balance settle before the first frame

    def get_frame(self):
        # picamera2's "RGB888" format is actually BGR byte order despite the
        # name — matches what OpenCV expects, no channel swap needed.
        return self.picam2.capture_array()

    def stop(self):
        self.picam2.stop()


def main():
    print("[main] Starting ball-chase pipeline. Ctrl+C to stop.")
    motors = MotorController(config)
    camera = PiCameraGrabber(config.FRAME_WIDTH, config.FRAME_HEIGHT)
    vision = BallVision(config)
    fsm = BoatFSM(motors, config)

    try:
        while True:
            frame = camera.get_frame()
            vision_result = vision.detect(frame)
            fsm.update(vision_result)

            red = vision_result["red"]
            colored = vision_result["colored"]
            print(f"[main] state={fsm.state.value:8s} "
                  f"red_found={red['ball_found']!s:5s} colored_found={colored['ball_found']!s:5s} "
                  f"x_offset={red['x_offset']:+.2f} dist={red['distance_est']:.0f}  ",
                  end="\r")

            time.sleep(0.02)  # ~50Hz control loop

    except KeyboardInterrupt:
        print("\n[main] Stopped by user.")
    finally:
        motors.stop()
        camera.stop()
        print("[main] Thrusters and camera stopped.")


if __name__ == "__main__":
    main()
PYEOF

cat > stream_ball_detection.py << 'PYEOF'
"""
Live video feed with bounding boxes around the detected red/colored ball —
camera + detection only, no thrusters. Use this to check detection
quality on the actual Pi camera/lighting before trusting main_pi.py to
drive real thrusters off it.

Run on the Pi:
    python3 stream_ball_detection.py

Then on your Mac, open a browser to:
    http://talosbot.local:8000

Ctrl+C on the Pi to stop.
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

import config
from vision import BallVision

PORT = 8000

latest_jpeg = None
latest_lock = threading.Lock()
running = True


class PiCameraGrabber:
    def __init__(self, width, height):
        from picamera2 import Picamera2

        self.picam2 = Picamera2()
        cam_config = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.picam2.configure(cam_config)
        self.picam2.start()
        time.sleep(1.0)

    def get_frame(self):
        return self.picam2.capture_array()

    def stop(self):
        self.picam2.stop()


def _draw_ball(frame, result, label, color):
    if not result["bbox"]:
        return
    x, y, w, h = result["bbox"]
    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
    cv2.putText(frame, f"{label} d={result['distance_est']:.0f}", (x, max(0, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


def capture_loop():
    global latest_jpeg
    camera = PiCameraGrabber(config.FRAME_WIDTH, config.FRAME_HEIGHT)
    vision = BallVision(config)

    try:
        while running:
            frame = camera.get_frame()
            result = vision.detect(frame)

            _draw_ball(frame, result["red"], "RED", (0, 0, 255))
            _draw_ball(frame, result["colored"], "COLORED", (0, 255, 0))

            status = f"red={result['red']['ball_found']} colored={result['colored']['ball_found']}"
            cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            ok, jpeg = cv2.imencode(".jpg", frame)
            if ok:
                with latest_lock:
                    latest_jpeg = jpeg.tobytes()

            time.sleep(0.03)
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
PYEOF

echo ""
cat > test_face_forward.py << 'PYEOF2'
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
            main={"size": (width, height), "format": "RGB888"}
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
PYEOF2

echo "Done. Next steps:"
echo "  cd ~/ire"
echo "  python3 test_thrusters.py         # bench test with PROPS OFF, if you haven't already"
echo "  python3 test_face_forward.py      # NEW: face visible -> both thrusters forward, else stop"
echo "  python3 stream_ball_detection.py  # ball detection ONLY, no motors, view at http://talosbot.local:8000"
echo "  python3 main_pi.py                # FULL real pipeline: camera -> ball detection -> FSM -> thrusters"
echo "  (face-detection stand-ins also still here: test_face_detection.py, stream_face_detection.py, main_face_demo.py)"
