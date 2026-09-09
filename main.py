"""
Entry point. Run this on the Pi (ideally via systemd on boot).
    python3 main.py

Structure:
  - e-stop runs on its own thread so it's never blocked by vision/control work
  - camera capture runs on its own thread (producer) so OpenCV's blocking
    read() doesn't stall the control loop
  - main loop: get latest frame -> vision.py finds ball -> fsm.py decides -> motors act
"""

import threading
import time
import sys

import cv2

import config
from fsm import BoatFSM
from vision import BallVision
from motor_control import MockMotorController  # noqa: F401 — swap the import+call below to MotorController on real hardware
# from motor_control import MotorController

DEBUG_DISPLAY = True  # set False on the Pi / headless runs


class FrameGrabber:
    """Runs camera capture on its own thread so the main loop is never
    blocked waiting on cv2.VideoCapture.read()."""

    def __init__(self, camera_index, width, height):
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.latest_frame = None
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _loop(self):
        while self.running:
            ok, frame = self.cap.read()
            if ok:
                with self.lock:
                    self.latest_frame = frame

    def get_frame(self):
        with self.lock:
            return None if self.latest_frame is None else self.latest_frame.copy()

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()


class EStopMonitor:
    """Polls the hardware e-stop independently of the main loop.
    Required by rulebook §2.2 / §4.4 — must cut power regardless of
    what the vision/control loop is doing."""

    def __init__(self, motors, pin, poll_hz):
        self.motors = motors
        self.pin = pin
        self.poll_interval = 1.0 / poll_hz
        self.triggered = False
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _loop(self):
        # TODO: replace with actual GPIO read, e.g. RPi.GPIO or gpiozero
        while self.running:
            if self._read_estop_pin():
                self.triggered = True
                self.motors.emergency_stop()
            time.sleep(self.poll_interval)

    def _read_estop_pin(self):
        # placeholder — wire up to real GPIO
        return False

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)


def main():
    # --- init hardware/software components ---
    motors = MockMotorController()  # swap to MotorController(config) on real hardware
    estop = EStopMonitor(motors, config.ESTOP_GPIO_PIN, config.ESTOP_POLL_HZ).start()
    grabber = FrameGrabber(config.CAMERA_INDEX, config.FRAME_WIDTH, config.FRAME_HEIGHT).start()
    vision = BallVision(config)
    fsm = BoatFSM(motors, config)

    window_name = "Boat Vision Debug"
    if DEBUG_DISPLAY:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    print("[main] Boat autonomy loop starting... "
          "(press 'q' or ESC in the video window, or close the window, to quit)")

    try:
        while True:
            if estop.triggered:
                print("[main] E-STOP triggered — halting loop.")
                break

            frame = grabber.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            vision_result = vision.detect(frame)
            fsm.update(vision_result)

            if DEBUG_DISPLAY:
                _draw_debug_overlay(frame, vision_result, fsm.state.value)
                cv2.imshow(window_name, frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):  # 'q' or ESC
                    break
                # user clicked the window's close button
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break

            time.sleep(0.02)  # ~50Hz control loop; tune to your actual processing time

    except KeyboardInterrupt:
        print("[main] Interrupted by user.")
    finally:
        grabber.stop()
        estop.stop()
        motors.stop()
        if DEBUG_DISPLAY:
            cv2.destroyAllWindows()
            # macOS/Qt backends can leave the window hanging around unless
            # the event loop gets pumped a few more times after destroy.
            for _ in range(4):
                cv2.waitKey(1)
        print("[main] Shutdown complete.")


def _draw_ball(frame, ball_result, label, color):
    if not ball_result["bbox"]:
        return
    x, y, w, h = ball_result["bbox"]
    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
    cv2.putText(
        frame,
        f"{label} offset={ball_result['x_offset']:.2f} dist={ball_result['distance_est']:.0f}",
        (x, max(0, y - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        1,
    )


def _draw_debug_overlay(frame, vision_result, state_label):
    _draw_ball(frame, vision_result["red"], "RED", (0, 0, 255))
    _draw_ball(frame, vision_result["colored"], "COLORED", (0, 255, 0))
    cv2.putText(
        frame, f"STATE: {state_label}", (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2,
    )


if __name__ == "__main__":
    sys.exit(main())
