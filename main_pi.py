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
            main={"size": (width, height), "format": "RGB888"},
            buffer_count=2,  # low buffering so we always process the freshest
                             # frame instead of a growing backlog when
                             # detection can't keep up with capture rate
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
