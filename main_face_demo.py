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
