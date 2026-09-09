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
