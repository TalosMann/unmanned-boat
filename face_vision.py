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

# Debian's python3-opencv package doesn't ship cascade XML files at all (no
# cv2.data submodule, and none on disk under /usr either) — unlike pip's
# opencv-python, which bundles them. haarcascade_frontalface_default.xml
# next to this script is our own copy, checked in for exactly this case.
_CASCADE_SEARCH_PATHS = [
    lambda: os.path.dirname(os.path.abspath(__file__)),
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
