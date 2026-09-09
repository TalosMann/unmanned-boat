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
