"""
Derives an HSV range for the red ball directly from label_ball.py's
ground-truth labels, instead of manually clicking through calibrate_hsv.py.
For every labeled visible frame, samples the actual pixel colors in a
small patch around the clicked (x, y) and aggregates the HSV stats.

Usage:
    python3 auto_calibrate_hsv.py video_3.mp4 video_3_labels.csv video_5.mp4 video_5_labels.csv

Accepts any number of (video, labels_csv) pairs — pass every labeled
video you have so the range covers lighting variation across all of them.
Prints a suggested RED_HSV_LOWER/UPPER for config.py; does not edit any
files itself.
"""

import csv
import sys

import cv2
import numpy as np


def load_labels(path):
    labels = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            if int(row["visible"]):
                labels[int(row["frame_idx"])] = (float(row["x"]), float(row["y"]))
    return labels


def sample_video(video_path, labels, patch_radius=8):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[calibrate] Could not open {video_path}")
        return []

    samples = []
    frame_idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame_idx in labels:
            x, y = labels[frame_idx]
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            h_frame, w_frame = hsv.shape[:2]
            x0, y0 = max(0, int(x - patch_radius)), max(0, int(y - patch_radius))
            x1, y1 = min(w_frame, int(x + patch_radius)), min(h_frame, int(y + patch_radius))
            patch = hsv[y0:y1, x0:x1].reshape(-1, 3)
            samples.append(patch)
        frame_idx += 1

    cap.release()
    return samples


def main():
    args = sys.argv[1:]
    if len(args) < 2 or len(args) % 2 != 0:
        print("Usage: python3 auto_calibrate_hsv.py video1.mp4 labels1.csv [video2.mp4 labels2.csv ...]")
        return

    all_samples = []
    for i in range(0, len(args), 2):
        video_path, labels_path = args[i], args[i + 1]
        labels = load_labels(labels_path)
        print(f"[calibrate] Sampling {len(labels)} labeled frames from {video_path}...")
        all_samples.extend(sample_video(video_path, labels))

    if not all_samples:
        print("[calibrate] No samples collected.")
        return

    pixels = np.vstack(all_samples)
    h, s, v = pixels[:, 0], pixels[:, 1], pixels[:, 2]

    print(f"\n[calibrate] {len(pixels)} pixels sampled across all videos.")
    print(f"  H: min={h.min()} p5={np.percentile(h, 5):.0f} median={np.median(h):.0f} "
          f"p95={np.percentile(h, 95):.0f} max={h.max()}")
    print(f"  S: min={s.min()} p5={np.percentile(s, 5):.0f} median={np.median(s):.0f} max={s.max()}")
    print(f"  V: min={v.min()} p5={np.percentile(v, 5):.0f} median={np.median(v):.0f} max={v.max()}")

    # Red wraps the hue circle (0/180) — split low-hue vs high-hue samples
    # and report two ranges the same way config.py's RED_HSV_LOWER/UPPER1/2 do.
    low_hue = h[h < 90]
    high_hue = h[h >= 90]

    s_lo = max(0, np.percentile(s, 5) - 20)
    v_lo = max(0, np.percentile(v, 5) - 20)

    print("\n[calibrate] Suggested config.py values (5th/95th percentile bounds, padded):")
    if len(low_hue) > 0:
        h1_lo = max(0, np.percentile(low_hue, 5) - 5)
        h1_hi = min(180, np.percentile(low_hue, 95) + 5)
        print(f"  RED_HSV_LOWER1 = ({h1_lo:.0f}, {s_lo:.0f}, {v_lo:.0f})")
        print(f"  RED_HSV_UPPER1 = ({h1_hi:.0f}, 255, 255)")
    if len(high_hue) > 0:
        h2_lo = max(0, np.percentile(high_hue, 5) - 5)
        h2_hi = min(180, np.percentile(high_hue, 95) + 5)
        print(f"  RED_HSV_LOWER2 = ({h2_lo:.0f}, {s_lo:.0f}, {v_lo:.0f})")
        print(f"  RED_HSV_UPPER2 = ({h2_hi:.0f}, 255, 255)")

    print("\n[calibrate] These videos were shot indoors on a different camera/lens than the "
          "Pi's — treat this as a starting point, not final. Re-run evaluate_detection.py "
          "after updating config.py, and re-calibrate against real Pi-camera footage of the "
          "ball once you have it.")


if __name__ == "__main__":
    main()
