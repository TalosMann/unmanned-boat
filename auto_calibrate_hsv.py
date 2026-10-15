"""
Derives HSV detection ranges directly from label_ball.py's ground-truth
labels, instead of manually clicking through calibrate_hsv.py. For every
labeled visible frame, samples the actual pixel colors in a small patch
around the clicked (x, y) and clusters them by hue.

Handles both a single-color ball needing hue-wrap (true red sits at both
ends of OpenCV's 0-180 hue range, so it needs two ranges OR'd together)
and a multi-color practice ball (e.g. a red/yellow/green ball, used when
a real red ball isn't available) — clustering naturally finds as many
hue clusters as the ball actually shows across the labeled frames (one
per color patch, as the ball rotates and different faces face the
camera), rather than assuming exactly one wrap-around pair.

Usage:
    python3 auto_calibrate_hsv.py video1.mp4 labels1.csv [video2.mp4 labels2.csv ...]

Accepts any number of (video, labels_csv) pairs — pass every labeled
video you have so the range covers lighting variation across all of them.
Prints a suggested RED_HSV_RANGES list for config.py; does not edit any
files itself.
"""

import csv
import sys

import cv2
import numpy as np

N_HUE_BINS = 180  # one bin per integer hue value (OpenCV hue range is 0-179)


def load_labels(path):
    labels = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            if int(row["visible"]):
                labels[int(row["frame_idx"])] = (float(row["x"]), float(row["y"]))
    return labels


def sample_video(video_path, labels, patch_radius=3):
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


def find_hue_clusters(hues, min_bin_count_frac=0.02):
    """Groups hue samples into contiguous clusters on the circular 0-179
    hue range, handling wraparound (a cluster can straddle the 179/0
    boundary, as true red's does). Returns a list of (lo, hi, wraps) —
    wraps=True means the cluster spans through the 179/0 boundary and
    should be emitted as two OpenCV ranges, like red already was."""
    counts = np.zeros(N_HUE_BINS, dtype=int)
    for h in hues:
        counts[int(h) % N_HUE_BINS] += 1

    # Real photographed footage (JPEG compression, lighting/reflection
    # variation) puts small noisy dips in an otherwise-continuous color
    # band, which would otherwise fragment one true color patch (e.g. a
    # yellow-to-green gradient) into several small overlapping clusters.
    # Smooth circularly before thresholding so those dips don't split a
    # single real cluster apart.
    smooth_window = 3
    padded = np.concatenate([counts[-smooth_window:], counts, counts[:smooth_window]])
    kernel = np.ones(smooth_window) / smooth_window
    smoothed_padded = np.convolve(padded, kernel, mode="same")
    smoothed = smoothed_padded[smooth_window:smooth_window + N_HUE_BINS]

    threshold = max(1, smoothed.max() * min_bin_count_frac)
    active = smoothed >= threshold

    visited = [False] * N_HUE_BINS
    clusters = []  # list of lists of bin indices, each contiguous (circularly)

    for start in range(N_HUE_BINS):
        if active[start] and not visited[start]:
            bins = []
            i = start
            while active[i] and not visited[i]:
                visited[i] = True
                bins.append(i)
                i = (i + 1) % N_HUE_BINS
                if i == start:  # wrapped all the way around (entire circle active)
                    break
            clusters.append(bins)

    # A cluster that reaches bin 0 and one that reaches bin 179 are
    # actually one cluster split by where the scan happened to start —
    # merge them if both exist.
    if len(clusters) > 1 and clusters[0][0] == 0 and clusters[-1][-1] == N_HUE_BINS - 1:
        clusters[0] = clusters[-1] + clusters[0]
        clusters.pop()

    # Split each cluster into simple (non-wrapping) sub-intervals — a
    # cluster that wraps past 179 back to 0 becomes two.
    raw_intervals = []
    for bins in clusters:
        wraps = len(bins) > 1 and bins[-1] < bins[0]
        if wraps:
            split = next(i for i in range(1, len(bins)) if bins[i] < bins[i - 1])
            raw_intervals.append((bins[0], bins[split - 1]))
            raw_intervals.append((bins[split], bins[-1]))
        else:
            raw_intervals.append((bins[0], bins[-1]))

    return raw_intervals


def pad_and_merge_intervals(intervals, pad=5, hi_max=179):
    """Pads each (lo, hi) interval by `pad` on each side, then merges any
    that now overlap or touch — padding two originally-separate-but-close
    clusters can easily make them overlap, and printing them as separate
    redundant ranges would just be noise."""
    padded = sorted((max(0, lo - pad), min(hi_max, hi + pad)) for lo, hi in intervals)
    merged = []
    for lo, hi in padded:
        if merged and lo <= merged[-1][1] + 1:
            merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
        else:
            merged.append((lo, hi))
    return merged


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

    s_lo = max(0, np.percentile(s, 5) - 20)
    v_lo = max(0, np.percentile(v, 5) - 20)

    raw_intervals = find_hue_clusters(h)
    merged = pad_and_merge_intervals(raw_intervals)
    print(f"\n[calibrate] Found {len(raw_intervals)} raw hue cluster(s), "
          f"merged into {len(merged)} range(s) after padding — one range "
          f"per distinct color, OR'd together (a single-color ball "
          f"typically yields 1-2, a multi-color practice ball yields one "
          f"per visible color).")

    print("\n[calibrate] Suggested config.py value:")
    print("RED_HSV_RANGES = [")
    for lo, hi in merged:
        print(f"    (({lo:.0f}, {s_lo:.0f}, {v_lo:.0f}), ({hi:.0f}, 255, 255)),")
    print("]")

    print("\n[calibrate] These videos may have been shot on a different camera/lens than the "
          "Pi's — treat this as a starting point, not final. Re-run evaluate_detection.py "
          "after updating config.py, and re-calibrate against real Pi-camera footage of the "
          "actual ball being used once you have it.")


if __name__ == "__main__":
    main()
