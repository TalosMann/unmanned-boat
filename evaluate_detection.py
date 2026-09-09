"""
Checks vision.py's red-ball detector against hand-labeled ground truth
(from label_ball.py) across a whole video. Runs the detector on every
frame in sequence (so CSRT tracking behaves as it would live), and scores
it only at the frames you labeled.

For each labeled frame, reports whether the detector found the ball, how
far off (px) it was from your click, and the ball's apparent speed
(px/frame, from consecutive labels) so you can see whether errors/misses
cluster at higher speeds.

Usage:
    python3 evaluate_detection.py video_3.mp4 video_3_labels.csv
    python3 evaluate_detection.py video_5.mp4 video_5_labels.csv --show
    python3 evaluate_detection.py video_3.mp4 video_3_labels.csv --dump-worst 5
"""

import argparse
import csv
import math
import os

import cv2

import config
from vision import _SingleBallTracker


def load_labels(path):
    labels = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            labels[int(row["frame_idx"])] = {
                "visible": bool(int(row["visible"])),
                "x": float(row["x"]),
                "y": float(row["y"]),
            }
    return labels


def main():
    parser = argparse.ArgumentParser(description="Evaluate red-ball detection against labeled ground truth.")
    parser.add_argument("video")
    parser.add_argument("labels_csv")
    parser.add_argument("--show", action="store_true", help="Display each labeled frame with GT (green) vs detection (red)")
    parser.add_argument("--dump-worst", type=int, default=0, metavar="N",
                         help="Save annotated images for the N highest-error frames, plus all false positives")
    args = parser.parse_args()

    labels = load_labels(args.labels_csv)
    if not labels:
        print(f"[eval] No labels found in {args.labels_csv}")
        return

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"[eval] Could not open video: {args.video}")
        return

    red = _SingleBallTracker(
        hsv_ranges=[
            (config.RED_HSV_LOWER1, config.RED_HSV_UPPER1),
            (config.RED_HSV_LOWER2, config.RED_HSV_UPPER2),
        ],
        min_radius_px=config.MIN_BALL_RADIUS_PX,
        min_color_match_fraction=config.MIN_COLOR_MATCH_FRACTION,
    )

    last_gt = None  # (frame_idx, x, y) of previous *visible* label, for speed calc
    rows = []

    frame_idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        result = red.detect(frame)

        if frame_idx in labels:
            gt = labels[frame_idx]
            speed_px_per_frame = None
            if gt["visible"] and last_gt is not None:
                dframes = frame_idx - last_gt[0]
                if dframes > 0:
                    speed_px_per_frame = math.hypot(gt["x"] - last_gt[1], gt["y"] - last_gt[2]) / dframes

            error_px = None
            if gt["visible"] and result["ball_found"]:
                bbox = result["bbox"]
                cx = bbox[0] + bbox[2] / 2
                cy = bbox[1] + bbox[3] / 2
                error_px = math.hypot(cx - gt["x"], cy - gt["y"])

            annotated = None
            if args.show or args.dump_worst:
                annotated = frame.copy()
                if gt["visible"]:
                    cv2.drawMarker(annotated, (int(gt["x"]), int(gt["y"])), (0, 255, 0),
                                    markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
                if result["ball_found"]:
                    bbox = result["bbox"]
                    cx, cy = int(bbox[0] + bbox[2] / 2), int(bbox[1] + bbox[3] / 2)
                    cv2.drawMarker(annotated, (cx, cy), (0, 0, 255),
                                    markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

            rows.append({
                "frame_idx": frame_idx,
                "gt_visible": gt["visible"],
                "detected": result["ball_found"],
                "error_px": error_px,
                "speed_px_per_frame": speed_px_per_frame,
                "annotated": annotated,
            })

            if args.show:
                cv2.imshow("eval — green=labeled, red=detected", annotated)
                cv2.waitKey(0)

            if gt["visible"]:
                last_gt = (frame_idx, gt["x"], gt["y"])

        frame_idx += 1

    cap.release()
    if args.show:
        cv2.destroyAllWindows()
        for _ in range(4):
            cv2.waitKey(1)

    _report(rows, args.video, args.dump_worst)


def _report(rows, video_path, dump_worst):
    visible_rows = [r for r in rows if r["gt_visible"]]
    not_visible_rows = [r for r in rows if not r["gt_visible"]]

    hits = [r for r in visible_rows if r["detected"]]
    misses = [r for r in visible_rows if not r["detected"]]
    false_positives = [r for r in not_visible_rows if r["detected"]]

    print(f"\n[eval] Labeled frames: {len(rows)} "
          f"({len(visible_rows)} ball-visible, {len(not_visible_rows)} not-visible)")
    print(f"[eval] Hit rate on visible frames: {len(hits)}/{len(visible_rows)} "
          f"({100 * len(hits) / max(1, len(visible_rows)):.0f}%)")
    if false_positives:
        print(f"[eval] False positives on not-visible frames: {len(false_positives)}/{len(not_visible_rows)}")

    errors = [r["error_px"] for r in hits if r["error_px"] is not None]
    if errors:
        errors_sorted = sorted(errors)
        mean_err = sum(errors) / len(errors)
        median_err = errors_sorted[len(errors_sorted) // 2]
        print(f"[eval] Position error on hits: mean={mean_err:.1f}px  median={median_err:.1f}px  "
              f"max={max(errors):.1f}px")

    # Bucket by speed to see if errors/misses cluster at higher speeds
    speed_rows = [r for r in visible_rows if r["speed_px_per_frame"] is not None]
    if speed_rows:
        speeds_sorted = sorted(r["speed_px_per_frame"] for r in speed_rows)
        median_speed = speeds_sorted[len(speeds_sorted) // 2]
        slow = [r for r in speed_rows if r["speed_px_per_frame"] <= median_speed]
        fast = [r for r in speed_rows if r["speed_px_per_frame"] > median_speed]

        def bucket_stats(bucket):
            n = len(bucket)
            hit_n = sum(1 for r in bucket if r["detected"])
            errs = [r["error_px"] for r in bucket if r["detected"] and r["error_px"] is not None]
            mean_e = sum(errs) / len(errs) if errs else None
            return n, hit_n, mean_e

        n, hit_n, mean_e = bucket_stats(slow)
        print(f"[eval] Slow half (<= {median_speed:.1f} px/frame): "
              f"hit {hit_n}/{n}" + (f", mean error {mean_e:.1f}px" if mean_e is not None else ""))
        n, hit_n, mean_e = bucket_stats(fast)
        print(f"[eval] Fast half (>  {median_speed:.1f} px/frame): "
              f"hit {hit_n}/{n}" + (f", mean error {mean_e:.1f}px" if mean_e is not None else ""))

    if misses:
        print(f"[eval] Missed frames: {[r['frame_idx'] for r in misses]}")

    if dump_worst:
        _dump_worst(rows, video_path, dump_worst, hits, misses, false_positives)


def _dump_worst(rows, video_path, n, hits, misses, false_positives):
    ranked = sorted((r for r in hits if r["error_px"] is not None),
                     key=lambda r: r["error_px"], reverse=True)[:n]
    to_dump = {r["frame_idx"]: (r, "error") for r in ranked}
    for r in misses:
        to_dump[r["frame_idx"]] = (r, "miss")
    for r in false_positives:
        to_dump[r["frame_idx"]] = (r, "false_positive")

    if not to_dump:
        print("[eval] Nothing to dump.")
        return

    out_dir = f"{os.path.splitext(video_path)[0]}_worst_frames"
    os.makedirs(out_dir, exist_ok=True)

    for frame_idx, (r, kind) in sorted(to_dump.items()):
        if r["annotated"] is None:
            continue
        err_tag = f"_{r['error_px']:.0f}px" if r["error_px"] is not None else ""
        out_path = os.path.join(out_dir, f"frame{frame_idx:06d}_{kind}{err_tag}.png")
        cv2.imwrite(out_path, r["annotated"])

    print(f"[eval] Dumped {len(to_dump)} frames to {out_dir}/ "
          "(green marker = your label, red marker = detection)")


if __name__ == "__main__":
    main()
