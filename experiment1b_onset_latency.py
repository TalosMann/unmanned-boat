"""
Experiment 1B — Detection Onset Latency (a "reaction time" metric,
distinct from Experiment 1's per-call compute latency).

Experiment 1 (experiment1_latency.py) measures how long a SINGLE call to
vision.BallVision.detect() takes. That's a compute-cost metric. It does
NOT measure how many frames pass between the ball genuinely becoming
visible (per human ground truth) and the detector first agreeing —
which can take several frames even when each individual detect() call
is fast, e.g. because Hough Circles needs enough of the ball's silhouette
visible, or CSRT needs to re-acquire after occlusion. This script
measures that onset lag instead.

Method: for every ground-truth "appearance event" (a labeled frame goes
from not-visible to visible, or the ball reappears after being hidden),
find the first frame at or after that point where the live detector also
reports ball_found=True, and report the delay in frames/milliseconds.

IMPORTANT — label density: this needs the ground truth to be exact about
WHEN the ball became visible, not just sampled every N frames like the
Experiment 2 labels. Label short "appearance event" clips with
    python3 label_ball.py clip.mp4 --every 1
(every frame, at least through the appearance transition) rather than
the sparser --every 5 used for Experiment 2's condition comparison.

No human timing involved — this is entirely automated once ground truth
exists, same as Experiments 1 and 2.

Usage:
    python3 experiment1b_onset_latency.py clip1.mp4 clip1_labels.csv [clip2.mp4 clip2_labels.csv ...]
    python3 experiment1b_onset_latency.py clip1.mp4 clip1_labels.csv --out onset_events.csv
"""

import argparse
import csv

from evaluate_detection import load_labels, run_detection


def find_appearance_events(labels):
    """Returns sorted frame indices where a labeled frame is visible=1 and
    the immediately-preceding LABELED frame was visible=0 — i.e. moments
    where ground truth says the ball just became visible (from off-frame,
    or after being occluded)."""
    frame_indices = sorted(labels.keys())
    events = []
    prev_visible = None
    for idx in frame_indices:
        visible = labels[idx]["visible"]
        if visible and prev_visible is False:
            events.append(idx)
        prev_visible = visible
    return events


def onset_delay_for_event(event_frame, all_frames_by_idx, max_search_frames=150):
    """Scans forward from event_frame through the (dense, every-frame)
    detection results to find the first frame the detector reports
    ball_found=True. Returns the frame delta, or None if never found
    within max_search_frames."""
    for delta in range(max_search_frames):
        idx = event_frame + delta
        if idx not in all_frames_by_idx:
            continue
        if all_frames_by_idx[idx]["detected"]:
            return delta
    return None


def main():
    parser = argparse.ArgumentParser(
        description="Measure detection onset latency (frames between ground-truth ball "
                    "appearance and the detector first agreeing) across one or more clips."
    )
    parser.add_argument("clips", nargs="+", help="video1.mp4 labels1.csv [video2.mp4 labels2.csv ...]")
    parser.add_argument("--out", default=None, help="Optional path to save per-event results as CSV")
    args = parser.parse_args()

    if len(args.clips) < 2 or len(args.clips) % 2 != 0:
        print("Usage: python3 experiment1b_onset_latency.py video1.mp4 labels1.csv [video2.mp4 labels2.csv ...]")
        return

    all_events = []

    for i in range(0, len(args.clips), 2):
        video_path, labels_path = args.clips[i], args.clips[i + 1]
        labels = load_labels(labels_path)
        if not labels:
            print(f"[exp1b] No labels found in {labels_path}, skipping.")
            continue

        # Sanity check: warn if labels look sparse (every-N rather than
        # every-frame), since onset-frame precision depends on label density.
        frame_indices = sorted(labels.keys())
        if len(frame_indices) > 1:
            gaps = [b - a for a, b in zip(frame_indices, frame_indices[1:])]
            typical_gap = sorted(gaps)[len(gaps) // 2]
            if typical_gap > 1:
                print(f"[exp1b] WARNING: {labels_path} looks sparsely labeled "
                      f"(typical gap {typical_gap} frames) — onset timing will only be "
                      f"accurate to within that gap. Re-label with --every 1 for precise "
                      f"onset measurement.")

        print(f"[exp1b] Processing {video_path}...")
        try:
            _, all_frames = run_detection(video_path, labels, record_all_frames=True)
        except RuntimeError as e:
            print(f"[exp1b]   {e}, skipping.")
            continue

        all_frames_by_idx = {f["frame_idx"]: f for f in all_frames}
        events = find_appearance_events(labels)

        if not events:
            print(f"[exp1b]   No appearance events found in {labels_path} "
                  f"(needs at least one visible=0 -> visible=1 transition).")
            continue

        for event_frame in events:
            delta_frames = onset_delay_for_event(event_frame, all_frames_by_idx)
            all_events.append({
                "video": video_path,
                "event_frame": event_frame,
                "onset_delay_frames": delta_frames if delta_frames is not None else "",
                "detected": delta_frames is not None,
            })

    if not all_events:
        print("[exp1b] No appearance events found across any clip.")
        return

    found = [e for e in all_events if e["detected"]]
    print(f"\n[exp1b] {len(all_events)} appearance event(s) found, "
          f"{len(found)} eventually detected within the search window.")

    if found:
        deltas = [e["onset_delay_frames"] for e in found]
        mean_delay = sum(deltas) / len(deltas)
        median_delay = sorted(deltas)[len(deltas) // 2]
        print(f"[exp1b] Onset delay: mean={mean_delay:.1f} frames  "
              f"median={median_delay} frames  min={min(deltas)}  max={max(deltas)}")
        print("[exp1b] (Multiply by the video's frame interval, e.g. 1/30s at 30fps, "
              "to convert to milliseconds — frame rate can differ per clip so this is "
              "left in frames rather than assuming one fps for all clips.)")

    never_found = [e for e in all_events if not e["detected"]]
    if never_found:
        print(f"[exp1b] {len(never_found)} event(s) were NEVER detected within the search "
              f"window — worth inspecting these specifically, they may indicate a real "
              f"detection failure rather than just latency.")

    if args.out:
        with open(args.out, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(all_events[0].keys()))
            writer.writeheader()
            writer.writerows(all_events)
        print(f"\n[exp1b] Saved per-event results to {args.out}")


if __name__ == "__main__":
    main()
