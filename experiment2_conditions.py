"""
Experiment 2 — Detection Reliability Under Real Aquatic Conditions.

Runs evaluate_detection.py's detector-vs-ground-truth comparison across
several labeled video clips at once — one per condition (glare, wake,
occlusion, speed, etc.) — and prints/saves a single comparison table
matching the format in experiment_design.pdf, instead of manually
re-running evaluate_detection.py per clip and copying numbers by hand.

Ground truth still requires one manual pass with label_ball.py per clip
(the boat has no independent sensor to confirm ball position on its
own) — everything after that, including this comparison table, is
fully automated.

Usage:
    Create a manifest CSV with columns: condition,video,labels_csv
    e.g.:
        condition,video,labels_csv
        Calm water shaded,clips/calm.mp4,clips/calm_labels.csv
        Direct sun glare,clips/glare.mp4,clips/glare_labels.csv
        Boat wake,clips/wake.mp4,clips/wake_labels.csv

    python3 experiment2_conditions.py manifest.csv
    python3 experiment2_conditions.py manifest.csv --out results.csv
"""

import argparse
import csv

from evaluate_detection import load_labels, run_detection, compute_metrics


def load_manifest(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def main():
    parser = argparse.ArgumentParser(
        description="Compare detection reliability across multiple labeled condition clips."
    )
    parser.add_argument("manifest_csv", help="CSV with columns: condition,video,labels_csv")
    parser.add_argument("--out", default=None, help="Optional path to save the results as CSV")
    args = parser.parse_args()

    manifest = load_manifest(args.manifest_csv)
    if not manifest:
        print(f"[exp2] No rows found in {args.manifest_csv}")
        return

    results = []
    for entry in manifest:
        condition = entry["condition"]
        video_path = entry["video"]
        labels_path = entry["labels_csv"]

        print(f"[exp2] Running {condition!r} ({video_path})...")
        labels = load_labels(labels_path)
        if not labels:
            print(f"[exp2]   No labels found in {labels_path}, skipping.")
            continue

        try:
            rows, _ = run_detection(video_path, labels)
        except RuntimeError as e:
            print(f"[exp2]   {e}, skipping.")
            continue

        metrics = compute_metrics(rows)
        results.append({
            "condition": condition,
            "n_labeled_frames": metrics["n_labeled"],
            "hit_rate_pct": round(metrics["hit_rate_pct"], 1),
            "false_positive_rate_pct": round(metrics["false_positive_rate_pct"], 1),
            "mean_error_px": round(metrics["mean_error_px"], 1) if metrics["mean_error_px"] is not None else "",
        })

    if not results:
        print("[exp2] No results to report.")
        return

    print("\n[exp2] Condition comparison:\n")
    header = f"{'Condition':<28} {'Hit Rate %':>10} {'False Pos %':>12} {'Mean Err (px)':>14}"
    print(header)
    print("-" * len(header))
    for r in results:
        print(f"{r['condition']:<28} {r['hit_rate_pct']:>10} {r['false_positive_rate_pct']:>12} {r['mean_error_px']:>14}")

    if args.out:
        with open(args.out, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(results[0].keys()))
            writer.writeheader()
            writer.writerows(results)
        print(f"\n[exp2] Saved to {args.out}")


if __name__ == "__main__":
    main()
