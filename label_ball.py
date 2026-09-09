"""
Manual ball-position labeling tool. Steps through a video every N frames,
you click the ball's center on each one, and it saves ground-truth
(frame_idx, x, y) to a CSV. Use this to build labeled data from one video,
then check vision.py's detector against it (and against a second, unseen
video) to see how detection accuracy holds up across ball speeds.

Usage:
    python3 label_ball.py path/to/video.mp4
    python3 label_ball.py path/to/video.mp4 --every 20 --out labels_video1.csv

Controls (per shown frame):
    left-click   mark/move the ball center for this frame
    n / space    confirm this frame's label and advance to the next one
    s            no ball visible in this frame — mark as not-visible, advance
    u            undo the last confirmed label, step back to it
    q / ESC      quit and save everything confirmed so far

Output CSV columns: frame_idx, timestamp_sec, x, y, visible
(x, y are -1 and visible is 0 for frames marked not-visible with 's')
"""

import argparse
import csv
import os

import cv2


def main():
    parser = argparse.ArgumentParser(description="Label ball position every N frames of a video.")
    parser.add_argument("video", help="Path to video file")
    parser.add_argument("--every", type=int, default=100, help="Label every Nth frame (default: 100)")
    parser.add_argument("--out", default=None, help="Output CSV path (default: <video>_labels.csv)")
    args = parser.parse_args()

    out_path = args.out or f"{os.path.splitext(args.video)[0]}_labels.csv"

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"[label] Could not open video file: {args.video}")
        return

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    window = "Label ball — click center, n=confirm, s=not visible, u=undo, q=quit"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    labels = []  # list of dicts, one per confirmed frame, in order
    click_point = [None]  # mutable box so the mouse callback can set it

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click_point[0] = (x, y)

    cv2.setMouseCallback(window, on_mouse)

    frame_idx = 0
    print(f"[label] {args.video}: {total_frames} frames @ {fps:.1f}fps, "
          f"labeling every {args.every} frames -> {out_path}")

    while True:
        if frame_idx >= total_frames:
            print("[label] Reached end of video.")
            break

        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ok, frame = cap.read()
        if not ok:
            print("[label] Could not read frame, stopping.")
            break

        click_point[0] = None
        confirmed = False

        while not confirmed:
            display = frame.copy()
            if click_point[0] is not None:
                cv2.drawMarker(display, click_point[0], (0, 255, 0),
                                markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
            status = f"frame {frame_idx}/{total_frames}  labeled {len(labels)}"
            status += "  [click ball, n=confirm, s=not visible, u=undo, q=quit]"
            cv2.putText(display, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow(window, display)

            key = cv2.waitKey(30) & 0xFF

            if key in (ord("q"), 27):
                cap.release()
                cv2.destroyAllWindows()
                for _ in range(4):
                    cv2.waitKey(1)
                _save(out_path, labels)
                return

            if key in (ord("n"), ord(" ")):
                if click_point[0] is None:
                    print("[label] Click the ball first, or press 's' if it's not visible.")
                    continue
                x, y = click_point[0]
                labels.append({"frame_idx": frame_idx, "timestamp_sec": frame_idx / fps,
                                "x": x, "y": y, "visible": 1})
                confirmed = True

            elif key == ord("s"):
                labels.append({"frame_idx": frame_idx, "timestamp_sec": frame_idx / fps,
                                "x": -1, "y": -1, "visible": 0})
                confirmed = True

            elif key == ord("u"):
                if labels:
                    removed = labels.pop()
                    frame_idx = removed["frame_idx"] - args.every
                    print(f"[label] Undid frame {removed['frame_idx']}, back to {frame_idx}")
                else:
                    frame_idx -= args.every
                    print("[label] Nothing to undo, stepping back one interval.")
                confirmed = True
                frame_idx = max(0, frame_idx)
                break  # re-enter outer loop at the rewound frame_idx

            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                cap.release()
                cv2.destroyAllWindows()
                _save(out_path, labels)
                return

        if key == ord("u"):
            continue  # frame_idx was already rewound above

        frame_idx += args.every

    cap.release()
    cv2.destroyAllWindows()
    for _ in range(4):
        cv2.waitKey(1)
    _save(out_path, labels)


def _save(out_path, labels):
    if not labels:
        print("[label] No labels to save.")
        return
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["frame_idx", "timestamp_sec", "x", "y", "visible"])
        writer.writeheader()
        writer.writerows(labels)
    print(f"[label] Saved {len(labels)} labels to {out_path}")


if __name__ == "__main__":
    main()
