"""
On-site HSV calibration helper. Not a training script — no model, no
learning. It just reads the HSV value under your cursor so you can pick
numeric thresholds for config.py by hand.

Live camera:
    python3 calibrate_hsv.py

Recorded video (e.g. footage of the ball handled on the water):
    python3 calibrate_hsv.py path/to/video.mp4

Controls:
    space       pause / resume (video mode only)
    n           step one frame forward while paused (video mode only)
    left-click  sample HSV at that pixel (works live or while paused)
    q / ESC     quit

Sample several points across each ball's surface — center, edge, in
shadow, in glare/sun reflection — across several different paused frames
if using a video, since lighting/angle changes as the ball moves. The
wider your samples, the tighter and more accurate a range you can set
without also catching background objects (clothing, skin, water glare).
"""

import sys

import cv2
import config

clicked_hsv = []


def _print_summary():
    if not clicked_hsv:
        return
    hs = [p[0] for p in clicked_hsv]
    ss = [p[1] for p in clicked_hsv]
    vs = [p[2] for p in clicked_hsv]
    print(f"  -> range so far ({len(clicked_hsv)} samples): "
          f"H[{min(hs)}-{max(hs)}] S[{min(ss)}-{max(ss)}] V[{min(vs)}-{max(vs)}] "
          f"(pad a few units each side before using)")


def _on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_frame = param
        if y >= hsv_frame.shape[0] or x >= hsv_frame.shape[1]:
            return
        h, s, v = hsv_frame[y, x]
        clicked_hsv.append((int(h), int(s), int(v)))
        print(f"HSV at ({x},{y}): H={h} S={s} V={v}")
        _print_summary()


def main():
    video_path = sys.argv[1] if len(sys.argv) > 1 else None
    is_video_file = video_path is not None

    if is_video_file:
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"[calibrate] Could not open video file: {video_path}")
            return
    else:
        cap = cv2.VideoCapture(config.CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)

    window = "HSV Calibration — click the ball, 'q'/ESC to quit"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    if is_video_file:
        print(f"[calibrate] Playing {video_path}. SPACE=pause/resume, "
              "N=step one frame while paused, click to sample, q/ESC to quit.")
    else:
        print("[calibrate] Live camera. Click points on the ball's surface "
              "(center, edge, shaded side, glare spot), q/ESC to quit.")

    paused = False
    frame = None

    try:
        while True:
            if not paused or frame is None:
                ok, new_frame = cap.read()
                if not ok:
                    if is_video_file:
                        print("[calibrate] End of video.")
                        break
                    continue
                frame = new_frame

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            cv2.setMouseCallback(window, _on_mouse, hsv)

            display = frame.copy()
            status = f"{len(clicked_hsv)} samples"
            if is_video_file:
                status += "  [PAUSED]" if paused else "  [PLAYING - space to pause]"
            cv2.putText(display, status, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow(window, display)

            key = cv2.waitKey(30 if not paused else 0) & 0xFF
            if key in (ord("q"), 27):
                break
            if is_video_file and key == ord(" "):
                paused = not paused
            if is_video_file and paused and key == ord("n"):
                ok, new_frame = cap.read()
                if ok:
                    frame = new_frame
                else:
                    print("[calibrate] End of video.")
                    break
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        for _ in range(4):
            cv2.waitKey(1)

    if clicked_hsv:
        hs = [p[0] for p in clicked_hsv]
        ss = [p[1] for p in clicked_hsv]
        vs = [p[2] for p in clicked_hsv]
        print("\nFinal suggested range (pad ~5-10 each side, and re-test):")
        print(f"  LOWER = ({max(0, min(hs) - 5)}, {max(0, min(ss) - 20)}, {max(0, min(vs) - 20)})")
        print(f"  UPPER = ({min(180, max(hs) + 5)}, 255, 255)")
    else:
        print("\nNo samples taken.")


if __name__ == "__main__":
    main()
