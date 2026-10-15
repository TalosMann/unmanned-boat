"""
Experiment 1 — Detection-to-Actuation Latency Breakdown.

Replays a recorded video through the ACTUAL pipeline (vision.BallVision
-> fsm.BoatFSM -> motor_control) and times each stage with
time.perf_counter(), exactly as described in experiment_design.pdf:

    (a) Capture -> Detection   : vision.BallVision.detect()
    (b) Detection -> Decision  : gap between detect() returning and the
                                  FSM issuing a motor command
    (c) Decision -> Actuation  : gap between the FSM calling a motor
                                  method and the PWM value actually being
                                  written to the Servo object

No human timing involved — every timestamp comes from the Pi's own
clock. Run this ON THE PI with the real MotorController for authentic
embedded-hardware numbers (the ESC does not need to be powered/armed
for this — no propellers need to spin, we're only timing how fast the
signal is dispatched, not whether it moves anything). It also runs fine
on a laptop with --mock for a quick sanity check, using
MockMotorController instead.

NOTE ON "CAPTURE": since this replays a video file rather than a live
camera, the "capture" stage here is video-decode time, not real sensor
capture latency — call this out in the paper as a substitution made
necessary by the damaged battery preventing live on-water camera
testing. Detection/decision/actuation timings are unaffected by this
substitution (that compute happens identically either way).

Usage (on the Pi, real hardware):
    python3 experiment1_latency.py ball_on_water.mp4 --out latency.csv

Usage (laptop, no hardware):
    python3 experiment1_latency.py ball_on_water.mp4 --mock --out latency.csv
"""

import argparse
import csv
import time

import cv2

import config
from vision import BallVision
from fsm import BoatFSM


class TimestampingMotors:
    """Wraps a real or mock motor controller. Records the instant each
    drive command is issued (decision -> actuation dispatch), and, when
    wrapping the real MotorController, the instant the PWM value is FIRST
    written to a Servo object for that command (actuation dispatch ->
    signal write).

    Records only the FIRST _set_thruster call per command, not the last:
    shoot() internally does full-thrust -> time.sleep(SHOOT_BURST_SEC) ->
    stop() (which itself calls _set_thruster again). If we kept
    overwriting on every call, a SHOOT command's recorded "write" would be
    the trailing stop()'s write AFTER the burst sleep, contaminating the
    latency reading with ~500ms that has nothing to do with dispatch
    speed."""

    def __init__(self, inner):
        self.inner = inner
        self.command_issued_at = None
        self.pwm_written_at = None

        # Only the real MotorController has _set_thruster (writes PWM);
        # MockMotorController just prints, so there's nothing to time there.
        if hasattr(inner, "_set_thruster"):
            original_set_thruster = inner._set_thruster

            def timed_set_thruster(servo, speed):
                result = original_set_thruster(servo, speed)
                if self.pwm_written_at is None:  # first write for this command only
                    self.pwm_written_at = time.perf_counter()
                return result

            inner._set_thruster = timed_set_thruster

    def _mark_and_call(self, method_name, *args, **kwargs):
        self.command_issued_at = time.perf_counter()
        self.pwm_written_at = None
        return getattr(self.inner, method_name)(*args, **kwargs)

    def rotate_in_place(self, speed):
        return self._mark_and_call("rotate_in_place", speed)

    def drive_toward(self, x_offset, speed):
        return self._mark_and_call("drive_toward", x_offset, speed)

    def shoot(self):
        return self._mark_and_call("shoot")

    def stop(self):
        return self.inner.stop()

    def emergency_stop(self):
        return self.inner.emergency_stop()


def main():
    parser = argparse.ArgumentParser(description="Time the detect->decide->actuate pipeline against a recorded video.")
    parser.add_argument("video")
    parser.add_argument("--out", default="latency_results.csv", help="Where to save per-frame timing data")
    parser.add_argument("--mock", action="store_true",
                         help="Use MockMotorController instead of real GPIO hardware "
                              "(for a quick sanity check off the Pi; run without this "
                              "flag ON THE PI for real embedded-hardware numbers)")
    args = parser.parse_args()

    if args.mock:
        from motor_control import MockMotorController
        inner_motors = MockMotorController()
    else:
        from motor_control import MotorController
        inner_motors = MotorController(config)

    motors = TimestampingMotors(inner_motors)
    vision = BallVision(config)
    fsm = BoatFSM(motors, config)

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"[exp1] Could not open video: {args.video}")
        return

    rows = []
    frame_idx = 0

    print(f"[exp1] Replaying {args.video} through the real pipeline "
          f"({'mock' if args.mock else 'real hardware'} motors)...")

    while True:
        t_capture_start = time.perf_counter()
        ok, frame = cap.read()
        if not ok:
            break
        t_capture_done = time.perf_counter()

        t_detect_start = time.perf_counter()
        vision_result = vision.detect(frame)
        t_detect_done = time.perf_counter()

        detection_path = "steady-state (CSRT)" if vision._red.tracking else "cold-start (HSV+Hough)"

        motors.command_issued_at = None
        motors.pwm_written_at = None
        fsm.update(vision_result)

        capture_to_detect_ms = (t_detect_done - t_detect_start) * 1000
        detect_to_decision_ms = None
        decision_to_actuation_ms = None

        if motors.command_issued_at is not None:
            detect_to_decision_ms = (motors.command_issued_at - t_detect_done) * 1000
            if motors.pwm_written_at is not None:
                decision_to_actuation_ms = (motors.pwm_written_at - motors.command_issued_at) * 1000

        rows.append({
            "frame_idx": frame_idx,
            "detection_path": detection_path,
            "ball_found": vision_result["red"]["ball_found"],
            "fsm_state": fsm.state.value,
            "capture_decode_ms": round((t_capture_done - t_capture_start) * 1000, 3),
            "capture_to_detect_ms": round(capture_to_detect_ms, 3),
            "detect_to_decision_ms": round(detect_to_decision_ms, 3) if detect_to_decision_ms is not None else "",
            "decision_to_actuation_ms": round(decision_to_actuation_ms, 3) if decision_to_actuation_ms is not None else "",
        })

        frame_idx += 1

    cap.release()
    motors.stop()

    if not rows:
        print("[exp1] No frames processed.")
        return

    with open(args.out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    _report(rows, args.out)


def _report(rows, out_path):
    def stage_stats(rows, key, path_filter=None):
        vals = [r[key] for r in rows if r[key] != "" and (path_filter is None or r["detection_path"] == path_filter)]
        if not vals:
            return None
        n = len(vals)
        mean = sum(vals) / n
        variance = sum((v - mean) ** 2 for v in vals) / n
        return {"n": n, "mean_ms": mean, "std_ms": variance ** 0.5}

    print(f"\n[exp1] Processed {len(rows)} frames. Full data saved to {out_path}\n")
    print(f"{'Stage':<32} {'Path':<24} {'n':>6} {'Mean (ms)':>10} {'Std (ms)':>10}")
    print("-" * 86)

    for path in ("cold-start (HSV+Hough)", "steady-state (CSRT)"):
        stats = stage_stats(rows, "capture_to_detect_ms", path)
        if stats:
            print(f"{'Capture -> Detection':<32} {path:<24} {stats['n']:>6} {stats['mean_ms']:>10.2f} {stats['std_ms']:>10.2f}")

    stats = stage_stats(rows, "detect_to_decision_ms")
    if stats:
        print(f"{'Detection -> Decision':<32} {'—':<24} {stats['n']:>6} {stats['mean_ms']:>10.2f} {stats['std_ms']:>10.2f}")

    stats = stage_stats(rows, "decision_to_actuation_ms")
    if stats:
        print(f"{'Decision -> Actuation':<32} {'—':<24} {stats['n']:>6} {stats['mean_ms']:>10.2f} {stats['std_ms']:>10.2f}")
    else:
        print(f"{'Decision -> Actuation':<32} {'(only measured with real MotorController, not --mock)':<24}")


if __name__ == "__main__":
    main()
