"""
Full pipeline for the Pi: Pi Camera Module -> BallVision/GoalVision
(HSV+Hough+CSRT ball, color-blob goals) -> BoatFSM -> real thrusters +
kicker via MotorController, gated by the FS-i6X/IA10B remote's CH5 kill
switch and CH9 mode switch (MANUAL / AUTONOMOUS).

Run on the Pi:
    python3 main_pi.py

Ctrl+C to stop — cleanly stops the thrusters and camera on exit.

SAFETY:
  - This does NOT touch the kill relays — CH5 drives them directly in
    hardware, independent of the Pi, and stays that way. This script ALSO
    reads CH5 and commands neutral when it's off, as a second, redundant
    software safety layer — never the only kill path, never something
    that overrides the hardware relay.
  - MANUAL mode bypasses vision/FSM entirely — camera/detection aren't
    even run while in MANUAL, so there's no path for an autonomous
    command to leak through while you're driving it manually.
  - AUTONOMOUS mode: this script's RC reading only still acts as kill;
    the FSM drives movement.
  - If the remote link is lost or never connects, treated the same as kill.

NOTE: goal-color selection (config.REMOTE_CH_GOAL_SELECT) is currently
unassigned (None) — pi_controller_spec.md's channel map doesn't include
it, every switch is spoken for. AUTONOMOUS mode here runs with whatever
BoatFSM's default goal color is until a channel is identified for it.
"""

import time

import config
from fsm import BoatFSM
from vision import BallVision, GoalVision
from motor_control import MotorController
from remote_input import IBusReceiver
from remote_control import (
    RemoteMode,
    read_kill_active,
    read_control_mode,
    read_power_cap,
    read_kick_trigger,
    read_manual_mix,
)


class PiCameraGrabber:
    def __init__(self, width, height):
        from picamera2 import Picamera2

        self.picam2 = Picamera2()
        cam_config = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            buffer_count=2,  # low buffering so we always process the freshest
                             # frame instead of a growing backlog when
                             # detection can't keep up with capture rate
        )
        self.picam2.configure(cam_config)
        self.picam2.start()
        time.sleep(1.0)  # let auto-exposure/white-balance settle before the first frame

    def get_frame(self):
        # picamera2's "RGB888" format is actually BGR byte order despite the
        # name — matches what OpenCV expects, no channel swap needed.
        return self.picam2.capture_array()

    def stop(self):
        self.picam2.stop()


def main():
    print("[main] Starting pipeline. Ctrl+C to stop.")
    motors = MotorController(config)
    camera = PiCameraGrabber(config.FRAME_WIDTH, config.FRAME_HEIGHT)
    vision = BallVision(config)
    goal_vision = GoalVision(config)
    fsm = BoatFSM(motors, config)
    remote = IBusReceiver(config.REMOTE_SERIAL_PORT).start()

    try:
        while True:
            channels = remote.get_channels()
            link_ok = channels is not None and not remote.is_stale(config.REMOTE_LINK_TIMEOUT_SEC)

            if not link_ok:
                killed = True
            else:
                killed = read_kill_active(channels, config)

            if killed:
                motors.stop()
                status = "link_lost -> neutral" if not link_ok else "KILL (CH5) -> neutral"

            elif read_control_mode(channels, config) == RemoteMode.MANUAL:
                power_cap = read_power_cap(channels, config)
                left, right = read_manual_mix(channels, config, power_cap)
                motors.manual_drive(left, right)
                if read_kick_trigger(channels, config):
                    motors.kick()
                status = f"MANUAL cap={power_cap:.0%} left={left:+.2f} right={right:+.2f}"

            else:  # AUTONOMOUS
                frame = camera.get_frame()
                vision_result = vision.detect(frame)
                goal_result = goal_vision.detect(frame)
                fsm.update(vision_result, goal_result)
                red = vision_result["red"]
                status = (f"AUTO state={fsm.state.value} goal={fsm.selected_goal_color} "
                          f"ball_found={red['ball_found']!s:5s} x_offset={red['x_offset']:+.2f}")

            print(f"[main] {status}                              ", end="\r")
            time.sleep(0.02)  # ~50Hz control loop

    except KeyboardInterrupt:
        print("\n[main] Stopped by user.")
    finally:
        motors.stop()
        camera.stop()
        remote.stop()
        print("[main] Thrusters, camera, and remote stopped.")


if __name__ == "__main__":
    main()
