"""
Bench/water test: the FlySky FS-i6X controller drives the two thrusters
and kick servo directly over i-BUS — no camera, no vision, no FSM.
Implements every safety rule from pi_controller_spec.md:

  - Holds neutral for RC_STARTUP_HOLD_SEC after arming before accepting
    any RC input at all.
  - CH5 (kill switch) -> neutral in software too, redundant with (never a
    substitute for) the independent hardware relay on the same channel.
  - No valid i-BUS packet for REMOTE_LINK_TIMEOUT_SEC -> neutral (does not
    rely on staleness alone for the kill check — that's CH5's job, checked
    separately, since a dead transmitter can leave stale-but-present stick
    values being repeated by the receiver).
  - Slew-rate limited thruster commands — no instant full-reverse snaps.
  - Power cap from CH8's 3-position switch (25/50/100%), defaults to
    whatever position the switch is physically in; test on the LOW
    position first.
  - CH9 selects MANUAL (this script drives) vs AUTONOMOUS (this script
    only enforces kill, drives nothing — there's no FSM wired into this
    standalone test).
  - Clean neutral-and-exit on Ctrl-C, SIGTERM, or any exception.

Run on the Pi (props/water-safe area, same posture as every other
hardware test in this project):
    python3 test_rc_control.py
"""

import signal
import sys
import time

import config
from remote_input import IBusReceiver
from remote_control import (
    RemoteMode,
    read_kill_active,
    read_control_mode,
    read_power_cap,
    read_kick_trigger,
    read_manual_mix,
)
from motor_control import MotorController

_shutdown_requested = False


def _handle_shutdown_signal(signum, frame):
    global _shutdown_requested
    _shutdown_requested = True


def _slew_limit(current, target, max_step):
    if target > current:
        return min(target, current + max_step)
    return max(target, current - max_step)


def main():
    signal.signal(signal.SIGTERM, _handle_shutdown_signal)
    signal.signal(signal.SIGINT, _handle_shutdown_signal)

    print(f"[rc-test] Connecting to receiver on {config.REMOTE_SERIAL_PORT}...")
    receiver = IBusReceiver(config.REMOTE_SERIAL_PORT).start()

    print("[rc-test] Starting motors (arms ESCs, holds neutral)...")
    motors = MotorController(config)

    print(f"[rc-test] Holding neutral for {config.RC_STARTUP_HOLD_SEC}s before accepting RC input...")
    time.sleep(config.RC_STARTUP_HOLD_SEC)
    print("[rc-test] Ready. Props/water-safe area. Ctrl+C to stop.\n")

    left_cmd = 0.0
    right_cmd = 0.0
    last_log_at = 0.0
    loop_period_sec = 0.02
    max_step = config.CONTROL_SLEW_RATE_PER_SEC * loop_period_sec

    try:
        while not _shutdown_requested:
            channels = receiver.get_channels()
            stale = receiver.is_stale(config.REMOTE_LINK_TIMEOUT_SEC)

            if channels is None or stale:
                target_left, target_right = 0.0, 0.0
                status = "NO LINK / STALE -> neutral"

            elif read_kill_active(channels, config):
                target_left, target_right = 0.0, 0.0
                status = "KILL (CH5) -> neutral"

            else:
                mode = read_control_mode(channels, config)

                if mode == RemoteMode.AUTONOMOUS:
                    # No FSM in this standalone test — AUTONOMOUS means
                    # "this controller only acts as kill," per the brief.
                    target_left, target_right = 0.0, 0.0
                    status = "AUTONOMOUS selected (no FSM here) -> holding neutral"
                else:
                    power_cap = read_power_cap(channels, config)
                    target_left, target_right = read_manual_mix(channels, config, power_cap)

                    if read_kick_trigger(channels, config):
                        motors.kick()

                    status = (f"MANUAL cap={power_cap:.0%} "
                              f"left={target_left:+.2f} right={target_right:+.2f}")

            left_cmd = _slew_limit(left_cmd, target_left, max_step)
            right_cmd = _slew_limit(right_cmd, target_right, max_step)
            motors.manual_drive(left_cmd, right_cmd)

            now = time.time()
            if now - last_log_at > 0.5:
                print(f"[rc-test] {status}", flush=True)
                last_log_at = now

            time.sleep(loop_period_sec)

    except Exception:
        print("[rc-test] Exception in control loop — neutralizing before re-raising:", file=sys.stderr)
        raise
    finally:
        print("\n[rc-test] Shutting down.")
        motors.stop()
        receiver.stop()
        print("[rc-test] Motors neutral, receiver stopped.")


if __name__ == "__main__":
    main()
