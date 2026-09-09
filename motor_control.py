"""
Two motor controller implementations:
  - MockMotorController: prints actions, for testing on a laptop webcam
    with no ESCs/thrusters attached.
  - MotorController: real PWM/ESC driver for the Pi, via gpiozero + two
    bidirectional RC-style ESCs (one per thruster, differential drive).

main.py should import MockMotorController for webcam testing and swap to
MotorController once you're wiring real thrusters.

MotorController uses gpiozero.Servo (backed by the lgpio pin factory,
gpiozero's default on modern Raspberry Pi OS / Pi 5). Install with:
    sudo apt install python3-gpiozero python3-lgpio
No daemon needs to be started (unlike pigpio, which Debian Trixie no
longer packages and which doesn't reliably support newer Pi kernels).

Wire the two ESC signal wires to config.LEFT_THRUSTER_PIN /
config.RIGHT_THRUSTER_PIN (BCM numbering). ESC power/ground comes from the
battery, not the Pi — only the PWM signal wire (+ shared ground) goes to
the GPIO pins.

Servo.detach() (used in emergency_stop) stops sending pulses, which most
ESCs treat as a stop condition, but that's a software fail-safe, not a
guaranteed power cut — it does NOT satisfy rulebook §4.4's hardware
e-stop requirement (main.py's EStopMonitor / real GPIO relay is what's
required for that).
"""

import time


class MockMotorController:
    def __init__(self):
        self._last_action = None

    def _log(self, msg):
        if msg != self._last_action:  # avoid spamming identical repeats
            print(f"[motors] {msg}")
            self._last_action = msg

    def rotate_in_place(self, speed):
        direction = "right" if speed > 0 else "left"
        self._log(f"rotate_in_place({direction}, speed={abs(speed):.2f})")

    def drive_toward(self, x_offset, speed):
        self._log(f"drive_toward(x_offset={x_offset:.2f}, speed={speed:.2f})")

    def shoot(self):
        print("[motors] SHOOT")

    def stop(self):
        self._log("stop()")

    def emergency_stop(self):
        print("[motors] *** EMERGENCY STOP ***")


class MotorController:
    """Real hardware driver: two bidirectional ESCs (left/right thruster)
    over gpiozero Servo PWM, matching MockMotorController's interface so
    main.py doesn't need to change when you swap it in."""

    def __init__(self, config):
        from gpiozero import Servo  # imported here so this module still loads on machines without gpiozero (e.g. your laptop)

        self.config = config
        min_pulse_s = config.ESC_MIN_US / 1_000_000
        max_pulse_s = config.ESC_MAX_US / 1_000_000

        self.left = Servo(config.LEFT_THRUSTER_PIN, min_pulse_width=min_pulse_s, max_pulse_width=max_pulse_s)
        self.right = Servo(config.RIGHT_THRUSTER_PIN, min_pulse_width=min_pulse_s, max_pulse_width=max_pulse_s)
        self._arm()

    def _arm(self):
        """Most ESCs require a sustained neutral (stop) signal before they'll
        accept other commands — this is their startup calibration/safety check."""
        print("[motors] Arming ESCs (holding neutral)...")
        self.left.value = 0.0
        self.right.value = 0.0
        time.sleep(self.config.ESC_ARM_DELAY_SEC)
        print("[motors] ESCs armed.")

    def _set_thruster(self, servo, speed):
        """speed: -1.0 (full reverse) to 1.0 (full forward).
        Negated here: both thrusters are physically mounted/wired such that
        a positive PWM value actually drives them in reverse — flipping the
        sign once, centrally, corrects this for every caller (rotate_in_place,
        drive_toward, shoot, stop) without changing any of their logic."""
        servo.value = -max(-1.0, min(1.0, speed))

    def rotate_in_place(self, speed):
        # positive speed = rotate right: left thruster forward, right thruster reverse
        speed = max(-1.0, min(1.0, speed))
        self._set_thruster(self.left, speed)
        self._set_thruster(self.right, -speed)

    def drive_toward(self, x_offset, speed):
        # x_offset in [-1, 1] (positive = target is right of center). Matches
        # rotate_in_place's convention (left relatively faster -> turn right):
        # target to the right -> boost left / ease right so the boat curves
        # toward it while still moving forward.
        steer = x_offset * self.config.STEER_GAIN
        self._set_thruster(self.left, speed + steer)
        self._set_thruster(self.right, speed - steer)

    def shoot(self):
        print("[motors] SHOOT")
        self._set_thruster(self.left, 1.0)
        self._set_thruster(self.right, 1.0)
        time.sleep(self.config.SHOOT_BURST_SEC)
        self.stop()

    def stop(self):
        self._set_thruster(self.left, 0.0)
        self._set_thruster(self.right, 0.0)

    def emergency_stop(self):
        print("[motors] *** EMERGENCY STOP ***")
        # detach() = stop sending pulses entirely, rather than commanding
        # neutral — a software fail-safe most ESCs honor, but not a
        # substitute for the hardware e-stop relay required by rulebook §4.4.
        self.left.detach()
        self.right.detach()
