"""
Two motor controller implementations:
  - MockMotorController: prints actions, for testing on a laptop webcam
    with no ESCs/thrusters attached.
  - MotorController: real PWM/ESC driver for the Pi, via gpiozero + two
    bidirectional RC-style ESCs (one per thruster, differential drive),
    plus a PCA9685-driven kick servo.

main.py should import MockMotorController for webcam testing and swap to
MotorController once you're wiring real thrusters.

MotorController uses gpiozero.Servo (backed by the lgpio pin factory,
gpiozero's default on modern Raspberry Pi OS / Pi 5) for the ESCs. Install
with:
    sudo apt install python3-gpiozero python3-lgpio
No daemon needs to be started (unlike pigpio, which Debian Trixie no
longer packages and which doesn't reliably support newer Pi kernels).
This is software-timed PWM (gpiozero prints a PWMSoftwareFallback warning
on startup) rather than the Pi 5 RP1's dedicated hardware PWM — a known
tradeoff, kept deliberately for now since it's already proven working on
real hardware; revisit if jitter becomes an actual problem in practice.

The kick servo instead uses a PCA9685 I2C PWM chip (pca9685.py) — see
config.py's KICKER_PCA_* constants.

Wire the two ESC signal wires to config.LEFT_THRUSTER_PIN /
config.RIGHT_THRUSTER_PIN (BCM numbering). ESC power/ground comes from the
battery, not the Pi — only the PWM signal wire (+ shared ground) goes to
the GPIO pins.

Two independent inversion layers exist for the thrusters, for two
different physical reasons — don't conflate them:
  - The sign flip inside _set_thruster() corrects for BOTH thrusters being
    physically mounted/wired such that a positive PWM value drives them in
    reverse (discovered and fixed earlier, applies to both equally).
  - config.LEFT_THRUSTER_INVERT / RIGHT_THRUSTER_INVERT correct for the
    thrusters being a MIRRORED pair (one reverse-pitch) — this only ever
    applies to one side.

Servo.detach() / PCA9685.release() (used in emergency_stop) stop sending
pulses, which most ESCs/servos treat as a stop condition, but that's a
software fail-safe, not a guaranteed power cut — it does NOT replace the
hardware kill relays (CH5, independent of the Pi) required for a real
e-stop.
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

    def manual_drive(self, left, right):
        self._log(f"manual_drive(left={left:.2f}, right={right:.2f})")

    def shoot(self):
        print("[motors] SHOOT")

    def kick(self):
        print("[motors] KICK")

    def stop(self):
        self._log("stop()")

    def emergency_stop(self):
        print("[motors] *** EMERGENCY STOP ***")


class MotorController:
    """Real hardware driver: two bidirectional ESCs (left/right thruster)
    over gpiozero Servo PWM, plus a PCA9685-driven kick servo. Matches
    MockMotorController's interface so main.py doesn't need to change
    when you swap it in."""

    def __init__(self, config):
        from gpiozero import Servo  # imported here so this module still loads on machines without gpiozero (e.g. your laptop)
        from pca9685 import PCA9685

        self.config = config
        min_pulse_s = config.ESC_MIN_US / 1_000_000
        max_pulse_s = config.ESC_MAX_US / 1_000_000

        self.left = Servo(config.LEFT_THRUSTER_PIN, min_pulse_width=min_pulse_s, max_pulse_width=max_pulse_s)
        self.right = Servo(config.RIGHT_THRUSTER_PIN, min_pulse_width=min_pulse_s, max_pulse_width=max_pulse_s)
        self.left_invert = config.LEFT_THRUSTER_INVERT
        self.right_invert = config.RIGHT_THRUSTER_INVERT

        self.pca = PCA9685(config.KICKER_PCA_I2C_ADDRESS)
        self.pca.set_pulse_us(config.KICKER_PCA_CHANNEL, config.KICKER_RETRACT_US)

        self._arm()

    def _arm(self):
        """Most ESCs require a sustained neutral (stop) signal before they'll
        accept other commands — this is their startup calibration/safety check."""
        print("[motors] Arming ESCs (holding neutral)...")
        self.left.value = 0.0
        self.right.value = 0.0
        time.sleep(self.config.ESC_ARM_DELAY_SEC)
        print("[motors] ESCs armed.")

    def _set_thruster(self, servo, speed, invert=False):
        """speed: -1.0 (full reverse) to 1.0 (full forward).
        `invert` applies the per-thruster mirrored-pair correction BEFORE
        the global physical-mounting sign flip (see module docstring for
        why these are two separate things)."""
        speed = max(-1.0, min(1.0, speed))
        if invert:
            speed = -speed
        servo.value = -speed

    def rotate_in_place(self, speed):
        # positive speed = rotate right: left thruster forward, right thruster reverse
        speed = max(-1.0, min(1.0, speed))
        self._set_thruster(self.left, speed, self.left_invert)
        self._set_thruster(self.right, -speed, self.right_invert)

    def drive_toward(self, x_offset, speed):
        # x_offset in [-1, 1] (positive = target is right of center). Matches
        # rotate_in_place's convention (left relatively faster -> turn right):
        # target to the right -> boost left / ease right so the boat curves
        # toward it while still moving forward.
        steer = x_offset * self.config.STEER_GAIN
        self._set_thruster(self.left, speed + steer, self.left_invert)
        self._set_thruster(self.right, speed - steer, self.right_invert)

    def manual_drive(self, left, right):
        """Directly sets each thruster independently — used by RC manual
        control (raw stick input, already mixed+slew-limited by the
        caller), bypassing the x_offset/speed abstraction rotate_in_place/
        drive_toward use for vision-driven autonomous steering."""
        self._set_thruster(self.left, left, self.left_invert)
        self._set_thruster(self.right, right, self.right_invert)

    def shoot(self):
        print("[motors] SHOOT")
        self._set_thruster(self.left, 1.0, self.left_invert)
        self._set_thruster(self.right, 1.0, self.right_invert)
        time.sleep(self.config.SHOOT_BURST_SEC)
        self.stop()

    def kick(self):
        """Fires the kick servo: retract -> kick position -> back to retract.
        Blocking for KICKER_KICK_HOLD_SEC, same pattern as shoot()'s burst."""
        print("[motors] KICK")
        self.pca.set_pulse_us(self.config.KICKER_PCA_CHANNEL, self.config.KICKER_KICK_US)
        time.sleep(self.config.KICKER_KICK_HOLD_SEC)
        self.pca.set_pulse_us(self.config.KICKER_PCA_CHANNEL, self.config.KICKER_RETRACT_US)

    def stop(self):
        self._set_thruster(self.left, 0.0, self.left_invert)
        self._set_thruster(self.right, 0.0, self.right_invert)

    def emergency_stop(self):
        print("[motors] *** EMERGENCY STOP ***")
        # detach()/release() = stop sending pulses entirely, rather than
        # commanding neutral — a software fail-safe most ESCs/servos honor,
        # but not a substitute for the hardware kill relays on CH5.
        self.left.detach()
        self.right.detach()
        self.pca.release(self.config.KICKER_PCA_CHANNEL)
