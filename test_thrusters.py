"""
Standalone bench test for MotorController — run this on the Pi with props
OFF before ever trusting the FSM/vision loop to drive real thrusters.
Confirms wiring, pin numbers, and direction sense one motion at a time.

Requires gpiozero + lgpio installed:
    sudo apt install python3-gpiozero python3-lgpio

Usage (on the Pi):
    python3 test_thrusters.py

Each step prints what it's about to do and waits for Enter before it does
it, and waits for Enter again before moving on — so you can react/cut
power if anything spins the wrong way or unexpectedly.
"""

import config
from motor_control import MotorController


def step(description, action, motors):
    input(f"\n[test] About to: {description}. Press Enter to run it (Ctrl+C to abort)...")
    action(motors)
    input("[test] Running. Press Enter to stop and continue to the next step...")
    motors.stop()


def main():
    print("[test] Initializing GPIO and arming ESCs...")
    motors = MotorController(config)

    try:
        step("LEFT thruster forward, half speed",
             lambda m: m._set_thruster(m.left, 0.5), motors)
        step("LEFT thruster reverse, half speed",
             lambda m: m._set_thruster(m.left, -0.5), motors)
        step("RIGHT thruster forward, half speed",
             lambda m: m._set_thruster(m.right, 0.5), motors)
        step("RIGHT thruster reverse, half speed",
             lambda m: m._set_thruster(m.right, -0.5), motors)
        step("rotate_in_place(right), both thrusters opposing",
             lambda m: m.rotate_in_place(0.4), motors)
        step("rotate_in_place(left), both thrusters opposing",
             lambda m: m.rotate_in_place(-0.4), motors)
        step("drive_toward(x_offset=0.0, speed=0.4) — straight forward",
             lambda m: m.drive_toward(0.0, 0.4), motors)
        step("drive_toward(x_offset=0.5, speed=0.4) — should curve right",
             lambda m: m.drive_toward(0.5, 0.4), motors)
        step("shoot() — brief full-power burst then stop",
             lambda m: m.shoot(), motors)

        print("\n[test] All steps complete.")

    except KeyboardInterrupt:
        print("\n[test] Aborted — stopping thrusters.")
    finally:
        motors.stop()
        print("[test] Thrusters stopped. Done.")


if __name__ == "__main__":
    main()
