"""
Tunable constants. Keep everything you'll adjust at the water's edge here,
not buried in logic files.
"""

# --- HSV thresholds (tune these on-site against actual river lighting) ---
# Derived from auto_calibrate_hsv.py against video_3/video_5's labeled
# ground truth (indoor basketball footage, NOT the Pi's camera/lighting —
# treat as a starting point and re-calibrate against real Pi-camera/water
# footage once available). Red wraps around the HSV hue circle (0 and 180
# are both "red"), so it needs two ranges OR'd together.
RED_HSV_LOWER1 = (0, 165, 109)
RED_HSV_UPPER1 = (9, 255, 255)
RED_HSV_LOWER2 = (170, 165, 109)
RED_HSV_UPPER2 = (180, 255, 255)

COLORED_HSV_LOWER = (20, 100, 100)   # example: yellow-ish ball — retune on-site
COLORED_HSV_UPPER = (35, 255, 255)

MIN_BALL_RADIUS_PX = 8       # ignore detections smaller than this (noise)

# CSRT tracks generic visual appearance, not color/shape — it can happily
# drift onto the wrong object (a vest, background, frame edge) and keep
# reporting success. Each frame we re-check that the tracked box still
# actually matches the ball's HSV color; below this fraction, we treat the
# track as lost and fall back to HSV+Hough re-acquisition.
MIN_COLOR_MATCH_FRACTION = 0.35

# --- FSM tuning ---
SEARCH_ROTATE_SPEED = 0.3       # 0-1 fraction of max rotate speed
APPROACH_SPEED = 0.5
ALIGN_ROTATE_SPEED = 0.25
ALIGN_TOLERANCE = 0.08          # x_offset considered "centered" if abs < this
ALIGN_DISTANCE_THRESHOLD = 60   # ball radius (px) at which to stop approaching, start aligning
LOST_BALL_TIMEOUT = 1.5         # seconds before dropping back to SEARCH
RESET_PAUSE_SEC = 1.0

# --- Safety ---
ESTOP_GPIO_PIN = 17             # hardware e-stop input pin
ESTOP_POLL_HZ = 50               # check e-stop at this frequency, independent of main loop

# --- Camera ---
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# --- Thrusters (motor_control.MotorController, gpiozero + bidirectional ESCs) ---
LEFT_THRUSTER_PIN = 12    # BCM GPIO number, not physical pin. BCM12 = physical pin 32
                          # (matches the single ESC currently wired to physical pin 32/34)
RIGHT_THRUSTER_PIN = 19  # BCM19 = physical pin 35 — not yet wired to a second ESC
ESC_NEUTRAL_US = 1500     # standard RC pulse widths: 1000=full reverse, 1500=stop, 2000=full forward
ESC_MIN_US = 1000
ESC_MAX_US = 2000
ESC_ARM_DELAY_SEC = 2.0   # most ESCs require a sustained neutral signal before they'll respond
STEER_GAIN = 0.2          # how hard x_offset biases the differential during drive_toward —
                          # kept well below FORWARD_SPEED so even a full-edge x_offset
                          # (+-1.0) still leaves both thrusters net-forward instead of
                          # near-spinning one to a stop, which was swinging the camera
                          # past the ball and losing it every time it tried to turn
SHOOT_BURST_SEC = 0.5     # duration of full-power burst on shoot()
