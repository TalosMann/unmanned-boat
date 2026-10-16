"""
Tunable constants. Keep everything you'll adjust at the water's edge here,
not buried in logic files.
"""

# --- HSV thresholds (tune these on-site against actual river lighting) ---
# List of (lower, upper) HSV bound pairs, OR'd together — one pair per
# distinct color cluster the "red" target ball can show the camera.
# A single-color ball needing hue-wrap (e.g. true red, which sits at both
# ends of OpenCV's 0-180 hue range) needs two pairs; a multi-color practice
# ball (e.g. a red/yellow/green ball, used when a real red ball isn't
# available) needs one pair per visible color so detection doesn't depend
# on which face happens to be toward the camera. auto_calibrate_hsv.py
# derives this list automatically via hue clustering against labeled
# ground truth — re-run it whenever the practice ball or lighting changes.
#
# Values below derived from real on-water footage of the red/yellow/green
# practice ball (videos/5aa986df..., 6807eb96..., ea784cec..., "midday on
# water" condition) via auto_calibrate_hsv.py. The 0-75 range is broad
# (covers red through yellow through green) because this ball's three
# color patches form a continuous warm-hue gradient in HSV space in this
# footage, not because of a clustering bug — verified by re-running the
# calibration with a tighter sampling radius and less smoothing, which
# produced the same result. This is broader than a true single-color red
# ball would need, so it relies more heavily on the Hough Circle shape
# filter and CSRT circularity re-validation to reject non-ball background
# clutter of similar hue (foliage, tan pavement, wood) — check
# evaluate_detection.py's false-positive rate specifically before trusting
# this in the field.
RED_HSV_RANGES = [
    ((0, 103, 42), (75, 255, 255)),
    ((167, 103, 42), (179, 255, 255)),
]

COLORED_HSV_LOWER = (20, 100, 100)   # example: yellow-ish ball — retune on-site
COLORED_HSV_UPPER = (35, 255, 255)

MIN_BALL_RADIUS_PX = 8       # ignore detections smaller than this (noise)

# --- Goal HSV thresholds (vision.GoalVision) ---
# PLACEHOLDERS — not calibrated against any real goal footage/photos yet.
# Same tooling applies as the ball: use calibrate_hsv.py or auto_calibrate_hsv.py
# once you have footage of the actual red/blue goal structures. Unlike the ball,
# goals are large/irregular, so GoalVision uses largest-color-blob detection
# (see vision.py's _ColorBlobDetector), not Hough Circle — MIN_GOAL_AREA_PX is
# a pixel-area threshold, not a radius.
RED_GOAL_HSV_RANGES = [
    ((0, 100, 80), (10, 255, 255)),
    ((170, 100, 80), (180, 255, 255)),
]
BLUE_GOAL_HSV_RANGES = [
    ((100, 100, 80), (130, 255, 255)),
]
MIN_GOAL_AREA_PX = 2000       # ignore color blobs smaller than this (noise/background clutter)

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

# ORIENT: bow is presumed against the ball (reached via ALIGN); rotate
# toward the selected goal (dragging the ball with the turn) before SHOOT.
ORIENT_ROTATE_SPEED = 0.2       # rotation speed once the goal is visible and being centered
ORIENT_SEARCH_ROTATE_SPEED = 0.15  # slower rotation while scanning for the goal at all
ORIENT_GOAL_TOLERANCE = 0.1     # goal x_offset considered "centered" if abs < this
ORIENT_GOAL_TIMEOUT_SEC = 4.0   # give up and return to SEARCH if the goal is never found —
                                # own-goal risk (TODO.md) if we kicked blind instead

# --- Safety ---
ESTOP_GPIO_PIN = 17             # hardware e-stop input pin
ESTOP_POLL_HZ = 50               # check e-stop at this frequency, independent of main loop

# --- Camera ---
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# --- Thrusters (motor_control.MotorController, gpiozero + bidirectional ESCs) ---
# Both CONFIRMED wired, per pi_controller_spec.md and the follow-up confirmation:
# right ESC on physical pins 35 (signal, BCM19) / 39 (ground); left ESC on the
# other pair, physical pins 32 (signal, BCM12) / 34 (ground).
LEFT_THRUSTER_PIN = 12    # BCM GPIO number, not physical pin. BCM12 = physical pin 32
RIGHT_THRUSTER_PIN = 19   # BCM19 = physical pin 35
ESC_NEUTRAL_US = 1500     # standard RC pulse widths: 1000=full reverse, 1500=stop, 2000=full forward
ESC_MIN_US = 1000
ESC_MAX_US = 2000
ESC_ARM_DELAY_SEC = 2.0   # most ESCs require a sustained neutral signal before they'll respond
STEER_GAIN = 0.2          # how hard x_offset biases the differential during drive_toward —
                          # kept well below FORWARD_SPEED so even a full-edge x_offset
                          # (+-1.0) still leaves both thrusters net-forward instead of
                          # near-spinning one to a stop, which was swinging the camera
                          # past the ball and losing it every time it tried to turn
SHOOT_BURST_SEC = 0.5     # duration of full-power burst on shoot() — the pre-kicker
                          # stand-in behavior; kept for test_thrusters.py/bench use,
                          # the real scoring action is now kick() below

# --- Kicker (motor_control.MotorController, PCA9685-driven servo) ---
# Per pi_controller_spec.md: PCA9685 over I2C, not a Pi GPIO pin — see
# pca9685.py. Retract/kick values confirmed against the real servo
# (~65-degree swing); KICKER_PCA_CHANNEL is still a placeholder.
KICKER_PCA_I2C_ADDRESS = 0x40   # PCA9685 default address (confirmed free, no BNO055 conflict at 0x28)
KICKER_PCA_CHANNEL = 0          # which PCA9685 output the servo is wired to — PLACEHOLDER, confirm
KICKER_RETRACT_US = 1100        # CONFIRMED against the real servo
KICKER_KICK_US = 1820           # CONFIRMED — ~65-degree swing from retract
KICKER_KICK_HOLD_SEC = 0.3      # how long to hold the kick position before retracting

# --- Thruster inversion (motor_control.MotorController) ---
# Per pi_controller_spec.md: the thrusters are a mirrored pair, one
# reverse-pitch, needing a per-thruster invert flag IN ADDITION TO (not
# instead of) the existing global sign flip in _set_thruster (that one
# corrects for both thrusters being mounted backwards collectively; this
# one corrects for the one reverse-pitch thruster specifically).
# PLACEHOLDER — which side needs inverting is one of the brief's own
# listed open items, confirm against the physical thrusters.
LEFT_THRUSTER_INVERT = False
RIGHT_THRUSTER_INVERT = False

# --- Remote control (FlySky FS-i6X transmitter / IA10B receiver) ---
# Confirmed receiver link: i-BUS, 115200 baud, Pi UART RX (physical pin 10,
# BCM GPIO15) — requires freeing this UART from the Linux serial console
# via raspi-config first (code-independent OS step). See remote_input.py's
# docstring for the earlier i-BUS-vs-SBUS discrepancy; pi_controller_spec.md
# confirms i-BUS directly, so that's settled.
#
# Kill switch: CH5 (SwA) drives BOTH relay coils directly in hardware,
# independent of the Pi — that path is NOT software's to control. Software
# ALSO reads CH5 below and commands neutral when it reads "off," as a
# second, redundant safety layer — never the only path, matching the
# brief's explicit rule ("software must never be the only kill path and
# must never try to override CH5").
REMOTE_SERIAL_PORT = "/dev/serial0"
REMOTE_LINK_TIMEOUT_SEC = 0.25   # per the brief: no valid packet for ~250ms -> neutral + retract

# i-BUS channel indices (0-based: CH1=index 0). Channel map per
# pi_controller_spec.md — confirm against the transmitter's own Display
# screen (test_rc_diagnostic.py) before trusting it; CONFIRMED items below
# are from the brief directly, PLACEHOLDER items are its own listed open items.
REMOTE_CH_TURN = 0           # CH1, right stick left/right (CONFIRMED)
REMOTE_CH_FORWARD = 1        # CH2, right stick up/down (CONFIRMED)
REMOTE_CH_KILL = 4           # CH5 / SwA — kill relay channel (CONFIRMED, see note above)
REMOTE_CH_KICK_TRIGGER = 6   # CH7 / SwB (CONFIRMED channel assignment in the brief)
REMOTE_CH_POWER_CAP = 7      # CH8 / SwC, 3-position: 25/50/100% (CONFIRMED channel)
REMOTE_CH_CONTROL_MODE = 8   # CH9 / SwD: manual vs autonomous (CONFIRMED channel)
REMOTE_CH_GOAL_SELECT = None  # NOT part of this brief's channel map — goal-select from earlier
                              # work is unassigned for now (every other switch is spoken for);
                              # not read by test_rc_control.py, needs a real channel once an
                              # unused one is identified before any future autonomous-mode use

REMOTE_KILL_OFF_IS_HIGH = True  # PLACEHOLDER — which raw CH5 position means "relays off."
                                 # Confirm via test_rc_diagnostic.py while physically flipping
                                 # SwA and listening/watching for the relay actually disengaging.

REMOTE_US_MIN = 1000
REMOTE_US_MID = 1500
REMOTE_US_MAX = 2000
REMOTE_SWITCH_HIGH_THRESHOLD_US = 1700
REMOTE_SWITCH_LOW_THRESHOLD_US = 1300
REMOTE_STICK_DEADZONE_US = 25   # ~5% of the 500us half-range, per the brief

# --- Manual control-mode tuning ---
RC_STARTUP_HOLD_SEC = 3.0            # hold neutral this long after arming before
                                      # accepting any RC input at all, per the brief
POWER_CAP_LEVELS = [0.25, 0.50, 1.00]  # low/mid/high positions of the SwC power-cap switch
                                      # — default/first bench tests should stay on low
CONTROL_SLEW_RATE_PER_SEC = 1.5      # max change in thrust command (-1..1 scale) per second —
                                      # conservative starting value for first-ever tests with
                                      # the new thrusters; a full -1..1 sweep takes ~1.3s
