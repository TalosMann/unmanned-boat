"""
Finite State Machine for autonomous ball-chasing-and-scoring behavior.
States: SEARCH -> APPROACH -> ALIGN -> ORIENT -> SHOOT -> RESET
"""

from enum import Enum
import time


class State(Enum):
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    ALIGN = "ALIGN"
    ORIENT = "ORIENT"
    SHOOT = "SHOOT"
    RESET = "RESET"


class BoatFSM:
    def __init__(self, motor_controller, config):
        self.state = State.SEARCH
        self.motors = motor_controller
        self.config = config
        self.state_entered_at = time.time()
        self.last_ball_seen_at = None
        self.selected_goal_color = "red"  # updated externally via set_goal_color()

    def set_goal_color(self, color):
        """Called from outside (remote_control.py, reading the goal-select
        switch) — can change at any time, including mid-pursuit."""
        assert color in ("red", "blue"), f"unknown goal color: {color}"
        self.selected_goal_color = color

    def _transition(self, new_state):
        if new_state != self.state:
            print(f"[FSM] {self.state.value} -> {new_state.value}")
            self.state = new_state
            self.state_entered_at = time.time()

    def time_in_state(self):
        return time.time() - self.state_entered_at

    def update(self, vision_result, goal_vision_result=None):
        """
        vision_result: {"red": {...}, "colored": {...}}, each shaped
            {"ball_found": bool, "x_offset": float, "distance_est": float}
        goal_vision_result: {"red": {...}, "blue": {...}}, each shaped
            {"found": bool, "x_offset": float, "area_px": float}, or None
            if the caller has no GoalVision wired up (e.g. dev/test scripts
            that predate goal detection) — ORIENT then just behaves as if
            the goal is never found, same as a real goal-detection miss,
            and times out back to SEARCH per its normal timeout rule.
        x_offset: -1.0 (far left) to 1.0 (far right), 0 = centered
        distance_est: rough proxy (e.g. ball radius in px, bigger = closer)

        Colored ball is worth 2x the red ball (more with full autonomy),
        so it's preferred whenever both are visible.
        """
        ball_target = self._select_target(vision_result)

        if self.state == State.SEARCH:
            self._do_search(ball_target)
        elif self.state == State.APPROACH:
            self._do_approach(ball_target)
        elif self.state == State.ALIGN:
            self._do_align(ball_target)
        elif self.state == State.ORIENT:
            self._do_orient(ball_target, goal_vision_result)
        elif self.state == State.SHOOT:
            self._do_shoot(ball_target)
        elif self.state == State.RESET:
            self._do_reset(ball_target)

    @staticmethod
    def _select_target(vision_result):
        """Prefer the colored ball (worth more) whenever it's visible."""
        if vision_result["colored"]["ball_found"]:
            return vision_result["colored"]
        return vision_result["red"]

    # ---- state behaviors ----

    def _do_search(self, v):
        if v["ball_found"]:
            self.last_ball_seen_at = time.time()
            self._transition(State.APPROACH)
            return
        # rotate slowly to scan; reverse sweep direction periodically
        self.motors.rotate_in_place(self.config.SEARCH_ROTATE_SPEED)

    def _do_approach(self, v):
        if not v["ball_found"]:
            # lost the ball — give it a short grace period before re-searching
            if time.time() - self.last_ball_seen_at > self.config.LOST_BALL_TIMEOUT:
                self._transition(State.SEARCH)
            return

        self.last_ball_seen_at = time.time()

        if v["distance_est"] > self.config.ALIGN_DISTANCE_THRESHOLD:
            self._transition(State.ALIGN)
            return

        # steer toward ball while moving forward
        self.motors.drive_toward(v["x_offset"], speed=self.config.APPROACH_SPEED)

    def _do_align(self, v):
        if not v["ball_found"]:
            self._transition(State.SEARCH)
            return

        if abs(v["x_offset"]) < self.config.ALIGN_TOLERANCE:
            self._transition(State.ORIENT)
            return

        self.motors.rotate_in_place(
            self.config.ALIGN_ROTATE_SPEED * (1 if v["x_offset"] > 0 else -1)
        )

    def _do_orient(self, v, goal_vision_result):
        """Bow is presumed against the ball (reached via ALIGN). Rotate
        toward the selected goal — since the bow is already touching the
        ball, rotating the boat drags/turns the ball with it — until the
        goal is centered, then commit to SHOOT.

        Confirmed behavior for the two edge cases (2026-10-12):
          - Ball lost mid-ORIENT -> SEARCH (same as ALIGN's existing rule).
          - Goal never found within ORIENT_GOAL_TIMEOUT_SEC -> SEARCH,
            abandoning possession rather than risk kicking blind (own-goal
            risk flagged in TODO.md).
        """
        if not v["ball_found"]:
            self._transition(State.SEARCH)
            return

        goal = (goal_vision_result or {}).get(self.selected_goal_color, {"found": False, "x_offset": 0.0})

        if not goal["found"]:
            if self.time_in_state() > self.config.ORIENT_GOAL_TIMEOUT_SEC:
                self._transition(State.SEARCH)
                return
            # keep slowly turning (still holding the ball on the bow) while
            # scanning for the goal to come into view
            self.motors.rotate_in_place(self.config.ORIENT_SEARCH_ROTATE_SPEED)
            return

        if abs(goal["x_offset"]) < self.config.ORIENT_GOAL_TOLERANCE:
            self._transition(State.SHOOT)
            return

        self.motors.rotate_in_place(
            self.config.ORIENT_ROTATE_SPEED * (1 if goal["x_offset"] > 0 else -1)
        )

    def _do_shoot(self, v):
        self.motors.kick()
        self._transition(State.RESET)

    def _do_reset(self, v):
        self.motors.stop()
        if self.time_in_state() > self.config.RESET_PAUSE_SEC:
            self._transition(State.SEARCH)
