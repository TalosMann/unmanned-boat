"""
Finite State Machine for autonomous ball-chasing behavior.
States: SEARCH -> APPROACH -> ALIGN -> SHOOT -> RESET
"""

from enum import Enum
import time


class State(Enum):
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    ALIGN = "ALIGN"
    SHOOT = "SHOOT"
    RESET = "RESET"


class BoatFSM:
    def __init__(self, motor_controller, config):
        self.state = State.SEARCH
        self.motors = motor_controller
        self.config = config
        self.state_entered_at = time.time()
        self.last_ball_seen_at = None

    def _transition(self, new_state):
        if new_state != self.state:
            print(f"[FSM] {self.state.value} -> {new_state.value}")
            self.state = new_state
            self.state_entered_at = time.time()

    def time_in_state(self):
        return time.time() - self.state_entered_at

    def update(self, vision_result):
        """
        vision_result: {"red": {...}, "colored": {...}}, each shaped
            {"ball_found": bool, "x_offset": float, "distance_est": float}
        x_offset: -1.0 (far left) to 1.0 (far right), 0 = centered
        distance_est: rough proxy (e.g. ball radius in px, bigger = closer)

        Colored ball is worth 2x the red ball (more with full autonomy),
        so it's preferred whenever both are visible.
        """
        vision_result = self._select_target(vision_result)

        if self.state == State.SEARCH:
            self._do_search(vision_result)
        elif self.state == State.APPROACH:
            self._do_approach(vision_result)
        elif self.state == State.ALIGN:
            self._do_align(vision_result)
        elif self.state == State.SHOOT:
            self._do_shoot(vision_result)
        elif self.state == State.RESET:
            self._do_reset(vision_result)

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
            self._transition(State.SHOOT)
            return

        self.motors.rotate_in_place(
            self.config.ALIGN_ROTATE_SPEED * (1 if v["x_offset"] > 0 else -1)
        )

    def _do_shoot(self, v):
        self.motors.shoot()
        self._transition(State.RESET)

    def _do_reset(self, v):
        self.motors.stop()
        if self.time_in_state() > self.config.RESET_PAUSE_SEC:
            self._transition(State.SEARCH)
