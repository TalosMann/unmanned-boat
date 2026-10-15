"""
Interprets raw i-BUS channel values (from remote_input.IBusReceiver) per
pi_controller_spec.md's channel map and mixing rules.

Kill (CH5) is handled SEPARATELY from mode (CH9), not folded into one
switch: CH5's hardware relays cut ESC power directly, independent of the
Pi — read_kill_active() here is a second, redundant software check, never
the only kill path and never something that overrides the hardware relay.
CH9 only chooses which "brain" drives when NOT killed: MANUAL (this
controller) or AUTONOMOUS (the FSM — the controller then only acts as kill).

All channel-to-function mapping is config-driven (config.REMOTE_CH_*).
"""

from enum import Enum


class RemoteMode(Enum):
    MANUAL = "MANUAL"
    AUTONOMOUS = "AUTONOMOUS"


def _switch_position(value_us, config):
    """Returns "low", "mid", or "high" for a 2 or 3-position switch channel."""
    if value_us >= config.REMOTE_SWITCH_HIGH_THRESHOLD_US:
        return "high"
    if value_us <= config.REMOTE_SWITCH_LOW_THRESHOLD_US:
        return "low"
    return "mid"


def _stick_to_unit(value_us, config):
    """Maps a stick channel's raw microsecond value to -1.0..1.0, with a
    small deadzone around center so it reads exactly 0.0 at rest."""
    centered = value_us - config.REMOTE_US_MID
    if abs(centered) < config.REMOTE_STICK_DEADZONE_US:
        return 0.0
    span = config.REMOTE_US_MAX - config.REMOTE_US_MID
    return max(-1.0, min(1.0, centered / span))


def read_kill_active(channels, config):
    """True if CH5 (SwA) is in the relays-off position. Software treats
    this as a hard stop too — redundant with, never a substitute for, the
    independent hardware relay on the same channel. Polarity (which raw
    position means "off") is config.REMOTE_KILL_OFF_IS_HIGH — confirm by
    physically flipping SwA and watching/listening for the relay."""
    position = _switch_position(channels[config.REMOTE_CH_KILL], config)
    off_position = "high" if config.REMOTE_KILL_OFF_IS_HIGH else "low"
    return position == off_position


def read_control_mode(channels, config):
    """CH9 (SwD), 2-position: which brain drives when not killed. In
    AUTONOMOUS, this controller does not drive movement at all — only the
    kill check above still applies."""
    position = _switch_position(channels[config.REMOTE_CH_CONTROL_MODE], config)
    return RemoteMode.AUTONOMOUS if position == "high" else RemoteMode.MANUAL


def read_power_cap(channels, config):
    """CH8 (SwC), 3-position -> one of config.POWER_CAP_LEVELS (low/mid/high)."""
    position = _switch_position(channels[config.REMOTE_CH_POWER_CAP], config)
    index = {"low": 0, "mid": 1, "high": 2}[position]
    return config.POWER_CAP_LEVELS[index]


def read_kick_trigger(channels, config):
    """CH7 (SwB): True while the kick switch is held/flipped to trigger position."""
    return _switch_position(channels[config.REMOTE_CH_KICK_TRIGGER], config) == "high"


def read_manual_mix(channels, config, power_cap):
    """forward = CH2, turn = CH1 -> left = forward+turn, right = forward-turn,
    clamped to -1..1, then scaled by the power cap. Does NOT apply the
    per-thruster invert flags or slew-rate limiting — those are
    motor_control.py's/the caller's job, this is pure stick-to-target-speed
    mixing."""
    forward = _stick_to_unit(channels[config.REMOTE_CH_FORWARD], config)
    turn = _stick_to_unit(channels[config.REMOTE_CH_TURN], config)

    left = max(-1.0, min(1.0, forward + turn)) * power_cap
    right = max(-1.0, min(1.0, forward - turn)) * power_cap
    return left, right


def read_goal_color(channels, config):
    """Not part of pi_controller_spec.md's channel map — left over from
    earlier goal-select work, config.REMOTE_CH_GOAL_SELECT is currently
    None (unassigned) and this is unused by test_rc_control.py. Kept for
    when a channel is identified for future autonomous-mode integration."""
    if config.REMOTE_CH_GOAL_SELECT is None:
        raise RuntimeError("REMOTE_CH_GOAL_SELECT is unassigned — not usable yet")
    position = _switch_position(channels[config.REMOTE_CH_GOAL_SELECT], config)
    return "blue" if position == "high" else "red"
