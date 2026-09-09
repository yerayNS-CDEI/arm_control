"""Joystick jogging for the UR10e, in the analytic IK's own DH convention.

Everything here answers one question: the operator pushed a control, how fast
should each joint turn? It is split out of the ROS node for the same reason
``sweep_trajectory`` is split out of ``wall_sweep_executor`` -- the parts that
can put the plate through a wall should be testable without a robot, a joystick,
or a running graph (ARM_SWEEP_PLAN §11.5 S4).

Two things in here are easy to get wrong and expensive to get wrong on hardware:

1. ``ur10e_kinematics.jacobian`` describes the *DH end frame*, which is not
   ``arm_tool0`` -- roughly 0.3 m and 120 deg away on this robot. A twist the
   operator means at the TCP has to be carried over to that frame before the
   Jacobian sees it, or "rotate about the tool" swings the tool through an arc.
   That is :func:`twist_at_dh_end`.

2. The teach pendant rotates *about the TCP* in both its Base and Tool modes;
   only the axes the arrows refer to change. Keeping that true is what lets the
   two cartesian jog modes share one control table.

Pure numpy, no ROS, no robot. Run the tests with:

    python3 -m pytest test/test_jog.py -v
"""

import numpy as np

from planner.planner_lib.ur10e_kinematics import (
    JOINT_LIMIT_RAD,
    JOINT_VELOCITY_LIMIT_RAD_S,
    joint_velocity_for_twist,
    min_singular_value,
)

# Jog modes. The mode is whichever gamepad button is held, so there is no latched
# state to get out of sync with what the operator believes is selected.
MODE_NONE = "none"
MODE_JOINT = "joint"
MODE_CARTESIAN_TCP = "cartesian_tcp"
MODE_CARTESIAN_BASE = "cartesian_base"


# ---------------------------------------------------------------------------
# Reading the gamepad
# ---------------------------------------------------------------------------

class AxisSource:
    """One DOF driven by a joystick or dpad axis.

    ``axis:1`` reads axis 1; ``-axis:1`` inverts it, which is how "push forward
    is positive" gets expressed for the sticks, whose Y axes report the opposite.
    """

    def __init__(self, index, sign=1.0):
        self.index = index
        self.sign = sign

    def value(self, axes, buttons, deadzone, proportional):
        del buttons
        if self.index >= len(axes):
            return 0.0
        raw = self.sign * axes[self.index]
        if abs(raw) < deadzone:
            return 0.0
        # Fixed-rate jog by default: an axis past the deadzone means full speed in
        # that direction, exactly like the pendant's arrows. The proportional path
        # rescales what is left of the travel so the response still starts at zero
        # rather than jumping to the deadzone fraction.
        if not proportional:
            return float(np.sign(raw))
        span = 1.0 - deadzone
        if span <= 0.0:
            return float(np.sign(raw))
        return float(np.sign(raw) * min(1.0, (abs(raw) - deadzone) / span))


class ButtonSource:
    """One DOF driven by a pair of buttons, negative first.

    ``btn:6,7`` is LT/RT. Buttons are digital, so this is always fixed-rate;
    pressing both at once cancels rather than picking a winner.
    """

    def __init__(self, negative, positive):
        self.negative = negative
        self.positive = positive

    def value(self, axes, buttons, deadzone, proportional):
        del axes, deadzone, proportional
        neg = self.negative < len(buttons) and buttons[self.negative]
        pos = self.positive < len(buttons) and buttons[self.positive]
        return float(bool(pos)) - float(bool(neg))


def parse_source(spec):
    """Parse one mapping entry: ``axis:4``, ``-axis:1`` or ``btn:6,7``.

    Strings rather than nested structures because ROS 2 parameters are flat
    typed values -- a list of dicts cannot survive the round trip through a
    params file, and two parallel integer arrays would not be readable.
    """
    text = str(spec).strip()
    sign = 1.0
    if text.startswith("-"):
        sign = -1.0
        text = text[1:].strip()
    if ":" not in text:
        raise ValueError(f"jog mapping '{spec}' is missing its 'axis:' or 'btn:' prefix")

    kind, _, rest = text.partition(":")
    kind = kind.strip().lower()
    parts = [p for p in rest.replace(",", " ").split() if p]

    if kind == "axis":
        if len(parts) != 1:
            raise ValueError(f"jog mapping '{spec}' must name exactly one axis")
        return AxisSource(int(parts[0]), sign)
    if kind in ("btn", "button", "buttons"):
        if len(parts) != 2:
            raise ValueError(f"jog mapping '{spec}' must name two buttons, negative first")
        negative, positive = int(parts[0]), int(parts[1])
        if sign < 0:
            negative, positive = positive, negative
        return ButtonSource(negative, positive)
    raise ValueError(f"jog mapping '{spec}' has unknown source kind '{kind}'")


def parse_mapping(specs):
    """Parse the six mapping entries of one mode, in DOF order."""
    if len(specs) != 6:
        raise ValueError(f"a jog mode needs exactly 6 mappings, got {len(specs)}")
    return [parse_source(s) for s in specs]


def read_command(sources, axes, buttons, deadzone=0.3, proportional=False):
    """Six-vector in [-1, 1] for one mode, in that mode's DOF order."""
    return np.array(
        [s.value(axes, buttons, deadzone, proportional) for s in sources],
        dtype=float,
    )


def rising_edge(buttons, previous, index):
    """True only on the press.

    ``joy_linux`` autorepeats at 20 Hz, so anything that steps a value -- the
    speed scale, an arm toggle -- has to act on the edge or a held button runs it
    end to end in half a second.
    """
    if index < 0 or index >= len(buttons) or not buttons[index]:
        return False
    return index >= len(previous) or not previous[index]


def active_mode(buttons, mode_buttons, blocking_buttons=()):
    """The single held mode button, else :data:`MODE_NONE`.

    Zero held means the operator let go, and several held means they cannot have
    meant any one of them -- both stop the arm. Ambiguity resolving to "stop" is
    the only safe direction here; picking a winner would move the arm in a way
    nobody asked for.

    ``blocking_buttons`` are the other consumers' enables (the base's, today), so
    driving the base can never also jog the arm.
    """
    if any(b < len(buttons) and buttons[b] for b in blocking_buttons):
        return MODE_NONE
    held = [mode for mode, button in mode_buttons.items()
            if button < len(buttons) and buttons[button]]
    return held[0] if len(held) == 1 else MODE_NONE


# ---------------------------------------------------------------------------
# Turning a command into joint velocities
# ---------------------------------------------------------------------------

def twist_at_dh_end(linear, angular, rotation, tcp_position, dh_end_position):
    """Operator twist -> twist of the DH end frame, both expressed in ``arm_base``.

    ``rotation`` takes the command frame into ``arm_base``: identity for base-frame
    jogging, ``R_base_tool0`` for tool-frame jogging. The cross-product term is the
    part that matters -- it carries a rotation commanded about the TCP over to the
    frame the Jacobian actually describes, so "spin about the tool" spins in place
    instead of swinging the tool around a point 0.3 m away.
    """
    v = np.asarray(rotation, dtype=float) @ np.asarray(linear, dtype=float)
    w = np.asarray(rotation, dtype=float) @ np.asarray(angular, dtype=float)
    lever = np.asarray(dh_end_position, dtype=float) - np.asarray(tcp_position, dtype=float)
    return np.concatenate((v + np.cross(w, lever), w))


def singularity_scale(sigma, slow_below, stop_below):
    """Speed factor in [0, 1] from the Jacobian's smallest singular value.

    Below ``stop_below`` the arm has effectively lost a Cartesian DOF and the
    joint speeds needed to keep the TCP moving blow up, so refuse. The linear
    ramp above it is there to make that refusal felt as the jog going heavy
    rather than as a sudden stop the operator cannot explain.
    """
    if not np.isfinite(sigma) or sigma <= stop_below:
        return 0.0
    if sigma >= slow_below:
        return 1.0
    return float((sigma - stop_below) / (slow_below - stop_below))


def clamp_joint_velocity(qdot, max_fraction):
    """Scale ``qdot`` as a whole to fit inside a fraction of the rated speeds.

    Scaled together rather than clipped per joint: clipping one joint of a
    coordinated Cartesian solution changes the direction the TCP travels, which
    is how a jog ends up somewhere other than along the axis the operator chose.
    Same idiom as ``sweep_trajectory``'s velocity check.
    """
    limit = JOINT_VELOCITY_LIMIT_RAD_S * float(max_fraction)
    ratio = float(np.max(np.abs(np.asarray(qdot, dtype=float)) / limit))
    if ratio > 1.0:
        return np.asarray(qdot, dtype=float) / ratio
    return np.asarray(qdot, dtype=float)


def block_at_joint_limits(q, qdot, dt, margin):
    """Zero any joint that would be driven further outside its limits.

    Outward motion only -- a joint parked on its limit must still be jogged back,
    or the only way out of the corner is the teach pendant.
    """
    q = np.asarray(q, dtype=float)
    qdot = np.asarray(qdot, dtype=float).copy()
    horizon = q + qdot * dt
    beyond = np.abs(horizon) > (JOINT_LIMIT_RAD - margin)
    outward = beyond & (np.sign(qdot) == np.sign(horizon))
    qdot[outward] = 0.0
    return qdot


def ramp(current, target, max_step):
    """Move ``current`` toward ``target`` by at most ``max_step`` per component.

    Fixed-rate jogging is a step input, and a step from zero to full speed is
    what tripped the drive faults on the base (README_Infranor_PDO_Functional
    §11, item 10). The arm gets the same treatment on the way up and down.
    """
    current = np.asarray(current, dtype=float)
    target = np.asarray(target, dtype=float)
    delta = np.clip(target - current, -abs(max_step), abs(max_step))
    return current + delta


def joint_jog_velocity(command, speed, max_fraction):
    """Joint-mode joint velocities: one command per joint, no kinematics needed."""
    qdot = np.asarray(command, dtype=float) * float(speed)
    return clamp_joint_velocity(qdot, max_fraction)


def cartesian_jog_velocity(
    q,
    command,
    rotation,
    tcp_position,
    dh_end_position,
    linear_speed,
    angular_speed,
    max_fraction,
    slow_below,
    stop_below,
):
    """Cartesian-mode joint velocities, plus why they came out that way.

    Returns ``(qdot, sigma, scale)`` so the caller can tell the operator that a
    jog went slow because of a nearby singularity rather than a dead joystick.
    """
    command = np.asarray(command, dtype=float)
    twist = twist_at_dh_end(
        command[:3] * float(linear_speed),
        command[3:] * float(angular_speed),
        rotation,
        tcp_position,
        dh_end_position,
    )

    sigma = min_singular_value(q)
    scale = singularity_scale(sigma, slow_below, stop_below)
    if scale <= 0.0:
        return np.zeros(6), sigma, 0.0

    qdot = joint_velocity_for_twist(q, twist) * scale
    return clamp_joint_velocity(qdot, max_fraction), sigma, scale
