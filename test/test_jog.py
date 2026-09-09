"""Unit tests for joystick jogging.

The two that matter are at the bottom: a jog commanded along an axis has to move
the TCP along *that* axis, and a rotation commanded about the tool has to spin
the tool in place. Both go wrong silently, because the Jacobian describes the
analytic IK's DH end frame rather than the URDF's `arm_tool0` -- 0.3 m and 120
deg away on this robot -- so an untransported twist produces a plausible-looking
motion in the wrong place.

Pure numpy, no ROS, no robot. Run with:

    python3 -m pytest test/test_jog.py -v
"""

import numpy as np
import pytest

from planner.planner_lib.jog import (
    MODE_CARTESIAN_BASE,
    MODE_CARTESIAN_TCP,
    MODE_JOINT,
    MODE_NONE,
    AxisSource,
    ButtonSource,
    active_mode,
    block_at_joint_limits,
    cartesian_jog_velocity,
    clamp_joint_velocity,
    joint_jog_velocity,
    parse_mapping,
    parse_source,
    ramp,
    read_command,
    rising_edge,
    singularity_scale,
    twist_at_dh_end,
)
from planner.planner_lib.ur10e_kinematics import (
    JOINT_LIMIT_RAD,
    JOINT_VELOCITY_LIMIT_RAD_S,
    fk,
    jacobian,
    pose_matrix,
)

# Well-conditioned configurations, away from the workspace boundary and from the
# wrist/shoulder singularities. Same set the kinematics tests use.
POSES = [
    np.array([0.1, -1.2, 1.4, -1.6, -1.57, 0.3]),
    np.array([-0.6, -1.0, 1.1, -1.2, -1.57, 0.0]),
    np.array([0.9, -1.5, 1.8, -1.9, -1.4, 1.2]),
]

# A stand-in for the measured tool0 -> DH end offset: a real displacement and a
# real rotation, so a test that ignores either one fails.
T_TOOL0_DHEND = pose_matrix(
    np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
    [0.0, 0.0, 0.3],
)

F710_AXES = 6
F710_BUTTONS = 12


def joy(axes=None, buttons=None):
    a = [0.0] * F710_AXES
    b = [0] * F710_BUTTONS
    for i, v in (axes or {}).items():
        a[i] = v
    for i in buttons or ():
        b[i] = 1
    return a, b


def frames(q):
    """``(R_base_tool0, p_tool0, p_dh_end)``, as the node computes them."""
    T_base_dhend = fk(q)
    T_base_tool0 = T_base_dhend @ np.linalg.inv(T_TOOL0_DHEND)
    return T_base_tool0[:3, :3], T_base_tool0[:3, 3], T_base_dhend[:3, 3]


def tcp_velocity(q, qdot, p_tcp):
    """Velocity of the TCP point, from the joint velocities the jog produced.

    Deliberately re-derived through ``jacobian`` rather than through anything in
    ``jog``: the test must be able to disagree with the code under test.
    """
    twist = jacobian(q) @ qdot
    v, w = twist[:3], twist[3:]
    return v + np.cross(w, p_tcp - fk(q)[:3, 3]), w


# ---------------------------------------------------------------------------
# Reading the gamepad
# ---------------------------------------------------------------------------

def test_parse_source_reads_every_supported_form():
    assert isinstance(parse_source("axis:4"), AxisSource)
    assert parse_source("axis:4").sign == 1.0
    assert parse_source("-axis:1").sign == -1.0
    assert isinstance(parse_source("btn:6,7"), ButtonSource)
    assert (parse_source("btn:6,7").negative, parse_source("btn:6,7").positive) == (6, 7)
    # Inverting a button pair swaps which one is negative.
    assert (parse_source("-btn:6,7").negative, parse_source("-btn:6,7").positive) == (7, 6)


@pytest.mark.parametrize("spec", ["axis", "axis:1,2", "btn:6", "dpad:1", ""])
def test_parse_source_rejects_malformed_mappings(spec):
    """A mapping that silently half-parses would put the operator's buttons on
    something other than what the config says."""
    with pytest.raises(ValueError):
        parse_source(spec)


def test_parse_mapping_requires_all_six_dofs():
    with pytest.raises(ValueError):
        parse_mapping(["axis:0"] * 5)


def test_fixed_rate_axes_are_full_speed_past_the_deadzone():
    sources = parse_mapping(["axis:0"] * 6)
    axes, buttons = joy(axes={0: 0.45})
    assert read_command(sources, axes, buttons, deadzone=0.3)[0] == 1.0
    axes, buttons = joy(axes={0: -1.0})
    assert read_command(sources, axes, buttons, deadzone=0.3)[0] == -1.0


def test_deadzone_holds_the_arm_still():
    sources = parse_mapping(["axis:0"] * 6)
    axes, buttons = joy(axes={0: 0.29})
    assert not np.any(read_command(sources, axes, buttons, deadzone=0.3))


def test_proportional_axes_start_from_zero_at_the_deadzone():
    """Without rescaling, crossing the deadzone would jump straight to 30% speed."""
    sources = parse_mapping(["axis:0"] * 6)
    axes, buttons = joy(axes={0: 0.3001})
    value = read_command(sources, axes, buttons, deadzone=0.3, proportional=True)[0]
    assert value == pytest.approx(0.0, abs=1e-3)
    axes, buttons = joy(axes={0: 1.0})
    assert read_command(sources, axes, buttons, 0.3, proportional=True)[0] == 1.0


def test_button_pairs_are_signed_and_cancel_when_both_are_held():
    sources = parse_mapping(["btn:6,7"] * 6)
    assert read_command(sources, *joy(buttons=(7,)))[0] == 1.0
    assert read_command(sources, *joy(buttons=(6,)))[0] == -1.0
    assert read_command(sources, *joy(buttons=(6, 7)))[0] == 0.0


def test_the_shipped_f710_mapping_covers_six_independent_controls():
    """Every DOF must be reachable, and no two may answer to the same input --
    a duplicate would move two axes at once without the operator asking."""
    for specs in (
        ["btn:6,7", "btn:4,5", "axis:0", "axis:1", "axis:4", "axis:5"],
        ["axis:4", "axis:5", "btn:4,5", "axis:1", "axis:0", "btn:6,7"],
    ):
        sources = parse_mapping(specs)
        seen = set()
        for source in sources:
            key = (("axis", source.index) if isinstance(source, AxisSource)
                   else ("btn", source.negative, source.positive))
            assert key not in seen, f"{specs} reuses {key}"
            seen.add(key)
        # Nothing outside what an F710 in DirectInput mode reports.
        for source in sources:
            if isinstance(source, AxisSource):
                assert source.index < F710_AXES
            else:
                assert max(source.negative, source.positive) < F710_BUTTONS


# ---------------------------------------------------------------------------
# Choosing a mode
# ---------------------------------------------------------------------------

# The shipped F710 layout: Y joint, A cartesian tool, B cartesian base.
MODE_BUTTONS = {MODE_JOINT: 3, MODE_CARTESIAN_TCP: 1, MODE_CARTESIAN_BASE: 2}
BASE_ENABLE = [0]                                   # X, held to drive the base


@pytest.mark.parametrize("button,mode", [
    (3, MODE_JOINT), (1, MODE_CARTESIAN_TCP), (2, MODE_CARTESIAN_BASE),
])
def test_each_mode_button_selects_its_mode(button, mode):
    _, buttons = joy(buttons=(button,))
    assert active_mode(buttons, MODE_BUTTONS, BASE_ENABLE) == mode


def test_releasing_every_mode_button_stops_the_arm():
    _, buttons = joy()
    assert active_mode(buttons, MODE_BUTTONS, BASE_ENABLE) == MODE_NONE


def test_two_mode_buttons_stop_rather_than_pick_a_winner():
    """The operator cannot have meant both, and guessing moves the arm in a way
    nobody asked for."""
    _, buttons = joy(buttons=(1, 3))
    assert active_mode(buttons, MODE_BUTTONS, BASE_ENABLE) == MODE_NONE


def test_driving_the_base_blocks_the_arm():
    """X is the base's enable_button. Holding it must never also jog the arm,
    however the arm mode buttons happen to be pressed."""
    for extra in ((), (1,), (3,)):
        _, buttons = joy(buttons=(0,) + extra)
        assert active_mode(buttons, MODE_BUTTONS, BASE_ENABLE) == MODE_NONE


def test_mode_selection_survives_a_pad_reporting_fewer_buttons():
    assert active_mode([0, 0], MODE_BUTTONS, BASE_ENABLE) == MODE_NONE


def test_speed_steps_only_on_the_press():
    """A 20 Hz autorepeat of a held button would otherwise run the speed scale
    from end to end in half a second."""
    _, held = joy(buttons=(9,))
    _, released = joy()
    assert rising_edge(held, released, 9)      # press
    assert not rising_edge(held, held, 9)      # still held, autorepeating
    assert not rising_edge(released, held, 9)  # release
    assert rising_edge(held, [], 9)            # very first message


# ---------------------------------------------------------------------------
# Limits and shaping
# ---------------------------------------------------------------------------

def test_singularity_scale_ramps_rather_than_dropping_out():
    assert singularity_scale(0.10, 0.05, 0.01) == 1.0
    assert singularity_scale(0.005, 0.05, 0.01) == 0.0
    assert singularity_scale(0.03, 0.05, 0.01) == pytest.approx(0.5)
    assert singularity_scale(float("nan"), 0.05, 0.01) == 0.0


def test_clamp_scales_the_whole_vector_so_the_direction_survives():
    """Clipping one joint of a coordinated cartesian solution would bend the path
    the TCP takes; scaling together only makes it slower."""
    qdot = JOINT_VELOCITY_LIMIT_RAD_S * np.array([1.0, 0.5, 0.25, 0.1, 0.1, 0.1])
    clamped = clamp_joint_velocity(qdot, 0.25)
    assert np.max(np.abs(clamped) / JOINT_VELOCITY_LIMIT_RAD_S) == pytest.approx(0.25)
    assert clamped / np.linalg.norm(clamped) == pytest.approx(qdot / np.linalg.norm(qdot))


def test_clamp_leaves_a_command_that_already_fits():
    qdot = JOINT_VELOCITY_LIMIT_RAD_S * 0.1
    assert clamp_joint_velocity(qdot, 0.25) == pytest.approx(qdot)


def test_joint_limits_block_outward_motion_but_allow_the_way_back():
    q = np.zeros(6)
    q[2] = JOINT_LIMIT_RAD - 0.05          # already inside the margin
    outward = np.zeros(6)
    outward[2] = 0.5
    assert block_at_joint_limits(q, outward, 0.01, 0.1)[2] == 0.0
    inward = np.zeros(6)
    inward[2] = -0.5
    assert block_at_joint_limits(q, inward, 0.01, 0.1)[2] == -0.5


def test_joint_limits_leave_the_other_joints_alone():
    q = np.zeros(6)
    q[0] = JOINT_LIMIT_RAD - 0.05
    qdot = np.full(6, 0.5)
    blocked = block_at_joint_limits(q, qdot, 0.01, 0.1)
    assert blocked[0] == 0.0
    assert blocked[1:] == pytest.approx(qdot[1:])


def test_ramp_limits_the_step_and_reaches_the_target():
    assert ramp(np.zeros(6), np.full(6, 1.0), 0.1) == pytest.approx(np.full(6, 0.1))
    assert ramp(np.full(6, 1.0), np.zeros(6), 0.1) == pytest.approx(np.full(6, 0.9))
    assert ramp(np.full(6, 0.05), np.full(6, 0.1), 0.1) == pytest.approx(np.full(6, 0.1))


def test_joint_mode_needs_no_kinematics():
    command = np.array([1.0, 0.0, -1.0, 0.0, 0.0, 0.0])
    qdot = joint_jog_velocity(command, 0.2, 0.25)
    assert qdot[0] == pytest.approx(0.2)
    assert qdot[2] == pytest.approx(-0.2)
    assert not np.any(qdot[[1, 3, 4, 5]])


# ---------------------------------------------------------------------------
# The twist transport, which is the part that goes wrong quietly
# ---------------------------------------------------------------------------

def test_pure_translation_needs_no_transport():
    twist = twist_at_dh_end([1.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                            np.eye(3), [0.0, 0.0, 0.0], [0.0, 0.0, 0.3])
    assert twist == pytest.approx([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_rotation_about_the_tcp_moves_the_dh_end_frame():
    """The whole point: the DH end frame sits 0.3 m from the TCP, so spinning
    about the TCP has to translate it. A transport-free implementation returns
    zero linear velocity here and swings the tool instead."""
    twist = twist_at_dh_end([0.0, 0.0, 0.0], [0.0, 0.0, 1.0],
                            np.eye(3), [0.0, 0.0, 0.0], [0.3, 0.0, 0.0])
    assert twist[:3] == pytest.approx([0.0, 0.3, 0.0])
    assert twist[3:] == pytest.approx([0.0, 0.0, 1.0])


@pytest.mark.parametrize("q", POSES)
@pytest.mark.parametrize("axis", range(3))
def test_base_frame_translation_moves_the_tcp_along_that_axis(q, axis):
    """Push dpad-right in base mode and the TCP travels along base X, at the
    commanded speed, without rotating."""
    command = np.zeros(6)
    command[axis] = 1.0
    _, p_tcp, p_dh_end = frames(q)

    qdot, _, scale = cartesian_jog_velocity(
        q, command, np.eye(3), p_tcp, p_dh_end,
        linear_speed=0.05, angular_speed=0.2, max_fraction=0.5,
        slow_below=0.05, stop_below=0.01)
    assert scale == 1.0

    v, w = tcp_velocity(q, qdot, p_tcp)
    expected = np.zeros(3)
    expected[axis] = 0.05
    assert v == pytest.approx(expected, abs=1e-9)
    assert w == pytest.approx(np.zeros(3), abs=1e-9)


@pytest.mark.parametrize("q", POSES)
@pytest.mark.parametrize("axis", range(3))
def test_tool_frame_rotation_spins_in_place_about_the_tool_axis(q, axis):
    """The pendant rotates about the TCP. So must we: the TCP point stays put and
    the angular velocity lies along the commanded *tool* axis, not the base one."""
    command = np.zeros(6)
    command[3 + axis] = 1.0
    rotation, p_tcp, p_dh_end = frames(q)

    qdot, _, scale = cartesian_jog_velocity(
        q, command, rotation, p_tcp, p_dh_end,
        linear_speed=0.05, angular_speed=0.2, max_fraction=0.5,
        slow_below=0.05, stop_below=0.01)
    assert scale == 1.0

    v, w = tcp_velocity(q, qdot, p_tcp)
    assert v == pytest.approx(np.zeros(3), abs=1e-9)
    assert w == pytest.approx(0.2 * rotation[:, axis], abs=1e-9)


@pytest.mark.parametrize("q", POSES)
def test_tool_frame_translation_follows_the_tool_axes(q):
    """+Z in tool mode is the approach direction, which is a different direction
    in base coordinates for every pose -- that is the whole difference between
    the A and B modes."""
    command = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    rotation, p_tcp, p_dh_end = frames(q)

    qdot, _, _ = cartesian_jog_velocity(
        q, command, rotation, p_tcp, p_dh_end,
        linear_speed=0.05, angular_speed=0.2, max_fraction=0.5,
        slow_below=0.05, stop_below=0.01)

    v, w = tcp_velocity(q, qdot, p_tcp)
    assert v == pytest.approx(0.05 * rotation[:, 2], abs=1e-9)
    assert w == pytest.approx(np.zeros(3), abs=1e-9)


@pytest.mark.parametrize("q", POSES)
def test_a_cartesian_jog_never_exceeds_the_velocity_ceiling(q):
    """Every direction, including the ones that ask most of the wrist."""
    _, p_tcp, p_dh_end = frames(q)
    for dof in range(6):
        command = np.zeros(6)
        command[dof] = 1.0
        qdot, _, _ = cartesian_jog_velocity(
            q, command, np.eye(3), p_tcp, p_dh_end,
            linear_speed=0.5, angular_speed=2.0, max_fraction=0.25,
            slow_below=0.05, stop_below=0.01)
        ratio = np.max(np.abs(qdot) / JOINT_VELOCITY_LIMIT_RAD_S)
        assert ratio <= 0.25 + 1e-9


def test_a_singular_pose_refuses_to_jog():
    """Wrist singularity: wrist_2 at zero lines up wrist_1 and wrist_3."""
    q = np.array([0.0, -1.2, 1.4, -1.6, 0.0, 0.0])
    _, p_tcp, p_dh_end = frames(q)
    qdot, _, scale = cartesian_jog_velocity(
        q, np.array([1.0, 0, 0, 0, 0, 0]), np.eye(3), p_tcp, p_dh_end,
        linear_speed=0.05, angular_speed=0.2, max_fraction=0.25,
        slow_below=0.05, stop_below=0.01)
    assert scale == 0.0
    assert not np.any(qdot)
