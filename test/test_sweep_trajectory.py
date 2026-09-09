"""Unit tests for arm-driven sweep generation (ARM_SWEEP_PLAN §4.1, §7.B, §11.5 S4).

The sweep is sent to a controller with **no path tolerance**, at a moment when the
sensor plate is pressed against a wall and every collision checker has been
deliberately taken out of the loop. Validation is therefore the only thing
standing between a bad IK chain and the plate being dragged across the wall, so
these tests are mostly about what gets REJECTED.

Offline: pure numpy against the real UR10e DH table, no ROS and no simulator.

Run with:

    python3 -m pytest test/test_sweep_trajectory.py -v
"""

import numpy as np
import pytest

from planner.planner_lib.ur10e_kinematics import fk
from planner.planner_lib.sweep_trajectory import (
    SweepPlanError,
    blend_rotations,
    interpolation_error,
    joint_velocities,
    plan_sweep,
    sample_line,
    solve_ik_chain,
)

# A configuration with the plate out in front of the arm, of the kind
# `press_prepare` leaves behind.
Q_PRESSED = np.array([0.1, -1.2, 1.4, -1.6, -1.57, 0.3])
T_PRESSED = fk(Q_PRESSED)
ROTATION = T_PRESSED[:3, :3]
P_CENTRE = T_PRESSED[:3, 3]
# Lateral direction: horizontal and perpendicular to the arm's reach direction.
TANGENT = np.array([0.0, 1.0, 0.0])


def partition(length):
    """A partition of ``length`` centred on where the arm currently is."""
    return P_CENTRE - TANGENT * length / 2.0, P_CENTRE + TANGENT * length / 2.0


def lead_in_then_sweep(length, **kwargs):
    """The real two-step flow: the base parks at the partition CENTRE, so the arm
    must travel to the partition start before it can sweep across."""
    p_start, p_end = partition(length)
    lead = plan_sweep(P_CENTRE, p_start, ROTATION, Q_PRESSED, **kwargs)
    sweep = plan_sweep(p_start, p_end, ROTATION, lead.q[-1], **kwargs)
    return lead, sweep


# ---------------------------------------------------------------------------
# sample_line
# ---------------------------------------------------------------------------

def test_samples_include_both_endpoints():
    s = sample_line([0.0, 0.0, 0.0], [0.9, 0.0, 0.0], 0.03)
    assert s[0] == pytest.approx(0.0)
    assert s[-1] == pytest.approx(0.9)


def test_samples_are_evenly_spaced_at_or_below_the_requested_spacing():
    """An undersized final step would give the last waypoint a different velocity
    profile from every other, right where the plate leaves the wall."""
    s = sample_line([0.0, 0.0, 0.0], [0.8, 0.0, 0.0], 0.03)
    steps = np.diff(s)
    assert np.all(steps <= 0.03 + 1e-9)
    assert steps == pytest.approx(steps[0])


def test_a_degenerate_partition_is_rejected_not_swept():
    with pytest.raises(SweepPlanError) as excinfo:
        sample_line([1.0, 1.0, 1.0], [1.0, 1.0, 1.0], 0.03)
    assert excinfo.value.reason == "degenerate_sweep"


def test_a_non_positive_spacing_is_rejected():
    with pytest.raises(SweepPlanError) as excinfo:
        sample_line([0.0, 0.0, 0.0], [0.5, 0.0, 0.0], 0.0)
    assert excinfo.value.reason == "bad_spacing"


# ---------------------------------------------------------------------------
# Generation
# ---------------------------------------------------------------------------

def test_a_realistic_partition_plans_with_margin():
    _, sweep = lead_in_then_sweep(0.8)
    assert len(sweep) > 25
    assert sweep.max_interpolation_error < 0.001      # sub-millimetre at 3 cm
    assert sweep.peak_velocity_ratio < 0.2            # nowhere near rated speed
    assert sweep.warnings == []


def test_every_waypoint_lands_exactly_on_the_commanded_line():
    """The Cartesian half of the guarantee: waypoints are on the line by
    construction, and FK must confirm the IK actually achieved them."""
    _, sweep = lead_in_then_sweep(0.8)
    for i in range(len(sweep)):
        assert fk(sweep.q[i])[:3, 3] == pytest.approx(sweep.tcp[i], abs=1e-9)


def test_orientation_is_held_constant_across_the_sweep():
    """RX/RY are not force-compliant, so the commanded orientation is the only
    thing keeping the plate flat against the wall (§4.2)."""
    _, sweep = lead_in_then_sweep(0.8)
    for q_i in sweep.q:
        assert fk(q_i)[:3, :3] == pytest.approx(ROTATION, abs=1e-9)


def test_timing_gives_a_constant_wall_tangent_speed():
    """`t_i = s_i / speed` is what makes `speed` the single control over how fast
    the plate crosses the wall (ARM_SWEEP_PLAN §9.4)."""
    _, sweep = lead_in_then_sweep(0.8, speed=0.05)
    speeds = np.linalg.norm(np.diff(sweep.tcp, axis=0), axis=1) / np.diff(sweep.times)
    assert speeds == pytest.approx(0.05, abs=1e-9)
    assert sweep.duration == pytest.approx(0.8 / 0.05, abs=1e-9)


def test_speed_is_the_only_thing_that_changes_when_speed_changes():
    """Halving the speed must double the duration and leave the geometry alone --
    otherwise a bench run cannot change sweep speed in isolation."""
    _, slow = lead_in_then_sweep(0.8, speed=0.025)
    _, fast = lead_in_then_sweep(0.8, speed=0.05)
    assert slow.q == pytest.approx(fast.q)
    assert slow.duration == pytest.approx(2.0 * fast.duration)
    assert slow.qdot == pytest.approx(0.5 * fast.qdot)


def test_interior_waypoints_carry_through_velocities():
    """The bridge node zeroes every waypoint, turning a sweep into 27 stops. The
    executor exists partly to not do that (§3.2)."""
    _, sweep = lead_in_then_sweep(0.8)
    assert np.all(np.abs(sweep.qdot[1:-1]).max(axis=1) > 1e-6)


def test_the_endpoints_alone_are_deliberate_stops():
    """The arm starts from rest against the wall and must be stationary before
    Force Mode is released."""
    _, sweep = lead_in_then_sweep(0.8)
    assert sweep.qdot[0] == pytest.approx(np.zeros(6))
    assert sweep.qdot[-1] == pytest.approx(np.zeros(6))


def test_joint_velocities_reproduce_the_commanded_cartesian_speed():
    _, sweep = lead_in_then_sweep(0.8, speed=0.05)
    from planner.planner_lib.ur10e_kinematics import jacobian
    for i in range(1, len(sweep) - 1):
        twist = jacobian(sweep.q[i]) @ sweep.qdot[i]
        assert np.linalg.norm(twist[:3]) == pytest.approx(0.05, abs=1e-9)
        assert twist[3:] == pytest.approx(np.zeros(3), abs=1e-9)


def test_ik_seeds_chain_so_the_arm_never_flips_branch():
    """The guarantee from §3.5. Seeding every solve with the same q would let the
    IK pick a different elbow/wrist branch partway across the wall."""
    p_start, p_end = partition(0.8)
    arc = sample_line(p_start, p_end, 0.03)
    direction = (p_end - p_start) / np.linalg.norm(p_end - p_start)
    q = solve_ik_chain(ROTATION, p_start, direction, arc, Q_PRESSED)
    assert np.max(np.abs(np.diff(q, axis=0))) < 0.1


# ---------------------------------------------------------------------------
# Rejection -- the half that actually protects the wall
# ---------------------------------------------------------------------------

def test_an_unreachable_partition_is_rejected():
    """Beyond UR10e reach there is no IK solution, and a partial trajectory must
    never be published."""
    far = P_CENTRE + TANGENT * 3.0
    with pytest.raises(SweepPlanError) as excinfo:
        plan_sweep(P_CENTRE, far, ROTATION, Q_PRESSED)
    assert excinfo.value.reason in ("ik_unreachable", "near_singular", "joint_velocity")


def test_a_sweep_that_does_not_start_where_the_arm_is_gets_rejected():
    """Without the lead-in, executing the sweep would fling the plate sideways
    across the wall to reach its start point."""
    p_start, p_end = partition(0.8)
    with pytest.raises(SweepPlanError) as excinfo:
        plan_sweep(p_start, p_end, ROTATION, Q_PRESSED)
    assert excinfo.value.reason == "seed_not_at_start"
    assert "lead-in" in excinfo.value.detail


def test_an_impossible_speed_is_rejected_before_the_robot_slows_it_down():
    """Above the rated joint speed the robot stretches the trajectory itself, and
    the constant tangent speed the scan depends on quietly stops being constant."""
    with pytest.raises(SweepPlanError) as excinfo:
        lead_in_then_sweep(0.8, speed=5.0)
    assert excinfo.value.reason == "joint_velocity"
    assert "sweep_speed_mps" in excinfo.value.detail


def test_a_non_positive_speed_is_rejected():
    with pytest.raises(SweepPlanError) as excinfo:
        lead_in_then_sweep(0.8, speed=0.0)
    assert excinfo.value.reason == "bad_speed"


def test_coarse_spacing_is_caught_by_the_interpolation_check():
    """Waypoints are on the line by construction, so only the between-waypoint
    check can catch this -- the TCP bows off the wall tangent as spacing grows."""
    # 25 cm spacing still passes the branch-continuity check (0.24 rad steps), so
    # this really is the between-waypoint bow talking and not another limit.
    with pytest.raises(SweepPlanError) as excinfo:
        lead_in_then_sweep(0.8, spacing=0.25)
    assert excinfo.value.reason == "interpolation_error"
    assert "waypoint_spacing_m" in excinfo.value.detail


def test_the_default_spacing_leaves_an_order_of_magnitude_of_bow_margin():
    """3 cm spacing bows the TCP ~0.2 mm off the line against a 5 mm limit. Worth
    pinning: this margin is what allows the orientation-replan work of §4.2 to
    subdivide a partial sweep without immediately tripping the check."""
    _, sweep = lead_in_then_sweep(0.8, spacing=0.03)
    assert sweep.max_interpolation_error < 0.0005


def test_a_tight_singularity_threshold_rejects_rather_than_warns():
    with pytest.raises(SweepPlanError) as excinfo:
        lead_in_then_sweep(0.8, min_singular_value=10.0)
    assert excinfo.value.reason == "near_singular"


def test_a_tight_step_limit_rejects_rather_than_warns():
    with pytest.raises(SweepPlanError) as excinfo:
        lead_in_then_sweep(0.8, max_joint_step=1e-4)
    assert excinfo.value.reason in ("seed_not_at_start", "branch_discontinuity")


def test_margins_are_recorded_even_on_success():
    """These are the numbers a bench run needs to tell "passed" from "passed with
    nothing left" when tuning partition length and sweep speed."""
    _, sweep = lead_in_then_sweep(0.8)
    assert sweep.max_joint_step > 0.0
    assert 0.0 < sweep.min_singular_value < float("inf")
    assert sweep.peak_velocity_ratio > 0.0
    assert sweep.max_interpolation_error > 0.0


def test_a_near_limit_sweep_warns_without_failing():
    _, sweep = lead_in_then_sweep(0.8, min_singular_value=0.15)
    assert sweep.warnings
    assert any("singularity" in w for w in sweep.warnings)


# ---------------------------------------------------------------------------
# interpolation_error
# ---------------------------------------------------------------------------

def test_interpolation_error_is_zero_for_a_single_segment_at_its_own_midpoint():
    q = np.vstack((Q_PRESSED, Q_PRESSED))
    tcp = np.vstack((P_CENTRE, P_CENTRE))
    assert interpolation_error(q, tcp) == pytest.approx(0.0, abs=1e-12)


def test_interpolation_error_grows_with_waypoint_spacing():
    p_start, p_end = partition(0.8)
    direction = (p_end - p_start) / np.linalg.norm(p_end - p_start)

    def bow(spacing):
        arc = sample_line(p_start, p_end, spacing)
        q = solve_ik_chain(ROTATION, p_start, direction, arc, Q_PRESSED)
        return interpolation_error(q, p_start + direction * arc[:, None])

    assert bow(0.20) > bow(0.03)


def test_joint_velocities_zero_the_ends_of_any_trajectory():
    q = np.tile(Q_PRESSED, (4, 1))
    qdot = joint_velocities(q, TANGENT, 0.05)
    assert qdot[0] == pytest.approx(np.zeros(6))
    assert qdot[-1] == pytest.approx(np.zeros(6))


# ---------------------------------------------------------------------------
# Orientation blending (ARM_SWEEP_PLAN §4.2, S6)
# ---------------------------------------------------------------------------

def rotated(degrees, axis="x"):
    """ROTATION a small angle off the pressed orientation."""
    from scipy.spatial.transform import Rotation
    return ROTATION @ Rotation.from_euler(axis, degrees, degrees=True).as_matrix()


def test_blend_endpoints_are_exact():
    ends = blend_rotations(ROTATION, rotated(6.0), [0.0, 1.0])
    assert ends[0] == pytest.approx(ROTATION, abs=1e-12)
    assert ends[1] == pytest.approx(rotated(6.0), abs=1e-12)


def test_blend_is_monotonic_and_stays_a_rotation():
    target = rotated(8.0)
    blended = blend_rotations(ROTATION, target, np.linspace(0.0, 1.0, 11))
    angles = []
    for m in blended:
        cos = (np.trace(ROTATION.T @ m) - 1.0) / 2.0
        angles.append(np.rad2deg(np.arccos(np.clip(cos, -1.0, 1.0))))
        assert m.T @ m == pytest.approx(np.eye(3), abs=1e-12)   # still orthonormal
    assert np.all(np.diff(angles) > -1e-9)
    assert angles[-1] == pytest.approx(8.0, abs=1e-6)


def test_fractions_are_clamped_so_the_blend_holds_after_it_completes():
    """Waypoints past the blend distance must sit at the corrected orientation,
    not keep rotating past it."""
    beyond = blend_rotations(ROTATION, rotated(5.0), [1.0, 2.5, 10.0])
    for m in beyond:
        assert m == pytest.approx(rotated(5.0), abs=1e-12)


def test_a_blended_sweep_ends_at_the_corrected_orientation():
    p_start, p_end = partition(0.8)
    lead = plan_sweep(P_CENTRE, p_start, ROTATION, Q_PRESSED)
    target = rotated(4.0)
    sweep = plan_sweep(p_start, p_end, ROTATION, lead.q[-1],
                       rotation_to=target, blend_distance=0.10)
    assert fk(sweep.q[0])[:3, :3] == pytest.approx(ROTATION, abs=1e-9)
    assert fk(sweep.q[-1])[:3, :3] == pytest.approx(target, abs=1e-9)
    assert sweep.rotation == pytest.approx(target)


def test_the_blend_completes_within_the_blend_distance():
    """The correction is eased in over a short lateral distance and then held --
    the plate must not still be rotating for the rest of the partition."""
    p_start, p_end = partition(0.8)
    lead = plan_sweep(P_CENTRE, p_start, ROTATION, Q_PRESSED)
    target = rotated(4.0)
    sweep = plan_sweep(p_start, p_end, ROTATION, lead.q[-1],
                       rotation_to=target, blend_distance=0.10)
    arc = np.linalg.norm(sweep.tcp - sweep.tcp[0], axis=1)
    for i in np.where(arc > 0.10 + 1e-9)[0]:
        assert fk(sweep.q[i])[:3, :3] == pytest.approx(target, abs=1e-9)


def test_a_blended_sweep_still_passes_every_validation_gate():
    """The correction must not smuggle in a trajectory the plain path would have
    rejected: same IK, Jacobian, velocity and interpolation checks."""
    p_start, p_end = partition(0.8)
    lead = plan_sweep(P_CENTRE, p_start, ROTATION, Q_PRESSED)
    sweep = plan_sweep(p_start, p_end, ROTATION, lead.q[-1],
                       rotation_to=rotated(4.0), blend_distance=0.10)
    assert sweep.max_joint_step < 0.35
    assert sweep.min_singular_value > 0.02
    assert sweep.max_interpolation_error < 0.005


def test_an_impossible_correction_is_rejected_not_sent():
    p_start, p_end = partition(0.8)
    lead = plan_sweep(P_CENTRE, p_start, ROTATION, Q_PRESSED)
    with pytest.raises(SweepPlanError):
        plan_sweep(p_start, p_end, ROTATION, lead.q[-1],
                   rotation_to=rotated(80.0), blend_distance=0.01)
