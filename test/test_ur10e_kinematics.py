"""Unit tests for the shared UR10e kinematics.

The point of this module is that FK, the Jacobian and the analytic IK all speak
the SAME DH convention. That is not obvious and not free: the IK solves for a DH
end frame that differs from the URDF's `arm_tool0` by a fixed rigid transform, so
a convention slip here puts the sensor plate somewhere other than where the sweep
thinks it is -- and against a wall, "somewhere other" means into it.

Pure numpy, no ROS, no robot. Run with:

    python3 -m pytest test/test_ur10e_kinematics.py -v
"""

import numpy as np
import pytest

from planner.planner_lib.closed_form_algorithm import closed_form_algorithm
from planner.planner_lib.ur10e_kinematics import (
    JOINT_VELOCITY_LIMIT_RAD_S,
    calibrate_tool0_to_dh_end,
    fk,
    fk_chain,
    jacobian,
    joint_velocity_for_twist,
    min_singular_value,
    pose_matrix,
)

# A few well-conditioned configurations away from the workspace boundary and
# from the wrist/shoulder singularities.
POSES = [
    np.array([0.1, -1.2, 1.4, -1.6, -1.57, 0.3]),
    np.array([-0.6, -1.0, 1.1, -1.2, -1.57, 0.0]),
    np.array([0.9, -1.5, 1.8, -1.9, -1.4, 1.2]),
]


@pytest.mark.parametrize("q", POSES)
def test_fk_matches_the_ik_convention(q):
    """The invariant the whole module exists for: IK(FK(q)) reproduces the pose.

    If FK used the URDF's tool0 frame instead of the IK's DH end frame, this
    roundtrip would come back offset by the flange transform -- about 0.3 m.
    """
    T = fk(q)
    q_back = closed_form_algorithm(T, q, type=0)
    assert not np.any(np.isnan(q_back))
    assert fk(q_back) == pytest.approx(T, abs=1e-9)


@pytest.mark.parametrize("q", POSES)
def test_jacobian_matches_numerical_differentiation(q):
    """Linear rows against finite differences of FK; the analytic form is what
    turns a commanded sweep speed into joint velocities, so a wrong column shows
    up as a plate that crosses the wall at the wrong rate."""
    analytic = jacobian(q)[:3]
    eps = 1e-6
    p0 = fk(q)[:3, 3]
    numeric = np.column_stack([
        (fk(q + eps * np.eye(6)[i])[:3, 3] - p0) / eps for i in range(6)
    ])
    assert analytic == pytest.approx(numeric, abs=1e-5)


@pytest.mark.parametrize("q", POSES)
def test_angular_jacobian_rows_are_the_joint_axes(q):
    transforms = fk_chain(q)
    for i in range(6):
        assert jacobian(q)[3:, i] == pytest.approx(transforms[i][:3, 2], abs=1e-12)


def test_fk_chain_returns_identity_then_every_link():
    q = POSES[0]
    chain = fk_chain(q)
    assert len(chain) == 7
    assert chain[0] == pytest.approx(np.eye(4))
    assert chain[-1] == pytest.approx(fk(q))


@pytest.mark.parametrize("q", POSES)
def test_joint_velocity_reproduces_the_requested_twist(q):
    """qdot = J^+ V must actually produce V again -- otherwise the sweep is timed
    for a speed the arm is not moving at."""
    twist = np.array([0.0, 0.05, 0.0, 0.0, 0.0, 0.0])
    qdot = joint_velocity_for_twist(q, twist)
    assert jacobian(q) @ qdot == pytest.approx(twist, abs=1e-9)
    # A 5 cm/s sweep is a gentle motion; anything near the rated speed here would
    # mean the configuration is far worse conditioned than it looks.
    assert np.all(np.abs(qdot) < 0.25 * JOINT_VELOCITY_LIMIT_RAD_S)


def test_min_singular_value_collapses_at_a_wrist_singularity():
    """Sanity check that the conditioning metric detects the thing it exists for:
    with q5 = 0 the wrist axes align and the arm loses a Cartesian DOF."""
    healthy = np.array([0.1, -1.2, 1.4, -1.6, -1.57, 0.3])
    singular = healthy.copy()
    singular[4] = 0.0
    assert min_singular_value(singular) < 0.1 * min_singular_value(healthy)


def test_tool0_calibration_recovers_a_known_offset():
    """The calibration must invert exactly, or every sweep is biased by whatever
    it gets wrong -- and it is applied to every waypoint."""
    q = POSES[0]
    known_offset = pose_matrix(
        np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
        [0.0, 0.0, 0.3],
    )
    T_base_tool0 = fk(q) @ np.linalg.inv(known_offset)
    assert calibrate_tool0_to_dh_end(T_base_tool0, q) == pytest.approx(known_offset, abs=1e-12)


def test_pose_matrix_assembles_rotation_and_position():
    T = pose_matrix(np.eye(3), [1.0, 2.0, 3.0])
    assert T[:3, 3] == pytest.approx([1.0, 2.0, 3.0])
    assert T[3] == pytest.approx([0.0, 0.0, 0.0, 1.0])
