"""UR10e kinematics in the analytic IK's own DH convention.

Everything that has to agree on where the arm is must use the *same* DH table.
``closed_form_algorithm`` solves for a DH end-frame that differs from the URDF's
``arm_tool0`` by a fixed rigid transform (roughly a 0.3 m offset and a 120 deg
rotation on this robot), so mixing conventions silently puts the plate in the
wrong place. ``wall_parallel_controller`` already sidesteps that by calibrating
the constant offset from ``FK(q)`` against the live ``tool0`` TF; this module
lifts that trick out of it so the sweep executor can use the identical numbers
(ARM_SWEEP_PLAN §7.B).

Deliberately free of ROS imports so the sweep generator built on top can be
unit-tested offline (ARM_SWEEP_PLAN §11.5 S4).
"""

import numpy as np

# Must match planner_lib.closed_form_algorithm._ur10e_dh_params().
DH_D = (0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655)
DH_A = (0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0)
DH_ALPHA = (np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0)

# UR10e joint limits: every joint is +/- 2 pi mechanically.
JOINT_LIMIT_RAD = 2.0 * np.pi

# UR10e rated joint speeds: 120 deg/s on the three big joints, 180 deg/s on the
# wrist. The sweep must stay inside these at the requested Cartesian speed, or
# the robot slows the whole trajectory down and the "constant speed" the scan
# assumes quietly stops being constant.
JOINT_VELOCITY_LIMIT_RAD_S = np.array([
    2.0944, 2.0944, 2.0944, 3.1416, 3.1416, 3.1416,
])


def dh_link(theta: float, alpha: float, a: float, d: float) -> np.ndarray:
    """Standard Denavit-Hartenberg link transform."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,      ca,      d],
        [0.0,     0.0,     0.0,    1.0],
    ])


def fk_chain(q):
    """Cumulative transforms ``[T_0 .. T_6]``, base to DH end frame.

    ``T_0`` is the identity, so ``T_i`` is the pose of link ``i``'s frame and
    ``T_6`` is the DH end frame. The whole chain is returned (rather than just
    the tip) because the geometric Jacobian needs every joint axis and origin.
    """
    q = np.asarray(q, dtype=float)
    transforms = [np.eye(4)]
    for i in range(6):
        transforms.append(transforms[-1] @ dh_link(q[i], DH_ALPHA[i], DH_A[i], DH_D[i]))
    return transforms


def fk(q) -> np.ndarray:
    """Pose of the DH end frame in ``arm_base`` as a 4x4 homogeneous transform."""
    return fk_chain(q)[-1]


def jacobian(q) -> np.ndarray:
    """Geometric Jacobian (6x6) of the DH end frame, expressed in ``arm_base``.

    Rows are ``[vx, vy, vz, wx, wy, wz]``. For an all-revolute chain, joint ``i``
    contributes ``z_{i-1} x (o_6 - o_{i-1})`` to linear velocity and ``z_{i-1}``
    to angular velocity.

    Used for two things in the sweep: turning the requested Cartesian speed into
    per-waypoint joint velocities (``qdot = J^-1 V``), and flagging waypoints
    near a singularity, where that inverse would demand joint speeds the robot
    cannot deliver.
    """
    transforms = fk_chain(q)
    o_end = transforms[-1][:3, 3]
    columns = []
    for i in range(6):
        z = transforms[i][:3, 2]
        o = transforms[i][:3, 3]
        columns.append(np.concatenate((np.cross(z, o_end - o), z)))
    return np.column_stack(columns)


def min_singular_value(q) -> float:
    """Smallest singular value of the Jacobian: distance to a singularity.

    Near zero the arm loses a Cartesian degree of freedom, and the joint speeds
    needed to keep the TCP moving at the commanded rate blow up. Cheaper to
    reject a partition here than to watch the sweep stall mid-wall.
    """
    return float(np.linalg.svd(jacobian(q), compute_uv=False)[-1])


def joint_velocity_for_twist(q, twist) -> np.ndarray:
    """Joint velocities producing ``twist`` (``[v, omega]``) at the DH end frame.

    Uses a least-squares solve rather than a plain inverse so a rank-deficient
    Jacobian returns the minimum-norm answer instead of raising -- the caller
    catches the real problem through :func:`min_singular_value` and the joint
    velocity limits, which say something actionable about the sweep.
    """
    return np.linalg.lstsq(jacobian(q), np.asarray(twist, dtype=float), rcond=None)[0]


def calibrate_tool0_to_dh_end(T_base_tool0, q) -> np.ndarray:
    """Constant ``tool0 -> DH end frame`` transform, from FK(q) vs a live tool0 TF.

    The one honest way to relate the two conventions: measure them against each
    other on the real robot at a known joint configuration, rather than deriving
    a flange offset from the URDF and hoping it matches the IK's DH table.
    Constant, so one successful lookup holds for the session.
    """
    return np.linalg.inv(np.asarray(T_base_tool0, dtype=float)) @ fk(q)


def pose_matrix(rotation, position) -> np.ndarray:
    """Assemble a 4x4 transform from a 3x3 rotation and a 3-vector position."""
    T = np.eye(4)
    T[:3, :3] = np.asarray(rotation, dtype=float)
    T[:3, 3] = np.asarray(position, dtype=float)
    return T
