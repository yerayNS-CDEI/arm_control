"""Generate the arm-driven lateral wall sweep (ARM_SWEEP_PLAN §4.1, §7.B).

The sweep is **computed, not planned**. A search-based planner is the wrong tool
twice over: the plate is deliberately in contact with the wall, so any collision
checker refuses the motion outright, and there is no search problem to begin with
-- the path is a straight line at a known orientation.

So: sample the Cartesian line, run analytic IK per sample chaining the previous
solution as the seed (which is what keeps the arm on one IK branch), and derive
joint velocities from the Jacobian at the requested Cartesian speed.

**Safety comes from validation, not from a collision checker.** Nothing is sent
until the whole trajectory passes: no NaN, joint limits, branch continuity,
Jacobian conditioning, joint velocity limits, and -- the one that is easy to
forget -- the TCP error *between* waypoints. Every waypoint lies exactly on the
requested line by construction, but the robot interpolates in JOINT space, so the
path between two waypoints bows off the line by an amount nobody has bounded
until it is measured here.

No ROS imports: this is the offline-testable half of the executor
(ARM_SWEEP_PLAN §11.5 S4).
"""

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from .closed_form_algorithm import closed_form_algorithm
from .ur10e_kinematics import (
    JOINT_LIMIT_RAD,
    JOINT_VELOCITY_LIMIT_RAD_S,
    fk,
    jacobian,
    joint_velocity_for_twist,
    pose_matrix,
)

# Defaults mirror ARM_SWEEP_PLAN §7.D so the executor's ROS parameters and the
# offline tests cannot drift apart.
DEFAULT_SPEED_MPS = 0.05
DEFAULT_SPACING_M = 0.03
DEFAULT_MAX_JOINT_STEP_RAD = 0.35
DEFAULT_MIN_SINGULAR_VALUE = 0.02
DEFAULT_MAX_INTERP_ERROR_M = 0.005
# Fraction of the rated joint speed the sweep may ask for. Headroom matters:
# at 100% the robot's own speed scaling starts stretching the trajectory, and
# the constant wall-tangent speed the scan depends on stops being constant.
DEFAULT_VELOCITY_MARGIN = 0.8


class SweepPlanError(ValueError):
    """A sweep that must not be executed, with a reason fit for an action result.

    Deliberately an exception rather than a status flag: there is no partially
    valid trajectory worth sending, and ARM_SWEEP_PLAN §7.B requires that a
    rejected sweep never publishes one.
    """

    def __init__(self, reason: str, detail: str = ""):
        super().__init__(f"{reason}: {detail}" if detail else reason)
        self.reason = reason
        self.detail = detail


@dataclass
class SweepPlan:
    """A validated joint-space sweep, ready to become a ``JointTrajectory``."""

    times: np.ndarray                 # (N,) seconds from start
    q: np.ndarray                     # (N, 6) joint positions
    qdot: np.ndarray                  # (N, 6) joint velocities
    tcp: np.ndarray                   # (N, 3) commanded DH-end positions in arm_base
    rotation: np.ndarray              # (3, 3) plate orientation the sweep ENDS at
                                      # (== the whole sweep's, unless a correction
                                      # is being blended in -- §4.2)
    length: float                     # sweep length (m)
    speed: float                      # commanded Cartesian speed (m/s)
    # Populated by validation; recorded so a bench run can tell "it passed" from
    # "it passed with no margin left" (ARM_SWEEP_PLAN §7.B diagnostics).
    max_joint_step: float = 0.0
    min_singular_value: float = float("inf")
    peak_velocity_ratio: float = 0.0
    max_interpolation_error: float = 0.0
    warnings: List[str] = field(default_factory=list)

    @property
    def duration(self) -> float:
        return float(self.times[-1])

    def __len__(self) -> int:
        return int(self.q.shape[0])


def sample_line(p_start, p_end, spacing: float) -> np.ndarray:
    """Arc lengths along ``p_start -> p_end``, endpoints included.

    Evenly spaced rather than "``spacing`` until the remainder runs out": an
    undersized final step would give the last waypoint a different velocity
    profile from every other, right where the plate is about to leave the wall.
    """
    p_start = np.asarray(p_start, dtype=float)
    p_end = np.asarray(p_end, dtype=float)
    length = float(np.linalg.norm(p_end - p_start))
    if length < 1e-6:
        raise SweepPlanError("degenerate_sweep", f"length {length:.6f} m")
    if spacing <= 0.0:
        raise SweepPlanError("bad_spacing", f"spacing must be > 0, got {spacing}")
    count = max(2, int(np.ceil(length / spacing)) + 1)
    return np.linspace(0.0, length, count)


def blend_rotations(rotation_from, rotation_to, fractions) -> np.ndarray:
    """Slerp from one plate orientation to another over ``fractions`` in 0..1.

    An orientation correction cannot be applied as a step: the plate is against
    the wall and the arm would snap to the new pose in one waypoint interval.
    ARM_SWEEP_PLAN §4.2 asks for it to be blended in over a short lateral
    distance, which means the sweep's orientation is no longer constant and has
    to be interpolated properly rather than by lerping matrix entries.
    """
    from scipy.spatial.transform import Rotation, Slerp

    key = Rotation.from_matrix(np.stack((np.asarray(rotation_from, dtype=float),
                                         np.asarray(rotation_to, dtype=float))))
    return Slerp([0.0, 1.0], key)(np.clip(np.asarray(fractions, dtype=float), 0.0, 1.0)
                                  ).as_matrix()


def solve_ik_chain(rotation, p_start, direction, arc_lengths, q_seed) -> np.ndarray:
    """Analytic IK for every waypoint, seeding each solve with the previous answer.

    The seeding is the whole point. ``_select_best_solution`` picks the branch
    nearest its seed and unwraps each joint to the closest equivalent angle, so
    chaining ``q_prev`` forward guarantees branch continuity -- the arm cannot
    flip elbow or wrist configuration halfway across the wall (ARM_SWEEP_PLAN
    §3.5). Seeding every solve with the same initial ``q`` would not.
    """
    p_start = np.asarray(p_start, dtype=float)
    direction = np.asarray(direction, dtype=float)
    q_prev = np.asarray(q_seed, dtype=float)
    rotations = np.asarray(rotation, dtype=float)
    per_waypoint = rotations.ndim == 3

    solutions = np.empty((len(arc_lengths), 6))
    for i, s in enumerate(arc_lengths):
        T = pose_matrix(rotations[i] if per_waypoint else rotations,
                        p_start + direction * s)
        q_i = closed_form_algorithm(T, q_prev, type=0)
        if np.any(np.isnan(q_i)):
            raise SweepPlanError(
                "ik_unreachable",
                f"no IK solution {s:.3f} m into the sweep "
                f"(target {np.round(p_start + direction * s, 3).tolist()})",
            )
        solutions[i] = q_i
        q_prev = q_i
    return solutions


def joint_velocities(q, direction, speed: float) -> np.ndarray:
    """Per-waypoint joint velocities for a constant Cartesian sweep.

    ``qdot = J^-1 [v, 0]``: the commanded orientation is fixed along the sweep,
    so the angular half of the twist is zero. Interior waypoints carry through
    velocities -- the PTC is given a moving trajectory, not a string of stops.

    The endpoints alone are zeroed, because they *are* deliberate stops: the arm
    starts from rest against the wall and must be stationary when Force Mode is
    released. This is the opposite of what ``publisher_joint_trajectory_planned``
    does (it zeroes *every* waypoint, turning a 0.8 m sweep into 27 separate
    accelerate/decelerate cycles) and is exactly why the executor must own its
    own action client instead of going through that bridge (§3.2).
    """
    direction = np.asarray(direction, dtype=float)
    twist = np.concatenate((direction * speed, np.zeros(3)))
    qdot = np.array([joint_velocity_for_twist(q_i, twist) for q_i in q])
    qdot[0] = 0.0
    qdot[-1] = 0.0
    return qdot


def interpolation_error(q, tcp) -> float:
    """Worst TCP deviation from the commanded line *between* waypoints.

    Every waypoint sits exactly on the line by construction, so a sampled-only
    check would always pass and prove nothing. The robot interpolates in joint
    space, and a joint-space straight line is a Cartesian arc; this bows the TCP
    off the wall tangent by an amount that grows with waypoint spacing. Measured
    at each segment midpoint, which is where the bow is largest.
    """
    worst = 0.0
    for i in range(len(q) - 1):
        q_mid = 0.5 * (q[i] + q[i + 1])
        p_mid_actual = fk(q_mid)[:3, 3]
        p_mid_nominal = 0.5 * (tcp[i] + tcp[i + 1])
        worst = max(worst, float(np.linalg.norm(p_mid_actual - p_mid_nominal)))
    return worst


def plan_sweep(
    p_start,
    p_end,
    rotation,
    q_seed,
    *,
    speed: float = DEFAULT_SPEED_MPS,
    spacing: float = DEFAULT_SPACING_M,
    max_joint_step: float = DEFAULT_MAX_JOINT_STEP_RAD,
    min_singular_value: float = DEFAULT_MIN_SINGULAR_VALUE,
    max_interpolation_error: float = DEFAULT_MAX_INTERP_ERROR_M,
    velocity_margin: float = DEFAULT_VELOCITY_MARGIN,
    rotation_to=None,
    blend_distance: float = 0.0,
) -> SweepPlan:
    """Build and validate one lateral sweep, or raise :class:`SweepPlanError`.

    ``p_start``/``p_end`` are DH-end-frame positions in ``arm_base``; ``rotation``
    is the fixed 3x3 plate orientation captured after ``press_prepare``;
    ``q_seed`` is the arm's current joint configuration, which both seeds the IK
    branch and anchors the sweep to where the arm actually is.

    Timing comes straight from the geometry: ``t_i = s_i / speed``. That makes
    ``speed`` the single control over how fast the plate crosses the wall, which
    is what lets a bench run change sweep speed without touching anything else.
    """
    if speed <= 0.0:
        raise SweepPlanError("bad_speed", f"speed must be > 0, got {speed}")

    p_start = np.asarray(p_start, dtype=float)
    p_end = np.asarray(p_end, dtype=float)
    arc_lengths = sample_line(p_start, p_end, spacing)
    length = float(arc_lengths[-1])
    direction = (p_end - p_start) / length

    # Orientation is constant unless a correction is being blended in (§4.2, S6).
    rotation = np.asarray(rotation, dtype=float)
    target_rotation = rotation
    if rotation_to is not None:
        target_rotation = np.asarray(rotation_to, dtype=float)
        span = max(float(blend_distance), 1e-9)
        rotation = blend_rotations(rotation, target_rotation, arc_lengths / span)

    q = solve_ik_chain(rotation, p_start, direction, arc_lengths, q_seed)
    tcp = p_start[None, :] + direction[None, :] * arc_lengths[:, None]
    qdot = joint_velocities(q, direction, speed)
    times = arc_lengths / speed

    plan = SweepPlan(
        times=times, q=q, qdot=qdot, tcp=tcp,
        rotation=target_rotation, length=length, speed=speed,
    )
    _validate(
        plan,
        q_seed=np.asarray(q_seed, dtype=float),
        max_joint_step=max_joint_step,
        min_sv=min_singular_value,
        max_interp_error=max_interpolation_error,
        velocity_margin=velocity_margin,
    )
    return plan


def _validate(plan, *, q_seed, max_joint_step, min_sv, max_interp_error, velocity_margin):
    """Reject anything that must not reach the controller; annotate the rest.

    Ordered cheapest-first so an obviously bad sweep fails before the Jacobian
    and FK passes run over every waypoint.
    """
    q, qdot = plan.q, plan.qdot

    # The arm must already be standing at the first waypoint -- the executor
    # plans a lead-in for exactly that. A large step here does not mean the sweep
    # is bad; it means the sweep does not start where the arm is, and executing
    # it would fling the plate across the wall to reach its start point.
    seed_step = float(np.max(np.abs(q[0] - q_seed)))
    if seed_step > max_joint_step:
        raise SweepPlanError(
            "seed_not_at_start",
            f"the arm is {seed_step:.3f} rad from the first waypoint "
            f"(limit {max_joint_step:.3f}); plan a lead-in to "
            f"{plan.tcp[0].round(3).tolist()} first",
        )

    # Branch continuity along the sweep itself. A step this large between two
    # waypoints 3 cm apart is not a fast sweep, it is the IK having jumped to a
    # different arm configuration -- which the robot executes as a full
    # reconfiguration swing, dragging the plate sideways across the wall.
    steps = np.abs(np.diff(q, axis=0))
    plan.max_joint_step = float(np.max(steps))
    if plan.max_joint_step > max_joint_step:
        index = int(np.argmax(np.max(steps, axis=1))) + 1
        raise SweepPlanError(
            "branch_discontinuity",
            f"joint step {plan.max_joint_step:.3f} rad > {max_joint_step:.3f} "
            f"between waypoints {index - 1} and {index} "
            f"({plan.tcp[index].round(3).tolist()})",
        )

    if np.any(np.abs(q) > JOINT_LIMIT_RAD):
        joint = int(np.argmax(np.max(np.abs(q), axis=0)))
        raise SweepPlanError(
            "joint_limit",
            f"joint {joint} reaches {np.max(np.abs(q[:, joint])):.3f} rad, "
            f"outside +/-{JOINT_LIMIT_RAD:.3f}",
        )

    ratios = np.abs(qdot) / JOINT_VELOCITY_LIMIT_RAD_S[None, :]
    plan.peak_velocity_ratio = float(np.max(ratios))
    if plan.peak_velocity_ratio > velocity_margin:
        index, joint = np.unravel_index(int(np.argmax(ratios)), ratios.shape)
        raise SweepPlanError(
            "joint_velocity",
            f"joint {joint} needs {abs(qdot[index, joint]):.3f} rad/s "
            f"({plan.peak_velocity_ratio * 100:.0f}% of rated) at {plan.speed:.3f} m/s; "
            f"lower sweep_speed_mps or shorten the partition",
        )

    singular_values = np.array([
        np.linalg.svd(jacobian(q_i), compute_uv=False)[-1] for q_i in q
    ])
    plan.min_singular_value = float(np.min(singular_values))
    if plan.min_singular_value < min_sv:
        index = int(np.argmin(singular_values))
        raise SweepPlanError(
            "near_singular",
            f"Jacobian sigma_min {plan.min_singular_value:.4f} < {min_sv:.4f} at "
            f"waypoint {index} ({plan.tcp[index].round(3).tolist()})",
        )

    plan.max_interpolation_error = interpolation_error(q, plan.tcp)
    if plan.max_interpolation_error > max_interp_error:
        raise SweepPlanError(
            "interpolation_error",
            f"TCP bows {plan.max_interpolation_error * 1000:.1f} mm off the line "
            f"between waypoints (limit {max_interp_error * 1000:.1f} mm); "
            f"reduce waypoint_spacing_m",
        )

    # Warn where a limit is close rather than crossed: these are the numbers to
    # watch when tuning partition length and sweep speed on the bench.
    if plan.peak_velocity_ratio > 0.5 * velocity_margin:
        plan.warnings.append(
            f"peak joint velocity at {plan.peak_velocity_ratio * 100:.0f}% of rated"
        )
    if plan.min_singular_value < 2.0 * min_sv:
        plan.warnings.append(
            f"closest approach to a singularity sigma_min={plan.min_singular_value:.4f}"
        )
