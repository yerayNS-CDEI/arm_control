"""Read-only wall alignment from the six plate range sensors (ARM_SWEEP_PLAN §4.2).

Two nodes need to know how the sensor plate is oriented relative to the wall, and
they must agree to the last decimal:

- ``wall_parallel_controller`` uses it to *drive* the plate parallel during
  ``press_prepare``;
- ``wall_sweep_executor`` needs the same number *without* commanding anything, so
  it can detect angular drift mid-sweep and replan the remainder (§4.2, S6).

The second one must never publish arm commands -- two publishers on one
trajectory controller is not survivable (§3.3) -- so the calculation lives here,
in a module with no ROS imports, and both nodes call it. An `estimate_only` mode
inside the controller was the fallback option; extracting is cleaner and makes
the geometry unit-testable offline.

    six ranges -> validity window -> weighted robust plane fit (IRLS/Huber)
                -> EMA on the fitted normal -> tilt + correction angles

**Sensor array convention** (matches ``arduino_sensors[_sim].py``'s
``distance_sensors``): ``[C, A, B, S1, S2, S3]`` -- three ultrasonic then three
ToF -- with ranges measured along the plate +Z axis, toward the wall.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# Sensor (x, y) in the plate frame, metres. Matches both the sim
# (sensor_plate.urdf.xacro D/E/F poses) and the real plate's vl6180 mounting.
SENSOR_NAMES = ("C/U1", "A/U2", "B/U3", "S1", "S2", "S3")
SENSOR_XY = np.array([
    [0.00,   0.172],    # 0: C  ultrasonic, top-mid
    [-0.155, -0.17],    # 1: A  ultrasonic, bottom-left
    [0.155,  -0.17],    # 2: B  ultrasonic, bottom-right
    [-0.152,  0.17],    # 3: S1 ToF, top-left
    [0.152,   0.17],    # 4: S2 ToF, top-right
    [0.00,   -0.172],   # 5: S3 ToF, bottom-mid
])
# Per-sensor stddev (m), used as inverse-variance weights in the fit.
SENSOR_SIGMA = np.array([0.010, 0.010, 0.010, 0.010, 0.010, 0.010])
# Validity window (m): drop saturated / out-of-range readings. Shared with
# task_planner_fsm's scan_wall so the FSM and the estimator agree on what counts
# as a reading at all.
VALID_LO = np.array([0.02, 0.02, 0.02, 0.011, 0.011, 0.011])
VALID_HI = np.array([3.90, 3.90, 3.90, 0.258, 0.258, 0.258])

# A plane needs three points. Note this is stricter than the FSM's plain standoff
# reading, which is happy with one valid range -- that needs a distance, not a fit.
MIN_SENSORS_FOR_FIT = 3

# Largest residual from the FITTED PLANE still treated as part of the same wall.
#
# The validity windows above are the SENSORS' ranges, not plausible WALL
# distances: an ultrasonic is "valid" out to 3.90 m, so one that sees past the
# edge of the wall reports a metre and passes. Measured: the IRLS/Huber fit gives
# no protection at all against that -- one such reading deflects the fitted normal
# by 23 deg at 0.40 m and 65 deg at 1.20 m, and more iterations do not help (with
# six points the MAD is itself contaminated, so every weight stays near 1).
#
# That is almost certainly what made wall_parallel_controller command a huge
# reorientation when the plate was parked at a partition START, i.e. at a wall
# end with half the plate overhanging open space.
#
# The test is the residual from the plane, not the spread of the raw ranges: a
# steeply tilted plate has a large spread but small residuals, so a raw-range
# gate eats legitimate readings exactly when the plate is worst aligned. At 3x the
# 0.010 m sensor sigma this drops a sensor on 0.1% of noisy frames and catches
# every edge outlier tested. Set 0 to disable and get the original, unguarded
# behaviour back.
DEFAULT_MAX_PLANE_RESIDUAL_M = 0.03
# At most two of six may be discarded: beyond two bad readings the frame is not
# describing one surface at all.
MAX_TRIMMED_SENSORS = 2

# ...and never trim below this many, which is one MORE than a plane needs.
#
# Three points fit a plane exactly, so a 3-sensor consensus has zero residual by
# construction and always "wins" -- there is nothing left over to check it
# against. Measured with the floor at 3: a clean, noisy 4-sensor frame (the two
# top ToFs saturated past their 0.258 m ceiling) had a legitimate sensor
# discarded and the recovered tilt fell from 10.2 deg to 5.2 deg, i.e. the guard
# made the fit WORSE than no guard at all.
#
# With the floor at 4 the surviving set is always over-determined, so a consensus
# means something. The cost is that a 4-sensor frame is not trimmed at all.
MIN_SENSORS_AFTER_TRIM = MIN_SENSORS_FOR_FIT + 1


@dataclass
class WallAlignment:
    """One estimate of where the wall is, in the PLATE frame."""

    normal: np.ndarray      # unit wall normal; [0, 0, 1] means perfectly parallel
    distance: float         # mean of the valid ranges (m)
    tilt: float             # angle between plate +Z and the wall normal (rad)
    n_valid: int
    valid: np.ndarray       # bool mask over the six sensors

    @property
    def tilt_deg(self) -> float:
        return float(np.rad2deg(self.tilt))


def valid_mask(distances) -> np.ndarray:
    """Which of the six ranges are inside their per-sensor validity window.

    Window only -- this says the *sensor* produced a reading, not that the reading
    describes the same surface as the others. :func:`trim_to_plane` does that.
    """
    d = np.asarray(distances, dtype=float)
    return np.isfinite(d) & (d > VALID_LO) & (d < VALID_HI)


def _weighted_plane(distances, ok):
    """Inverse-variance weighted least-squares plane, and its residuals."""
    A = np.column_stack((SENSOR_XY[ok, 0], SENSOR_XY[ok, 1], np.ones(int(ok.sum()))))
    W = np.diag(1.0 / SENSOR_SIGMA[ok] ** 2)
    theta = np.linalg.solve(A.T @ W @ A, A.T @ W @ distances[ok])
    return theta, distances[ok] - A @ theta


def trim_to_plane(distances, ok, max_residual: float = DEFAULT_MAX_PLANE_RESIDUAL_M,
                  max_drops: int = MAX_TRIMMED_SENSORS) -> np.ndarray:
    """Keep only the readings that lie on one common plane.

    Exhaustive RANSAC: fit a plane through every triple of valid sensors, count
    how many of the rest fall within ``max_residual`` of it, and keep the largest
    such consensus set. With at most six sensors that is 20 exact 3x3 solves --
    nothing at a 2 Hz control rate -- and it buys a genuinely robust answer.

    Iterative "fit, drop the worst residual, refit" was tried first and is not
    good enough. Least squares has a 0% breakdown point, so with two bad readings
    the very first fit is dragged far enough that the largest residual belongs to
    a GOOD sensor: measured on two edge outliers it discarded the two correct ones
    and left a 75 deg error. Consensus never has that failure mode, because a
    wrong model simply explains fewer points.

    Never trims below :data:`MIN_SENSORS_AFTER_TRIM`, and never discards more
    than ``max_drops``. When no plane explains that many readings the frame is not
    describing one surface, and this returns too few sensors to fit on purpose --
    the caller then holds orientation rather than acting on a confident-looking
    answer derived from readings that contradict each other.
    """
    from itertools import combinations

    d = np.asarray(distances, dtype=float)
    ok = ok.copy()
    indices = np.where(ok)[0]
    # Too few sensors to trim and still leave an over-determined fit: keep them
    # all and let the fit be whatever the readings say.
    if max_residual <= 0.0 or len(indices) <= MIN_SENSORS_AFTER_TRIM:
        return ok

    floor = max(MIN_SENSORS_AFTER_TRIM, len(indices) - max_drops)
    best_mask, best_count, best_error = None, -1, np.inf

    for triple in combinations(indices, MIN_SENSORS_FOR_FIT):
        seed = np.zeros(6, dtype=bool)
        seed[list(triple)] = True
        try:
            theta, _ = _weighted_plane(d, seed)
        except np.linalg.LinAlgError:
            continue                      # collinear sensors: no unique plane
        residuals = d - (SENSOR_XY[:, 0] * theta[0] + SENSOR_XY[:, 1] * theta[1] + theta[2])
        inliers = ok & (np.abs(residuals) <= max_residual)
        count = int(inliers.sum())
        if count < floor:
            continue
        error = float(np.sum(residuals[inliers] ** 2))
        # Most points explained wins; a tie goes to the tighter fit.
        if count > best_count or (count == best_count and error < best_error):
            best_mask, best_count, best_error = inliers, count, error

    if best_mask is not None:
        return best_mask
    # No plane explains `floor` of the readings, so they are not one surface.
    # Report the largest self-consistent pair rather than the full set: the caller
    # then sees a failed fit and holds orientation, instead of being handed a
    # confident answer computed from readings that disagree with each other.
    starved = np.zeros(6, dtype=bool)
    starved[indices[:MIN_SENSORS_FOR_FIT - 1]] = True
    return starved


def usable_mask(distances, max_residual: float = DEFAULT_MAX_PLANE_RESIDUAL_M) -> np.ndarray:
    """Validity window followed by the plane-consistency trim."""
    ok = valid_mask(distances)
    if int(ok.sum()) < MIN_SENSORS_FOR_FIT:
        return ok
    return trim_to_plane(distances, ok, max_residual=max_residual)


def mean_distance(distances) -> Optional[float]:
    """Mean plate-to-wall distance over the valid ranges, or None.

    One valid reading is enough: a standoff needs no plane fit. Mirrors
    ``scan_wall._plate_wall_distance`` deliberately -- a sweep that aborts on
    "contact lost" where the FSM would have called the same frame valid is a
    debugging nightmare.
    """
    d = np.asarray(distances, dtype=float)
    if d.shape != (6,):
        return None
    ok = valid_mask(d)
    return float(np.mean(d[ok])) if ok.any() else None


def fit_wall_normal(
    distances,
    huber_k: float = 1.5,
    iterations: int = 3,
    max_residual: float = DEFAULT_MAX_PLANE_RESIDUAL_M,
):
    """Weighted robust plane fit ``d = a*x + b*y + c`` over the valid sensors.

    Returns ``(normal, mean_distance, n_valid)``, or ``(None, None, n_valid)``
    when fewer than three sensors are usable or the normal equations are
    singular.

    IRLS with a Huber weight handles mild disagreement between sensors. It does
    **not** handle a gross outlier -- with six points the MAD is itself
    contaminated and every weight stays near 1 -- so :func:`trim_to_plane` is
    what removes a sensor that has missed the wall entirely. Pass
    ``max_residual=0`` for the original, unguarded behaviour.
    """
    d = np.asarray(distances, dtype=float)
    ok = usable_mask(d, max_residual=max_residual)
    n_valid = int(ok.sum())
    if n_valid < MIN_SENSORS_FOR_FIT:
        return None, None, n_valid

    x, y = SENSOR_XY[ok, 0], SENSOR_XY[ok, 1]
    dv = d[ok]
    A = np.column_stack((x, y, np.ones_like(x)))

    w_base = 1.0 / (SENSOR_SIGMA[ok] ** 2)      # inverse-variance weights
    w = w_base.copy()

    theta = None
    for _ in range(iterations):
        W = np.diag(w)
        try:
            theta = np.linalg.solve(A.T @ W @ A, A.T @ W @ dv)
        except np.linalg.LinAlgError:
            return None, None, n_valid
        res = dv - A @ theta
        mad = np.median(np.abs(res - np.median(res)))
        scale = 1.4826 * mad if mad > 1e-6 else (np.std(res) + 1e-6)
        t = np.abs(res) / (huber_k * scale)
        huber = np.where(t <= 1.0, 1.0, 1.0 / np.maximum(t, 1e-9))
        w = w_base * huber

    a, b, _c = theta
    normal = np.array([-a, -b, 1.0])
    normal /= np.linalg.norm(normal)
    return normal, float(np.mean(dv)), n_valid


def correction_angles(normal, kp: float = 1.0, max_step: float = np.pi):
    """``(beta, gamma)`` -- the pitch/yaw increment that nulls the wall normal.

    Applied in the plate frame as ``R.from_euler('ZYX', [0, beta, gamma])``.
    ``kp`` and ``max_step`` are the controller's gain and per-cycle slew limit;
    a pure estimate wants ``kp=1`` and no clamp, which is the default here.
    """
    n = np.asarray(normal, dtype=float)
    gamma = -np.arctan2(n[1], n[2])
    beta = np.arctan2(n[0], n[2])
    return (
        float(np.clip(kp * beta, -max_step, max_step)),
        float(np.clip(kp * gamma, -max_step, max_step)),
    )


class WallAlignmentEstimator:
    """Stateful wrapper: EMA-filters the fitted normal across updates.

    The filter is on the fitted **normal**, not on the raw ranges -- filtering
    ranges individually smears the plane when only some sensors are noisy. State
    is just the filtered normal, so :meth:`reset` is enough between partitions.
    """

    def __init__(self, ema_alpha: float = 0.3, huber_k: float = 1.5,
                 max_residual: float = DEFAULT_MAX_PLANE_RESIDUAL_M):
        self.ema_alpha = float(ema_alpha)
        self.huber_k = float(huber_k)
        self.max_residual = float(max_residual)
        self.reset()

    def reset(self) -> None:
        # Seeded parallel: with no evidence yet, assume the plate is aligned and
        # let the first fits pull it off that, rather than starting from a tilt
        # nothing measured.
        self.normal = np.array([0.0, 0.0, 1.0])

    def update(self, distances) -> Optional[WallAlignment]:
        """Fold one ``distance_sensors`` frame in. None when the fit is not possible."""
        normal, distance, n_valid = fit_wall_normal(
            distances, huber_k=self.huber_k, max_residual=self.max_residual
        )
        if normal is None:
            return None

        self.normal = self.ema_alpha * normal + (1.0 - self.ema_alpha) * self.normal
        self.normal /= np.linalg.norm(self.normal)

        return WallAlignment(
            normal=self.normal.copy(),
            distance=distance,
            tilt=float(np.arccos(np.clip(self.normal[2], -1.0, 1.0))),
            n_valid=n_valid,
            valid=usable_mask(distances, max_residual=self.max_residual),
        )
