"""Unit tests for the shared wall-alignment estimator (ARM_SWEEP_PLAN §4.2, S3).

This module exists so `wall_parallel_controller` (which drives the plate) and
`wall_sweep_executor` (which must only observe) compute the SAME wall normal.
Two copies of the geometry would drift and the two nodes would then disagree
about where the wall is, which is exactly the failure the extraction prevents.

Pure numpy against the real plate layout -- no ROS, no simulator.

Run with:

    python3 -m pytest test/test_wall_alignment_estimator.py -v
"""

import numpy as np
import pytest

from sensors.wall_alignment_estimator import (
    MIN_SENSORS_FOR_FIT,
    SENSOR_XY,
    VALID_HI,
    VALID_LO,
    WallAlignmentEstimator,
    correction_angles,
    fit_wall_normal,
    mean_distance,
    usable_mask,
    valid_mask,
)

FLAT = 0.20


def ranges(tilt_x_deg=0.0, tilt_y_deg=0.0, standoff=FLAT, noise=0.0, seed=0):
    """Ranges a plate tilted about the plate X and/or Y axes would report.

    A plate rotated about +X by `t` puts sensors at +y further away by y*tan(t),
    which is what the plane fit has to invert.
    """
    tx, ty = np.deg2rad(tilt_x_deg), np.deg2rad(tilt_y_deg)
    d = standoff + SENSOR_XY[:, 1] * np.tan(tx) + SENSOR_XY[:, 0] * np.tan(ty)
    if noise:
        d = d + np.random.default_rng(seed).normal(0.0, noise, 6)
    return d.astype(float)


# ---------------------------------------------------------------------------
# Validity
# ---------------------------------------------------------------------------

def test_a_flat_plate_at_a_normal_standoff_is_all_valid():
    assert valid_mask(ranges()).all()


def test_two_independent_outliers_are_both_trimmed():
    clean = ranges(tilt_x_deg=5.0)
    fouled = clean.copy()
    fouled[1], fouled[2] = 0.90, 1.40
    assert deflection(clean, fouled) < 0.01
    assert int(usable_mask(fouled).sum()) == 4


def test_out_of_window_readings_are_dropped():
    d = ranges()
    d[0] = VALID_HI[0] + 1.0        # ultrasonic saturated
    d[3] = VALID_LO[3] * 0.5        # ToF below its floor
    d[5] = np.nan
    assert list(valid_mask(d)) == [False, True, True, False, True, False]


def test_the_tof_window_is_tighter_than_the_ultrasonic_one():
    """Regression guard: the ToF sensors saturate at 0.258 m, so a plate at the
    0.30 m sweep standoff sees ONLY the ultrasonics."""
    at_sweep_standoff = ranges(standoff=0.30)
    assert list(valid_mask(at_sweep_standoff)) == [True, True, True, False, False, False]


# ---------------------------------------------------------------------------
# mean_distance -- must agree with the FSM's own reading
# ---------------------------------------------------------------------------

def test_mean_distance_of_a_flat_plate_is_the_standoff():
    assert mean_distance(ranges(standoff=0.17)) == pytest.approx(0.17)


def test_one_valid_reading_is_enough_for_a_distance():
    """Deliberately laxer than the plane fit: a standoff needs no fit. Matches
    scan_wall._plate_wall_distance so the FSM and the estimator never disagree
    about whether a frame is usable at all."""
    d = np.full(6, np.nan)
    d[1] = 0.22
    assert mean_distance(d) == pytest.approx(0.22)


def test_no_valid_reading_gives_none():
    assert mean_distance(np.full(6, np.nan)) is None


def test_a_wrong_length_frame_is_rejected():
    assert mean_distance(np.array([0.2, 0.2, 0.2])) is None


# ---------------------------------------------------------------------------
# fit_wall_normal
# ---------------------------------------------------------------------------

def test_a_flat_plate_fits_a_parallel_normal():
    normal, distance, n_valid = fit_wall_normal(ranges())
    assert n_valid == 6
    assert distance == pytest.approx(FLAT)
    assert normal == pytest.approx([0.0, 0.0, 1.0], abs=1e-12)


@pytest.mark.parametrize("tilt", [-15.0, -5.0, 3.0, 12.0])
def test_the_fitted_normal_recovers_the_tilt(tilt):
    normal, _, _ = fit_wall_normal(ranges(tilt_x_deg=tilt))
    recovered = np.rad2deg(np.arctan2(-normal[1], normal[2]))
    assert recovered == pytest.approx(tilt, abs=1e-6)


def test_tilt_about_the_other_axis_lands_in_the_other_component():
    """Pins the sign convention. A plate tilted about +Y reads FURTHER at +x, so
    the fitted plane has a = +tan(t) and the normal's x component is -tan(t) --
    the correction has to oppose the tilt, not follow it."""
    normal, _, _ = fit_wall_normal(ranges(tilt_y_deg=7.0))
    assert np.rad2deg(np.arctan2(normal[0], normal[2])) == pytest.approx(-7.0, abs=1e-6)
    assert normal[1] == pytest.approx(0.0, abs=1e-9)


def test_the_normal_is_always_unit_length():
    for tilt in (-20.0, 0.0, 20.0):
        normal, _, _ = fit_wall_normal(ranges(tilt_x_deg=tilt))
        assert np.linalg.norm(normal) == pytest.approx(1.0)


def test_fewer_than_three_sensors_refuses_rather_than_guessing():
    """A plane needs three points. Returning a guess here would let the sweep
    correct its orientation toward a wall nobody measured."""
    d = np.full(6, np.nan)
    d[0], d[1] = 0.20, 0.20
    normal, distance, n_valid = fit_wall_normal(d)
    assert normal is None and distance is None
    assert n_valid == 2 < MIN_SENSORS_FOR_FIT


def deflection(clean, fouled, **kwargs):
    """Angle between the fits of a clean frame and a fouled one, in degrees."""
    a, _, _ = fit_wall_normal(clean, **kwargs)
    b, _, _ = fit_wall_normal(fouled, **kwargs)
    return float(np.rad2deg(np.arccos(np.clip(np.dot(a, b), -1.0, 1.0))))


@pytest.mark.parametrize("bad", [0.40, 0.60, 0.90, 1.20, 2.00])
def test_a_sensor_that_missed_the_wall_is_rejected(bad):
    """A plate parked at a partition START overhangs the wall end, and an
    ultrasonic that sees past it still passes its 3.90 m validity window. The
    median gate is what removes it."""
    clean = ranges(tilt_x_deg=6.0)
    fouled = clean.copy()
    fouled[2] = bad
    assert deflection(clean, fouled) < 0.01
    assert fit_wall_normal(fouled)[2] == 5      # the outlier, and only it, is dropped
    assert not usable_mask(fouled)[2]


@pytest.mark.parametrize("bad", [0.40, 1.20])
def test_without_the_gate_the_huber_fit_follows_the_outlier(bad):
    """Documents WHY the gate exists. IRLS gives no protection here: with six
    points the MAD is itself contaminated, so every weight stays near 1 and more
    iterations change nothing."""
    clean = ranges(tilt_x_deg=6.0)
    fouled = clean.copy()
    fouled[2] = bad
    assert deflection(clean, fouled, max_residual=0.0) > 20.0
    assert deflection(clean, fouled, max_residual=0.0, iterations=30) > 20.0


@pytest.mark.parametrize("tilt", [-30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0])
def test_the_gate_never_rejects_a_consistent_plate(tilt):
    """The guard must not eat legitimate tilt. The plate spans 0.344 m, so even
    30 deg only spreads the ranges by about +/-0.10 m, inside the 0.15 gate.

    Asserted against the window-only mask, because the ToF windows saturate at
    0.258 m and drop the far sensors on their own past ~15 deg -- that is the
    validity window doing its job, not the trim.

    This is exactly what a raw-range gate got wrong: a steep plate has a large
    range SPREAD but small residuals, so gating on spread ate legitimate readings
    precisely when the plate was worst aligned."""
    d = ranges(tilt_x_deg=tilt)
    assert list(usable_mask(d)) == list(valid_mask(d))


@pytest.mark.parametrize("tilt", [-12.0, -5.0, 5.0, 12.0])
def test_the_gate_leaves_the_recovered_tilt_untouched(tilt):
    normal, _, _ = fit_wall_normal(ranges(tilt_x_deg=tilt))
    assert np.rad2deg(np.arctan2(-normal[1], normal[2])) == pytest.approx(tilt, abs=1e-6)


def test_the_tof_windows_alone_thin_out_a_steep_plate():
    """Worth pinning separately: past ~15 deg at a 0.20 m standoff the top ToFs
    exceed their 0.258 m ceiling, so the fit runs on the ultrasonics."""
    assert int(valid_mask(ranges(tilt_x_deg=20.0)).sum()) == 4


def test_three_contradictory_readings_refuse_the_fit_rather_than_guess():
    """Past two bad readings the frame is not describing one surface. Returning a
    confident answer from the remaining minority would be worse than refusing:
    the controller holds orientation on a failed fit."""
    d = ranges(tilt_x_deg=4.0)
    d[0], d[1], d[2] = 1.5, 1.6, 1.7      # three sensors off the wall entirely
    assert fit_wall_normal(d)[0] is None


def test_at_most_two_sensors_are_trimmed_from_a_fittable_frame():
    clean = ranges(tilt_x_deg=5.0)
    fouled = clean.copy()
    fouled[1], fouled[2] = 0.90, 1.40
    assert int(valid_mask(fouled).sum()) - int(usable_mask(fouled).sum()) == 2


def test_a_four_sensor_frame_is_not_trimmed_at_all():
    """Trimming 4 down to 3 leaves an exact fit with nothing to check it against,
    so a 3-point consensus wins trivially. Measured with the floor at 3: a clean
    noisy frame lost a good sensor and the recovered tilt fell from 10.2 to 5.2
    deg -- the guard was worse than no guard."""
    d = ranges(tilt_x_deg=9.4, standoff=0.247, noise=0.010, seed=11)
    assert int(valid_mask(d).sum()) == 4          # both top ToFs saturated
    assert list(usable_mask(d)) == list(valid_mask(d))


def tilt_errors(n, noise, outlier, seed):
    """Mean |recovered tilt - true tilt|, guarded and unguarded, over n frames.

    Accuracy against the KNOWN truth, not agreement between the two fits: at 10 mm
    sensor noise on a 0.344 m baseline the fit is noise-dominated, so ANY change
    of subset moves it several degrees. Comparing the two estimates to each other
    measures that noise; comparing each to the truth measures the guard.
    """
    rng = np.random.default_rng(seed)
    guarded, plain = [], []
    for _ in range(n):
        tilt, standoff = rng.uniform(-10, 10), rng.uniform(0.15, 0.25)
        d = (standoff + SENSOR_XY[:, 1] * np.tan(np.deg2rad(tilt))
             + rng.normal(0.0, noise, 6)).astype(float)
        if outlier:
            d[rng.integers(0, 3)] = rng.uniform(0.6, 2.0)   # ultrasonic past the wall edge
        for out, kwargs in ((guarded, {}), (plain, {"max_residual": 0.0})):
            normal, _, _ = fit_wall_normal(d, **kwargs)
            if normal is not None:
                out.append(abs(np.rad2deg(np.arctan2(-normal[1], normal[2])) - tilt))
    return float(np.mean(guarded)), float(np.mean(plain))


def test_the_guard_costs_nothing_on_clean_frames():
    """The property that matters on hardware: with no outlier present, the guard
    must not degrade the fit."""
    guarded, plain = tilt_errors(1500, noise=0.010, outlier=False, seed=5)
    assert guarded == pytest.approx(plain, abs=0.15)


def test_the_guard_transforms_accuracy_when_a_sensor_misses_the_wall():
    """~39 deg of error becomes ~4 deg. This is the whole reason it exists."""
    guarded, plain = tilt_errors(1500, noise=0.010, outlier=True, seed=5)
    assert plain > 25.0
    assert guarded < 8.0


def test_noise_perturbs_the_fit_only_slightly():
    normal, _, _ = fit_wall_normal(ranges(tilt_x_deg=5.0, noise=0.002, seed=3))
    recovered = np.rad2deg(np.arctan2(-normal[1], normal[2]))
    assert recovered == pytest.approx(5.0, abs=2.0)


# ---------------------------------------------------------------------------
# correction_angles
# ---------------------------------------------------------------------------

def test_a_parallel_plate_needs_no_correction():
    beta, gamma = correction_angles([0.0, 0.0, 1.0])
    assert (beta, gamma) == pytest.approx((0.0, 0.0))


def test_the_correction_opposes_the_tilt():
    normal, _, _ = fit_wall_normal(ranges(tilt_x_deg=10.0))
    _, gamma = correction_angles(normal)
    assert np.rad2deg(gamma) == pytest.approx(10.0, abs=1e-6)


def test_the_slew_limit_clamps_the_correction():
    normal, _, _ = fit_wall_normal(ranges(tilt_x_deg=30.0))
    _, gamma = correction_angles(normal, kp=1.0, max_step=np.deg2rad(3.0))
    assert abs(np.rad2deg(gamma)) == pytest.approx(3.0)


def test_the_gain_scales_the_correction():
    normal, _, _ = fit_wall_normal(ranges(tilt_x_deg=10.0))
    _, full = correction_angles(normal, kp=1.0)
    _, half = correction_angles(normal, kp=0.5)
    assert half == pytest.approx(0.5 * full)


# ---------------------------------------------------------------------------
# WallAlignmentEstimator -- the stateful EMA wrapper
# ---------------------------------------------------------------------------

def test_the_estimator_converges_on_a_steady_tilt():
    est = WallAlignmentEstimator(ema_alpha=0.3)
    d = ranges(tilt_x_deg=8.0)
    for _ in range(60):
        alignment = est.update(d)
    assert alignment.tilt_deg == pytest.approx(8.0, abs=1e-3)
    assert alignment.n_valid == 6
    assert alignment.distance == pytest.approx(FLAT)


def test_the_ema_lags_a_step_rather_than_jumping():
    """Anti-shake: the filter is what stops the sweep replanning on one noisy
    frame. A single update must not deliver the whole change."""
    est = WallAlignmentEstimator(ema_alpha=0.3)
    first = est.update(ranges(tilt_x_deg=10.0))
    assert 0.0 < first.tilt_deg < 10.0


def test_a_lower_alpha_lags_more():
    slow, fast = WallAlignmentEstimator(ema_alpha=0.1), WallAlignmentEstimator(ema_alpha=0.9)
    d = ranges(tilt_x_deg=10.0)
    assert slow.update(d).tilt_deg < fast.update(d).tilt_deg


def test_reset_returns_the_filter_to_parallel():
    est = WallAlignmentEstimator()
    for _ in range(40):
        est.update(ranges(tilt_x_deg=12.0))
    est.reset()
    assert est.normal == pytest.approx([0.0, 0.0, 1.0])


def test_an_unfittable_frame_returns_none_and_leaves_the_filter_alone():
    est = WallAlignmentEstimator()
    for _ in range(40):
        est.update(ranges(tilt_x_deg=8.0))
    before = est.normal.copy()
    assert est.update(np.full(6, np.nan)) is None
    assert est.normal == pytest.approx(before)


def test_the_estimator_is_read_only_by_construction():
    """The whole reason this module exists: the executor must be able to ask
    where the wall is without commanding the arm (§3.3, §4.2). Nothing in here
    may reach ROS."""
    import inspect
    import sensors.wall_alignment_estimator as module
    source = inspect.getsource(module)
    for forbidden in ("import rclpy", "Publisher", "create_publisher", "JointTrajectory"):
        assert forbidden not in source
