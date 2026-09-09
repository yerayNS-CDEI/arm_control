"""Tests for the S6 orientation-drift trigger (ARM_SWEEP_PLAN §4.2).

Driven by REAL data: `data/recorded_sweep_tilt.json` is the `wall_tilt_deg`
channel from one complete Gazebo sweep of a flat wall (partition 0 of
rosbag2_2026_08_28-10_37_58, 955 samples over 58.6 s). Because the wall is flat
and the plate holds its commanded orientation to 0.000 deg throughout, **every
trigger on this trace is a false positive** -- which makes it exactly the right
fixture for choosing a threshold.

The headline result, and the reason the plan's numbers were not used. Counts are
over this 58.6 s trace, WITH the 5 s cooldown already suppressing repeats:

    threshold  dwell   false replans   raw exceedances (no cooldown)
      2.0 deg   none        11               32     <- ARM_SWEEP_PLAN §7.D's guess
      3.0 deg   none        10               22
      4.0 deg   none         6               10
      3.0 deg   1.0 s        0                -
      4.0 deg   1.0 s        0                -     <- shipped

The dwell is doing the work and is absent from the plan, which specifies only
threshold + hysteresis. Hysteresis cannot suppress a noise floor; requiring the
error to persist can.

The trigger's decision logic is re-implemented here rather than driven through
the node, because the node needs a ROS graph. Any change to
`_orientation_drifted` must be mirrored here -- `test_the_shipped_defaults_are_
what_the_node_declares` guards the constants.

Run with:

    python3 -m pytest test/test_orientation_replan_trigger.py -v
"""

import json
import os
import re

import pytest

FIXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "data", "recorded_sweep_tilt.json")
EXECUTOR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "control", "wall_sweep_executor.py")


@pytest.fixture(scope="module")
def trace():
    """(t, tilt_deg) from a real sweep of a flat wall."""
    return [tuple(row) for row in json.load(open(FIXTURE))]


def count_triggers(trace, threshold, dwell, hysteresis=1.0, cooldown=5.0):
    """Mirror of WallSweepExecutor._orientation_drifted over a recorded trace."""
    over_since, armed, last, triggers = None, True, -1e9, 0
    for t, tilt in trace:
        if tilt <= threshold - hysteresis:
            armed = True
        if tilt <= threshold:
            over_since = None
            continue
        if over_since is None:
            over_since = t
            continue
        if not armed or t - over_since < dwell or t - last < cooldown:
            continue
        armed, over_since, last = False, None, t
        triggers += 1
    return triggers


# ---------------------------------------------------------------------------
# The measurement that chose the defaults
# ---------------------------------------------------------------------------

def test_the_plans_guessed_threshold_is_unusable(trace):
    """2.0 deg with no dwell sits right on the median of the noise, so it fires
    repeatedly on a flat wall EVEN WITH the 5 s cooldown holding it back -- 11
    spurious preempts in one 58.6 s sweep, from 32 raw exceedances."""
    assert count_triggers(trace, threshold=2.0, dwell=0.0) >= 10
    assert count_triggers(trace, threshold=2.0, dwell=0.0, cooldown=0.0) >= 30


def test_the_shipped_defaults_never_fire_on_a_flat_wall(trace):
    assert count_triggers(trace, threshold=4.0, dwell=1.0) == 0


def test_dwell_is_what_suppresses_the_noise_not_threshold_alone(trace):
    """Raising the threshold alone still leaves false triggers; adding the dwell
    removes them. This is the finding the plan is missing."""
    assert count_triggers(trace, threshold=4.0, dwell=0.0) >= 5
    assert count_triggers(trace, threshold=4.0, dwell=1.0) == 0


@pytest.mark.parametrize("hysteresis", [0.5, 1.0, 1.5])
def test_hysteresis_alone_cannot_save_a_bad_threshold(hysteresis, trace):
    """§4.2 offers hysteresis as the anti-oscillation measure. Against a noise
    floor it does nothing on its own: the signal keeps crossing the whole band,
    so widening it changes nothing until it stops re-arming altogether."""
    assert count_triggers(trace, threshold=2.0, dwell=0.0, hysteresis=hysteresis) >= 10


def test_hysteresis_wide_enough_to_suppress_noise_just_disables_the_trigger(trace):
    """The trap in tuning hysteresis instead of adding a dwell. At threshold 2.0
    a 2.0 deg band puts the re-arm point at 0.0 deg, which the tilt essentially
    never reaches -- so it fires once and then never re-arms. That is not a
    working trigger, it is a disabled one, and it would silently stop correcting
    real drift too."""
    assert count_triggers(trace, threshold=2.0, dwell=0.0, hysteresis=2.0) == 1
    # Whereas a dwell suppresses the noise while leaving the trigger live:
    assert count_triggers(trace, threshold=4.0, dwell=1.0) == 0
    drifting = [(t, tilt + 6.0) for t, tilt in trace]
    assert count_triggers(drifting, threshold=4.0, dwell=1.0) >= 1


@pytest.mark.parametrize("threshold,dwell", [(4.0, 1.0), (5.0, 0.5), (3.0, 2.0)])
def test_every_admissible_setting_is_quiet_on_flat_wall(threshold, dwell, trace):
    assert count_triggers(trace, threshold=threshold, dwell=dwell) == 0


# ---------------------------------------------------------------------------
# It must still fire on real drift
# ---------------------------------------------------------------------------

def test_a_genuine_drift_still_triggers(trace):
    """Suppressing noise is only half of it: a sustained tilt must get through."""
    drifting = [(t, tilt + 6.0) for t, tilt in trace]
    assert count_triggers(drifting, threshold=4.0, dwell=1.0) >= 1


def test_a_brief_spike_does_not_trigger(trace):
    """A single bad frame -- one sensor glinting off the wall -- must not preempt
    a trajectory."""
    spiked = list(trace)
    spiked[100] = (spiked[100][0], 30.0)
    assert count_triggers(spiked, threshold=4.0, dwell=1.0) == 0


def test_the_cooldown_bounds_how_often_a_drift_can_preempt():
    """A steady 10 deg error for 60 s must not replan on every sample."""
    steady = [(i * 0.05, 10.0) for i in range(1200)]
    assert count_triggers(steady, threshold=4.0, dwell=1.0, cooldown=5.0) <= 13


def test_recovery_re_arms_the_trigger():
    """After the plate comes back inside the hysteresis band, a fresh excursion
    must be able to fire again."""
    trace = ([(i * 0.05, 10.0) for i in range(60)]        # 3 s drifted
             + [(3.0 + i * 0.05, 0.5) for i in range(200)]  # recovered
             + [(13.0 + i * 0.05, 10.0) for i in range(60)])
    assert count_triggers(trace, threshold=4.0, dwell=1.0, cooldown=5.0) == 2


# ---------------------------------------------------------------------------
# The node must actually ship these numbers
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name,expected", [
    ("orientation_replan_threshold_deg", "4.0"),
    ("orientation_replan_dwell_s", "1.0"),
    ("orientation_replan_hysteresis_deg", "1.0"),
    ("orientation_replan_min_interval_s", "5.0"),
])
def test_the_shipped_defaults_are_what_the_node_declares(name, expected):
    """Guards the link between this analysis and the running code: if someone
    retunes a default, this fails and points them back at the measurement."""
    source = open(EXECUTOR).read()
    match = re.search(rf'declare_parameter\("{name}",\s*([0-9.]+)\)', source)
    assert match, f"{name} is not declared in the executor"
    assert match.group(1) == expected
