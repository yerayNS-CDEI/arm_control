"""Regression tests for the trajectory bridge's hold (ARM_SWEEP_PLAN §3.3).

`publisher_joint_trajectory_planned` forwards `planned_trajectory` to the same
trajectory controller `wall_sweep_executor` drives. Two publishers on one
controller is not survivable: each new goal preempts the other's. Killing
`wall_parallel_controller` is NOT enough, because its last trajectory is still
queued in the bridge and gets re-dispatched as soon as a slot opens -- observed
in Gazebo as the executor's traverse leg coming back CANCELED (JTC status 5).

The bridge cannot be imported without a ROS graph, so the three methods that
implement the hold are extracted and bound to a stub. That keeps the test on the
logic that actually matters and off the ROS plumbing.

Run with:

    python3 -m pytest test/test_trajectory_bridge_hold.py -v
"""

import os
import re

import pytest

SOURCE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "control", "publisher_joint_trajectory_planned.py",
)
METHODS = ("hold_callback", "trajectory_callback", "timer_callback")


class Bool:
    """Stands in for std_msgs/Bool; timer_callback constructs one with no args."""

    def __init__(self, data=False):
        self.data = data


class _Logger:
    def info(self, message):
        pass

    warn = error = info


class Bridge:
    """Just enough state for the dispatch logic, with goal sends counted."""

    def __init__(self):
        self.held = False
        self.trajectory_received = False
        self.planned_trajectory = None
        self.execution_complete = True
        self.starting_point_ok = True
        self.current_goal_handle = None
        self.prev_status = True
        self.sent = 0
        self._logger = _Logger()
        self.status_pub = type("_Pub", (), {"publish": lambda self, msg: None})()

    def get_logger(self):
        return self._logger

    def send_trajectory_goal(self):
        self.sent += 1


def _bind_methods():
    source = open(SOURCE).read()
    for name in METHODS:
        match = re.search(
            rf"\n    def {name}\(self.*?\n(?=    def |\n    @|\Z)", source, re.S
        )
        assert match, f"{name} not found in {SOURCE}"
        namespace = {}
        exec("class _Extracted:\n" + match.group(0), {"Bool": Bool}, namespace)
        setattr(Bridge, name, getattr(namespace["_Extracted"], name))


_bind_methods()


@pytest.fixture
def bridge():
    return Bridge()


def test_dispatches_normally_when_not_held(bridge):
    bridge.trajectory_callback(Bool(True))
    bridge.timer_callback()
    assert bridge.sent == 1


def test_holding_drops_the_pending_trajectory(bridge):
    """Releasing must not fire a target from before the sweep -- by then it is
    stale by a whole partition."""
    bridge.trajectory_callback(Bool(True))
    bridge.hold_callback(Bool(True))
    assert bridge.trajectory_received is False
    assert bridge.planned_trajectory is None


def test_held_bridge_never_dispatches(bridge):
    """The actual defect: the bridge re-sending while the executor owns the arm
    preempts the executor's goal, and the sweep returns CANCELED."""
    bridge.trajectory_callback(Bool(True))
    bridge.hold_callback(Bool(True))
    for _ in range(10):
        bridge.timer_callback()
    assert bridge.sent == 0


def test_trajectories_arriving_while_held_are_dropped_not_queued(bridge):
    bridge.hold_callback(Bool(True))
    bridge.trajectory_callback(Bool(True))
    assert bridge.trajectory_received is False
    bridge.timer_callback()
    assert bridge.sent == 0


def test_release_does_not_fire_anything_stale(bridge):
    bridge.trajectory_callback(Bool(True))
    bridge.hold_callback(Bool(True))
    bridge.hold_callback(Bool(False))
    bridge.timer_callback()
    assert bridge.sent == 0


def test_normal_operation_resumes_after_release(bridge):
    """The hold must be a pause, not a kill: the planner's own goals go through
    this bridge for every arm move outside the sweep."""
    bridge.hold_callback(Bool(True))
    bridge.hold_callback(Bool(False))
    bridge.trajectory_callback(Bool(True))
    bridge.timer_callback()
    assert bridge.sent == 1


def test_holding_cancels_whatever_is_running(bridge):
    cancelled = []
    bridge.current_goal_handle = type(
        "_Handle", (), {"cancel_goal_async": lambda self: cancelled.append(True)}
    )()
    bridge.execution_complete = False
    bridge.hold_callback(Bool(True))
    assert cancelled == [True]
    assert bridge.execution_complete is True


def test_repeated_holds_are_idempotent(bridge):
    """The FSM re-asserts the hold per partition and releases it in on_exit; a
    duplicate must not cancel a goal the executor legitimately started."""
    bridge.hold_callback(Bool(True))
    cancelled = []
    bridge.current_goal_handle = type(
        "_Handle", (), {"cancel_goal_async": lambda self: cancelled.append(True)}
    )()
    bridge.hold_callback(Bool(True))
    assert cancelled == []
