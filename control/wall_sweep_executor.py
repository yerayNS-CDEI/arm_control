#!/usr/bin/env python3
"""Sweep the sensor plate across one wall partition with the ARM, base parked.

ARM_SWEEP_PLAN §7.B. Replaces the base-driven sweep, which could not hold the
plate on a wall: force mode's compliant axis cannot track even the slowest Nav2
velocity, and the global planner routes paths away from walls, over-stretching
the arm. Here the base is stationary for the whole goal.

    SweepLine goal (partition endpoints, map frame)
        -> transform into arm_base, capture the pressed plate orientation
        -> plan the LEAD-IN: current TCP -> partition start
        -> plan the SWEEP:   partition start -> partition end
        -> validate both (no NaN / limits / branch continuity / conditioning /
           joint speed / between-waypoint TCP bow)  [sweep_trajectory.plan_sweep]
        -> execute each through this node's OWN FollowJointTrajectory client
        -> watchdog on the six plate distance sensors while it runs

Two things about this node are deliberate and easy to undo by accident:

**It does not publish to ``planned_trajectory``.** That topic goes through
``publisher_joint_trajectory_planned``, which discards velocities and re-times
everything for zero-velocity cubics -- a 0.8 m sweep would become ~27 separate
accelerate-decelerate cycles, the exact opposite of the constant-speed sweep this
exists to produce (§3.2). It also lets only one publisher drive the controller,
and ``wall_parallel_controller`` is on that topic (§3.3).

**The lead-in is a separate trajectory.** The base parks at the partition
*centre*, so after ``press_prepare`` the plate is in the middle of the partition,
not at its start. Sweeping without a lead-in would fling the plate sideways
across the wall to reach the start point; ``plan_sweep`` refuses to plan that
(``seed_not_at_start``).

Controller target is a parameter, so the same node drives
``joint_trajectory_controller`` in Gazebo and
``passthrough_trajectory_controller`` on the robot (§11.3). In Gazebo the JTC
path tolerance must be erased per joint (``-1``), because this node deliberately
sends velocities and JTC's cubic re-spline would otherwise overshoot its window.
"""

import csv
import os
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
import rclpy.time
import tf2_ros
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Path
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arm_control.action import SweepLine
from arm_control.msg import SweepDiagnostics
from planner.planner_lib.sweep_trajectory import (
    DEFAULT_MAX_INTERP_ERROR_M,
    DEFAULT_MAX_JOINT_STEP_RAD,
    DEFAULT_MIN_SINGULAR_VALUE,
    DEFAULT_SPACING_M,
    DEFAULT_SPEED_MPS,
    DEFAULT_VELOCITY_MARGIN,
    SweepPlanError,
    plan_sweep,
)
from planner.planner_lib.ur10e_kinematics import calibrate_tool0_to_dh_end, fk, pose_matrix
from sensors.wall_alignment_estimator import (
    WallAlignmentEstimator,
    correction_angles,
    mean_distance,
    usable_mask,
)

class WallSweepExecutor(Node):

    # Below this the arm is already standing at the partition start (a serpentine
    # turnaround), and a zero-length lead-in would be rejected as degenerate.
    MIN_LEAD_IN_M = 0.01

    def __init__(self):
        super().__init__("wall_sweep_executor")

        # --- Frames and joints ---------------------------------------------
        self.declare_parameter("base_frame", "arm_base")      # IK base (UR DH base)
        self.declare_parameter("tool0_frame", "arm_tool0")    # plate TCP
        self.declare_parameter("joints", [
            "arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_joint",
            "arm_wrist_1_joint", "arm_wrist_2_joint", "arm_wrist_3_joint",
        ])
        # Gazebo: joint_trajectory_controller. Real robot:
        # passthrough_trajectory_controller, the only controller the UR driver
        # will run alongside force_mode_controller (§3.1).
        self.declare_parameter("controller_name", "joint_trajectory_controller")
        # JTC enforces a path tolerance; PTC has none (§3.4). -1 erases it per
        # control_msgs/JointTolerance, which is what a velocity-carrying
        # trajectory needs from JTC. Harmless but pointless against PTC.
        self.declare_parameter("erase_path_tolerance", True)
        # PTC: leave empty, the endpoint legitimately differs along the compliant
        # Z axis (§3.4). JTC: keep meaningful, there is no force mode to excuse a
        # miss. 0 disables the check.
        self.declare_parameter("goal_tolerance_rad", 0.05)
        self.declare_parameter("goal_time_tolerance_s", 5)

        # --- Sweep generation (ARM_SWEEP_PLAN §7.D) --------------------------
        self.declare_parameter("sweep_speed_mps", DEFAULT_SPEED_MPS)
        self.declare_parameter("waypoint_spacing_m", DEFAULT_SPACING_M)
        self.declare_parameter("max_joint_step_rad", DEFAULT_MAX_JOINT_STEP_RAD)
        self.declare_parameter("min_singular_value", DEFAULT_MIN_SINGULAR_VALUE)
        self.declare_parameter("max_interpolation_error_m", DEFAULT_MAX_INTERP_ERROR_M)
        self.declare_parameter("joint_velocity_margin", DEFAULT_VELOCITY_MARGIN)
        # The lead-in crosses open air, not the wall, so it may run faster than
        # the scan itself. Kept separate so tuning scan speed never silently
        # changes how long the approach takes.
        self.declare_parameter("lead_in_speed_mps", 0.08)
        # Sanity check only (§4.2): the plate-derived lateral direction must agree
        # with the partition tangent up to sign. Disagreement means the arm is not
        # facing the partition it was told to sweep.
        self.declare_parameter("tangent_agreement_deg", 35.0)
        # How far off the wall the plate actually sweeps. Sized against the six
        # range sensors, not the detected wall line, and deliberately non-zero:
        # sweeping on the surface means contact forces in Gazebo and a scrubbing
        # GPR wheel on the robot.
        self.declare_parameter("scan_standoff_m", 0.30)
        # Extra clearance the plate backs off to after the sweep -- the same margin
        # the FSM's approach brought it in at, so the two are symmetric.
        self.declare_parameter("approach_retract_m", 0.20)
        # Longest traverse from where the arm's normal-only approach left the plate
        # to the partition start. This carries the WHOLE lateral move: the FSM
        # cannot pre-position the plate laterally, because the partition centre
        # lies on the base centreline where the arm planner's column obstacle sits.
        # Generous, because the real gate is the trajectory validation -- a
        # traverse that would flip the wrist branch is rejected by name.
        self.declare_parameter("max_traverse_m", 1.10)
        # Taking the arm over from whatever was driving it before.
        self.declare_parameter("preempt_hold_s", 0.5)
        self.declare_parameter("at_rest_tolerance_rad", 0.002)
        self.declare_parameter("at_rest_dwell_s", 0.4)
        self.declare_parameter("at_rest_timeout_s", 90.0)
        # Trajectory watchdog. Its job is to catch a controller that has HUNG, not
        # to enforce performance: measured in Gazebo, every leg completes but takes
        # 1.5-3.5x its planned duration (a 2.5 s plunge took 8.8 s). A tight
        # deadline cancels motion that is executing perfectly well, which is what
        # turned working sweeps into `controller_timeout`.
        self.declare_parameter("trajectory_timeout_factor", 4.0)
        self.declare_parameter("trajectory_timeout_pad_s", 30.0)

        # --- Contact watchdog ------------------------------------------------
        # --- Orientation replan (ARM_SWEEP_PLAN §4.2, S6) --------------------
        # OFF by default: a mid-sweep preempt is not something to switch on
        # silently. Turn it on for the S6 validation run.
        self.declare_parameter("orientation_replan_enabled", True)
        # Thresholds derived from a recorded Gazebo sweep, NOT from the plan's
        # guessed 2.0 deg. At the 0.30 m sweep standoff the ToFs are out of range,
        # so the plane fit runs on three ultrasonics with no redundancy and the
        # tilt channel carries ~1 deg of irreducible jitter. Measured over 119 s of
        # FLAT wall, where every trigger is by definition false:
        #
        #   threshold  dwell   false triggers
        #     2.0 deg   none        65        <- the plan's guess: a preempt every 2 s
        #     3.0 deg   1.0 s        1
        #     4.0 deg   1.0 s        0
        #
        # The dwell is the load-bearing part and is NOT in the plan, which
        # specifies only threshold + hysteresis. Hysteresis cannot fix a noise
        # floor; requiring the error to persist can.
        self.declare_parameter("orientation_replan_threshold_deg", 4.0)
        self.declare_parameter("orientation_replan_dwell_s", 1.0)
        self.declare_parameter("orientation_replan_hysteresis_deg", 1.0)
        self.declare_parameter("orientation_replan_min_interval_s", 5.0)
        # Lateral distance over which a correction is eased in, so the plate is
        # never asked to snap to a new orientation while against the wall.
        self.declare_parameter("orientation_blend_m", 0.10)
        # Hard cap, so a persistently noisy frame cannot preempt forever.
        self.declare_parameter("orientation_max_replans", 3)

        self.declare_parameter("sweep_contact_loss_timeout_s", 0.20)
        self.declare_parameter("contact_max_distance_m", 0.60)
        # The FT half of §7.B's two independent contact signals. Pressing reads
        # NEGATIVE on tool0 Z, same convention as the FSM's own contact gate, so
        # "contact" is Fz <= min and "too hard" is Fz <= max (both negative).
        # Contact is lost when the force relaxes back toward zero.
        self.declare_parameter("contact_min_force_n", -2.0)
        self.declare_parameter("contact_max_force_n", -25.0)
        self.declare_parameter("watchdog_enabled", True)
        self.declare_parameter("diagnostics_enabled", True)
        self.declare_parameter("ft_topic", "/force_torque_sensor_broadcaster/wrench")
        # Directory for one CSV per sweep, alongside the topic. Empty disables it.
        self.declare_parameter("diagnostics_csv_dir", "")
        self.declare_parameter("diagnostics_rate_hz", 20.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.tool0_frame = self.get_parameter("tool0_frame").value
        self.joint_names = list(self.get_parameter("joints").value)

        # --- State -----------------------------------------------------------
        self._joint_state = None
        self._distances = None
        self._distances_stamp = 0.0
        self._T_tool0_dhend = None      # constant tool0 -> IK DH end frame
        self._abort_reason = None       # set by the watchdog, read by the sweep loop
        self._active_goal = None        # in-flight FollowJointTrajectory handle
        self._contact_lost_since = None  # start of the current contact dropout
        self._last_sample = None        # (time, TCP) for the measured-speed estimate
        self._lateral_speed = 0.0       # produced by the diagnostics, read by the feedback
        self._wrench = None             # latest TCP wrench (absent in Gazebo)
        self._alignment = WallAlignmentEstimator()   # read-only wall estimate
        self._replans = 0               # orientation-correction replans (S6)
        self._tilt_over_since = None    # start of the current threshold exceedance
        self._tilt_armed = True         # hysteresis: cleared until the error recovers
        self._last_replan = 0.0
        self._csv = None                # per-sweep CSV writer, when enabled
        self._csv_file = None
        self._goal_started = 0.0        # node-clock t0 for the diagnostics timeline
        self._lock = threading.Lock()

        callbacks = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            JointState, "joint_states", self._on_joint_state, 10, callback_group=callbacks
        )
        self.create_subscription(
            Float32MultiArray, "distance_sensors", self._on_distances, 10,
            callback_group=callbacks,
        )
        # The FT half of the contact watchdog and the force channel of the
        # diagnostics. Absent in Gazebo (no force_torque_sensor_broadcaster), which
        # is why every use of it is guarded rather than assumed.
        self.create_subscription(
            WrenchStamped, self.get_parameter("ft_topic").value, self._on_wrench, 10,
            callback_group=callbacks,
        )

        # RViz: the planned sweep, published BEFORE execution. Catches a bad
        # tangent or a rejected partition without waiting for the arm to move
        # (§11.4).
        self._path_pub = self.create_publisher(Path, "~/planned_sweep", 1)
        self._desired_pub = self.create_publisher(PoseStamped, "~/desired_tcp", 10)
        self._measured_pub = self.create_publisher(PoseStamped, "~/measured_tcp", 10)
        # Everything §7.B asks to record, on one topic, so a single `ros2 bag` of a
        # run is enough to plot TCP path, speed, force and orientation error.
        self._diag_pub = self.create_publisher(SweepDiagnostics, "~/diagnostics", 20)

        controller = self.get_parameter("controller_name").value
        self._traj_client = ActionClient(
            self, FollowJointTrajectory, f"{controller}/follow_joint_trajectory",
            callback_group=callbacks,
        )
        self._server = ActionServer(
            self, SweepLine, "sweep_line",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=callbacks,
        )
        self.get_logger().info(
            f"wall_sweep_executor ready: {self.base_frame} -> {self.tool0_frame}, "
            f"driving '{controller}'."
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _on_joint_state(self, msg):
        with self._lock:
            self._joint_state = msg

    def _on_distances(self, msg):
        with self._lock:
            self._distances = list(msg.data)
            self._distances_stamp = time.time()

    def _on_wrench(self, msg):
        with self._lock:
            self._wrench = msg.wrench

    def press_force(self):
        """Wall-normal press force (N, signed), or None when no wrench has arrived.

        Force mode presses along task-frame Z, which the broadcaster reports in
        the tool0 frame, so ``force.z`` is the wall-normal component and pressing
        reads NEGATIVE. Same convention as the FSM's ``_measured_press_force`` --
        the two must not disagree about the sign of contact.
        """
        with self._lock:
            wrench = self._wrench
        return None if wrench is None else float(wrench.force.z)

    def current_joints(self):
        """The six arm joints in IK order, or None.

        Ordered by name rather than by index: ``joint_states`` is merged from
        several sources on this robot (base, column, turret, arm), so positional
        assumptions about the array break the moment a publisher changes.
        """
        with self._lock:
            msg = self._joint_state
        if msg is None:
            return None
        try:
            return np.array([msg.position[msg.name.index(n)] for n in self.joint_names])
        except (ValueError, IndexError):
            return None

    def fresh_distances(self):
        """The last ``distance_sensors`` frame, or None when stale/absent."""
        with self._lock:
            readings, stamp = self._distances, self._distances_stamp
        if not readings or len(readings) != 6 or time.time() - stamp > 3.0:
            return None
        return np.asarray(readings, dtype=float)

    def plate_wall_distance(self):
        """Mean plate-to-wall distance over the valid ranges, or None.

        Delegates to the shared estimator so the executor, the alignment
        controller and the FSM all agree on which readings count. One valid
        reading is enough -- a standoff needs no plane fit.
        """
        d = self.fresh_distances()
        return None if d is None else mean_distance(d)

    def wall_alignment(self):
        """Latest read-only wall-alignment estimate, or None.

        The whole point of ``wall_alignment_estimator`` living outside
        ``wall_parallel_controller``: the executor needs the identical geometry
        while owning the trajectory controller, and must never publish an arm
        command from it (§3.3, §4.2).
        """
        d = self.fresh_distances()
        return None if d is None else self._alignment.update(d)

    # ------------------------------------------------------------------
    # TF
    # ------------------------------------------------------------------
    def _lookup(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(), Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f"{target}->{source} lookup failed: {e}")
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return pose_matrix(
            R.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), [t.x, t.y, t.z]
        )

    def tool0_to_dh_end(self, q_current):
        """Constant ``tool0 -> DH end frame`` transform, calibrated once.

        The analytic IK solves for its own DH end frame, which differs from the
        URDF's ``tool0`` by a fixed rigid transform. Measuring it from FK(q)
        against the live TF -- the trick ``wall_parallel_controller`` already
        uses -- beats deriving it from flange conventions and hoping (§7.B).
        """
        if self._T_tool0_dhend is not None:
            return self._T_tool0_dhend
        T_base_tool0 = self._lookup(self.base_frame, self.tool0_frame)
        if T_base_tool0 is None:
            return None
        self._T_tool0_dhend = calibrate_tool0_to_dh_end(T_base_tool0, q_current)
        self.get_logger().info(
            f"Calibrated {self.tool0_frame}->DH-end "
            f"(trans={np.round(self._T_tool0_dhend[:3, 3], 4).tolist()})"
        )
        return self._T_tool0_dhend

    # ------------------------------------------------------------------
    # Action server
    # ------------------------------------------------------------------
    def _on_goal(self, goal_request):
        del goal_request
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        del goal_handle
        self._abort("cancelled", "cancel requested by the FSM")
        return CancelResponse.ACCEPT

    def _abort(self, reason, detail):
        with self._lock:
            if self._abort_reason is None:
                self._abort_reason = (reason, detail)
        handle = self._active_goal
        if handle is not None:
            handle.cancel_goal_async()

    @staticmethod
    def _result(success, reason="", detail="", length=0.0, duration=0.0):
        result = SweepLine.Result()
        result.success = success
        result.reason = reason
        result.detail = detail
        result.swept_length = float(length)
        result.duration = float(duration)
        return result

    def _execute(self, goal_handle):
        """Plan, validate, and run one partition sweep.

        Failure here is never partial: nothing reaches the controller unless the
        whole trajectory validated. The FSM owns the cleanup that follows
        (GPR stop -> force-mode stop -> retract), so this returns a reason and
        stops rather than trying to recover (§7.B).
        """
        request = goal_handle.request
        with self._lock:
            self._abort_reason = None
        self._contact_lost_since = None
        self._last_sample = None
        self._lateral_speed = 0.0
        self._replans = 0
        self._tilt_over_since = None
        self._tilt_armed = True
        self._last_replan = 0.0
        self._alignment.reset()
        self._open_csv(request)
        self._goal_started = self._now()
        started = time.time()

        try:
            legs, sweep = self._plan(request)
        except SweepPlanError as e:
            self.get_logger().error(f"Sweep rejected [{e.reason}]: {e.detail}")
            goal_handle.abort()
            return self._result(False, e.reason, e.detail)
        except RuntimeError as e:
            self.get_logger().error(f"Sweep setup failed: {e}")
            goal_handle.abort()
            return self._result(False, "setup_failed", str(e))

        self._publish_planned_path(sweep)
        self.get_logger().info(
            "Partition planned: "
            + ", ".join(f"{name} {len(p)} wp / {p.duration:.1f}s" for name, p in legs)
            + f" — sweep covers {sweep.length:.3f} m at {sweep.speed:.3f} m/s "
            f"(step {sweep.max_joint_step:.3f} rad, sigma_min "
            f"{sweep.min_singular_value:.3f}, {sweep.peak_velocity_ratio * 100:.0f}% "
            f"of rated speed, TCP bow {sweep.max_interpolation_error * 1000:.2f} mm)."
        )
        for warning in sweep.warnings:
            self.get_logger().warn(f"Sweep margin: {warning}")

        # The contact watchdog belongs to the sweep alone. The plunge and retract
        # deliberately cross the standoff band, so "the plate is far from the wall"
        # is the expected state there, not a fault.
        press = bool(self.get_parameter("watchdog_enabled").value) and request.press
        swept = 0.0
        for name, plan in legs:
            if name == "sweep":
                ok, reason, detail = self._run_sweep_with_replans(
                    plan, goal_handle, sweep, press, request
                )
                if not ok:
                    return self._finish(goal_handle, False, reason, detail,
                                        self._progress(sweep) * sweep.length, started)
                swept = sweep.length
                continue
            # Publish the phase before the leg starts, not on the next 10 Hz tick:
            # the FSM starts the GPR line on the "sweep" phase and every
            # millisecond of delay is unrecorded wall.
            self._publish_feedback(goal_handle, sweep, name)
            ok, reason, detail = self._run(
                plan, goal_handle, sweep, watch=(press and name == "sweep"), phase=name,
                request=request,
            )
            if not ok:
                if name == "retract":
                    # The scan itself is complete and the GPR line is recorded, so
                    # report success and let the FSM count it. Backing the plate off
                    # still has to happen before the base moves, but transit_clear
                    # already does that from a measured distance -- failing the whole
                    # partition here would throw away a good measurement over a
                    # cleanup step that has its own retry.
                    self.get_logger().error(
                        f"Sweep completed but the retract failed [{reason}]: {detail}. "
                        f"The plate is still at the scan standoff; transit_clear must "
                        f"pull it back before the base moves."
                    )
                    return self._finish(
                        goal_handle, True, reason, detail, sweep.length, started
                    )
                return self._finish(goal_handle, False, reason, detail, swept, started)

        return self._finish(goal_handle, True, "", "", sweep.length, started)

    def _run_sweep_with_replans(self, plan, goal_handle, sweep, press, request):
        """Run the scan leg, regenerating the remainder whenever the plate drifts.

        The preempt/replan cycle of §4.2: PTC cannot have a trajectory edited in
        place, so a correction means cancelling the goal and sending a replacement
        for what is left. Force Mode is untouched throughout -- it lives in a
        different controller and keeps pressing across the gap.
        """
        remaining = plan
        while True:
            ok, reason, detail = self._run(
                remaining, goal_handle, sweep, watch=press, phase="sweep",
                request=request,
            )
            if ok or reason != "orientation_replan":
                return ok, reason, detail

            with self._lock:
                self._abort_reason = None      # the replan consumed it
            replacement = self._replan_remainder(sweep)
            if replacement is None:
                return False, "orientation_replan_failed", (
                    "the plate drifted off the wall and the corrected trajectory "
                    "did not validate"
                )
            self._replans += 1
            remaining = replacement
            self.get_logger().info(
                f"Replan {self._replans}: {len(replacement)} waypoints / "
                f"{replacement.duration:.1f}s to finish the partition."
            )

    def _finish(self, goal_handle, success, reason, detail, length, started):
        self._close_csv()
        duration = time.time() - started
        if success:
            goal_handle.succeed()
            self.get_logger().info(
                f"Sweep complete: {length:.3f} m in {duration:.1f}s."
            )
        else:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            self.get_logger().error(f"Sweep failed [{reason}]: {detail}")
        return self._result(success, reason, detail, length, duration)

    # ------------------------------------------------------------------
    # Taking over the arm
    # ------------------------------------------------------------------
    def _now(self):
        """Seconds on the NODE clock -- sim time in Gazebo, wall time on the robot.

        The trajectory controller advances trajectories on this clock, so any
        deadline compared against a trajectory's planned duration has to use it
        too. Mixing `time.time()` with a sim clock running below real time makes
        the watchdog fire early on trajectories that are running fine.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    def _preempt_and_settle(self):
        """Stop whatever else is driving the arm, then wait until it is at rest.

        Killing ``wall_parallel_controller`` does **not** stop the motion it has
        already handed to the trajectory controller -- the process dies, the goal
        keeps executing. Observed in Gazebo as a 67 s trajectory still running
        three seconds after the FSM reported the controller stopped, which put the
        plate most of a metre from where planning assumed it was.

        So: send a one-point goal at the current position. Both JTC and PTC preempt
        the in-flight goal when a new one arrives, and a goal that says "stay here"
        brings the arm to a stop rather than anywhere new. Then wait for the joint
        readings to actually go quiet before anyone reads them.

        Returns an error string, or None when the arm is stopped and steady.
        """
        q_now = self.current_joints()
        if q_now is None:
            return "no joint_states for the six arm joints yet"

        if self._traj_client.wait_for_server(timeout_sec=5.0):
            traj = JointTrajectory()
            traj.joint_names = list(self.joint_names)
            point = JointTrajectoryPoint()
            point.positions = q_now.tolist()
            point.velocities = [0.0] * len(self.joint_names)
            hold = float(self.get_parameter("preempt_hold_s").value)
            point.time_from_start = DurationMsg(
                sec=int(hold), nanosec=int((hold % 1.0) * 1e9)
            )
            traj.points.append(point)
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            # No tolerances: this goal exists to preempt, and the arm decelerating
            # out of someone else's trajectory will not hit a position window.
            goal.path_tolerance = [
                JointTolerance(name=n, position=-1.0) for n in self.joint_names
            ]
            self._spin_until(self._traj_client.send_goal_async(goal), timeout=2.0)

        return self._wait_until_at_rest()

    def _wait_until_at_rest(self):
        """Block until the arm joints stop changing, or time out.

        Position differencing rather than the velocity field: ``joint_states`` is
        merged from several publishers on this robot and velocities are not always
        populated, whereas positions always are.
        """
        settle_tol = float(self.get_parameter("at_rest_tolerance_rad").value)
        settle_for = float(self.get_parameter("at_rest_dwell_s").value)
        timeout = float(self.get_parameter("at_rest_timeout_s").value)

        deadline = time.time() + timeout
        reference, quiet_since = self.current_joints(), time.time()
        while rclpy.ok() and time.time() < deadline:
            time.sleep(0.05)
            q = self.current_joints()
            if q is None or reference is None:
                reference, quiet_since = q, time.time()
                continue
            if float(np.max(np.abs(q - reference))) > settle_tol:
                reference, quiet_since = q, time.time()
                continue
            if time.time() - quiet_since >= settle_for:
                return None
        return (
            f"the arm was still moving {timeout:.0f}s after the sweep goal arrived; "
            f"something else is still commanding it"
        )

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------
    def _plan(self, request):
        """The three legs of one partition, all validated before any is sent.

        Returns ``([(phase, plan), ...], sweep)`` -- the legs in execution order
        plus the sweep itself, which is what progress and feedback are measured
        against.

            plunge   close in from the approach standoff onto the sweep plane
            sweep    cross the partition, plate held off the wall
            retract  back the plate off before the base moves again

        Raises ``SweepPlanError`` (a sweep that must not run) or ``RuntimeError``
        (missing state: no joints, no TF, no distance reading).
        """
        # Nothing may be read until the arm has actually stopped: the previous
        # phase's alignment trajectory can still be executing when this goal
        # arrives, and planning off a moving joint state puts the whole sweep in
        # the wrong place.
        problem = self._preempt_and_settle()
        if problem is not None:
            raise RuntimeError(problem)

        q_current = self.current_joints()
        if q_current is None:
            raise RuntimeError("no joint_states for the six arm joints yet")

        T_tool0_dhend = self.tool0_to_dh_end(q_current)
        if T_tool0_dhend is None:
            raise RuntimeError(f"no {self.base_frame}->{self.tool0_frame} TF to calibrate")

        # arm_base is static for the whole sweep (base parked, turret fixed), so
        # solving in it is safe -- and one transform at the start is enough (§7.B).
        frame = request.frame_id or "map"
        if frame == self.base_frame:
            T_base_frame = np.eye(4)
        else:
            T_base_frame = self._lookup(self.base_frame, frame)
            if T_base_frame is None:
                raise RuntimeError(f"no {self.base_frame}->{frame} TF for the goal points")

        def to_base(point):
            return (T_base_frame @ np.array([point.x, point.y, point.z, 1.0]))[:3]

        p_start, p_end = to_base(request.start), to_base(request.end)

        # The commanded plate orientation for the whole sweep: whatever
        # press_prepare achieved. RX/RY are not force-compliant, so this command
        # is the only thing holding the plate flat against the wall (§4.2).
        T_current = fk(q_current)
        rotation = T_current[:3, :3]      # DH end frame -- the IK's own convention
        p_current = T_current[:3, 3]

        # The goal points describe where the PLATE (tool0) should go; the IK
        # solves for the DH end frame. The two differ by the calibrated rigid
        # transform, and because the plate orientation is held constant across the
        # whole sweep, that reduces to one constant translation -- so shifting the
        # endpoints is exact rather than an approximation.
        #   T_base_dhend = T_base_tool0 @ T_tool0_dhend
        #   => R_tool0 = R_dhend @ R_tool0_dhend^T
        #   => p_dhend = p_tool0 + R_tool0 @ t_tool0_dhend
        rotation_tool0 = rotation @ T_tool0_dhend[:3, :3].T
        offset = rotation_tool0 @ T_tool0_dhend[:3, 3]
        p_start, p_end = p_start + offset, p_end + offset

        self._check_tangent(p_start, p_end, rotation_tool0)

        speed = float(request.speed) if request.speed > 0.0 else float(
            self.get_parameter("sweep_speed_mps").value
        )
        limits = self._limits()

        # --- Where the sweep plane actually is -------------------------------
        #
        # NOT on the partition line. That line lies on the detected wall surface,
        # and sweeping along it would drive the plate into the wall. Lateral and
        # vertical placement come from the partition geometry, which is
        # authoritative there; the wall-normal offset does not.
        #
        # Who owns the wall-normal axis depends on whether Force Mode is pressing,
        # and the two cases are opposites:
        n_in = rotation_tool0[:, 2]      # plate +Z: the sensing/press axis

        if request.press:
            # REAL ROBOT. Force Mode owns this axis. press_settle has already
            # driven the plate into the wall and the FT sensor has confirmed
            # contact at scan_wall_touch_force_n, so the plate is exactly where it
            # belongs -- sweep in the plane it is already in.
            #
            # Emphatically do NOT size a standoff from the range sensors here.
            # In contact they read below their valid floor, so
            # plate_wall_distance() returns None; and any non-zero plunge would
            # either push harder than Force Mode intends or lift the plate off the
            # wall it just pressed against.
            plunge = 0.0
            p_plane = p_current
            self.get_logger().info(
                "Force Mode is pressing: sweeping in the plate's current plane and "
                "leaving the wall-normal axis to it."
            )
        else:
            # NO PRESS (Gazebo, or the first real-robot run per §11.2). Nothing
            # holds the plate off the wall, so the executor does: hold it
            # `scan_standoff_m` out, sized from the MEASURED distance rather than
            # the partition geometry, because the six range sensors know where the
            # wall really is and the detected line can be centimetres out.
            scan_standoff = float(self.get_parameter("scan_standoff_m").value)
            measured = self.plate_wall_distance()
            if measured is None:
                raise RuntimeError(
                    "no valid plate distance reading; cannot size the sweep standoff "
                    "without knowing where the wall is"
                )
            plunge = measured - scan_standoff
            p_plane = p_current + n_in * plunge      # where the plate ends up
            self.get_logger().info(
                f"Plate measured {measured:.3f} m off the wall; sweeping at "
                f"{scan_standoff:.3f} m (plunge {plunge:+.3f} m)."
            )

        def into_plane(point):
            """Project a wall-surface point into the plate's sweep plane."""
            return point - n_in * float(np.dot(point - p_plane, n_in))

        p_scan_start, p_scan_end = into_plane(p_start), into_plane(p_end)

        # The FSM's approach leaves the plate centred on the partition; the sweep
        # has to begin at one END of it, so there is a real lateral move first.
        # WHERE that move happens depends on who owns the wall-normal axis:
        #
        # - **Force Mode pressing**: stay in the contact plane. The GPR is a WHEEL
        #   -- it rolls along the wall, which is what it is for -- and commanding a
        #   0.20 m retreat here would fight the press directly and trip its
        #   deviation limits. So: no retreat, no plunge, just roll to the start.
        # - **No press**: nothing holds the plate off the wall, so the executor
        #   backs it off for the lateral move and plunges back in at the start.
        retract = 0.0 if request.press else float(
            self.get_parameter("approach_retract_m").value
        )
        p_safe_start = p_scan_start - n_in * retract
        traverse_distance = float(np.linalg.norm(p_safe_start - p_current))
        max_traverse = float(self.get_parameter("max_traverse_m").value)
        if traverse_distance > max_traverse:
            raise SweepPlanError(
                "approach_too_far",
                f"the plate is {traverse_distance:.2f} m from the partition start "
                f"(limit {max_traverse:.2f} m); the arm approach did not leave it "
                f"centred on this partition, and a move that long as a straight "
                f"Cartesian line flips the wrist branch rather than routing around "
                f"the singularity",
            )

        self.get_logger().info(
            f"Traverse {traverse_distance:.3f} m to the partition start, "
            + ("in the contact plane (Force Mode holds the wheel on the wall)."
               if request.press else f"{retract:.2f} m clear of the sweep plane.")
        )

        legs = []
        q_seed = q_current
        approach_speed = float(self.get_parameter("lead_in_speed_mps").value)

        def add(name, start, end, leg_speed):
            nonlocal q_seed
            if float(np.linalg.norm(np.asarray(end) - np.asarray(start))) <= self.MIN_LEAD_IN_M:
                return
            leg = plan_sweep(start, end, rotation, q_seed, speed=leg_speed, **limits)
            legs.append((name, leg))
            q_seed = leg.q[-1]

        # 1. Walk sideways to the partition start, held clear of the wall.
        add("traverse", p_current, p_safe_start, approach_speed)
        # 2. Close the remaining standoff: a pure wall-normal move.
        add("plunge", p_safe_start, p_scan_start, approach_speed)

        # 3. The scan itself -- always planned, even if it is the only leg.
        sweep = plan_sweep(p_scan_start, p_scan_end, rotation, q_seed, speed=speed, **limits)
        legs.append(("sweep", sweep))
        q_seed = sweep.q[-1]

        # 4. Back off by the margin the approach came in at, so the plate is clear
        #    of the wall before the base moves to the next partition.
        #
        #    Not under Force Mode: it is still pressing when the executor finishes
        #    (the FSM stops it only after the sweep result), so retracting here
        #    would fight it. transit_clear pulls the plate back from a measured
        #    distance once the press is released.
        if not request.press:
            add("retract", p_scan_end, p_scan_end - n_in * retract, approach_speed)

        return legs, sweep

    def _check_tangent(self, p_start, p_end, rotation_tool0):
        """Cross-check the partition tangent against the plate's own lateral axis.

        The partition geometry is authoritative -- the plate-derived direction
        depends on where the arm happens to be pointing (§4.2). But if the two
        disagree by more than a wide margin, the arm is not facing the partition
        it was told to sweep, and sweeping anyway drags the plate across the wall
        at an angle. Sign is ignored: a serpentine partition is swept both ways.

        Takes the **tool0** rotation, not the DH end frame's: the sensing axis the
        distance sensors and force mode both work along is tool0 +Z, and the two
        frames differ by roughly 120 degrees on this robot.
        """
        tangent = p_end - p_start
        tangent = tangent / np.linalg.norm(tangent)
        # cross(plate +Z, world up) is the plate's horizontal lateral axis.
        z_plate = rotation_tool0[:, 2]
        lateral = np.cross(z_plate, np.array([0.0, 0.0, 1.0]))
        norm = np.linalg.norm(lateral)
        if norm < 1e-6:
            self.get_logger().warn(
                "Plate +Z is vertical; skipping the tangent sanity check."
            )
            return
        agreement = abs(float(np.dot(tangent, lateral / norm)))
        tolerance = np.cos(np.deg2rad(float(self.get_parameter("tangent_agreement_deg").value)))
        if agreement < tolerance:
            raise SweepPlanError(
                "tangent_disagreement",
                f"partition tangent is {np.rad2deg(np.arccos(min(agreement, 1.0))):.1f} deg "
                f"off the plate's lateral axis; the arm is not square to this partition",
            )

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------
    def _to_msg(self, plan):
        """Turn a SweepPlan into a FollowJointTrajectory goal.

        Positions AND velocities AND explicit times -- the whole reason this node
        does not go through ``publisher_joint_trajectory_planned``, which would
        strip the first and recompute the last (§3.2).
        """
        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)
        for t, q, qdot in zip(plan.times, plan.q, plan.qdot):
            point = JointTrajectoryPoint()
            point.positions = q.tolist()
            point.velocities = qdot.tolist()
            point.time_from_start = DurationMsg(
                sec=int(t), nanosec=int((t % 1.0) * 1e9)
            )
            traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        # Generous: the real robot's speed scaling can stretch a trajectory, and a
        # sweep aborted for finishing late is a scan lost to a clock, not a fault.
        goal.goal_time_tolerance = DurationMsg(
            sec=int(self.get_parameter("goal_time_tolerance_s").value)
        )
        if bool(self.get_parameter("erase_path_tolerance").value):
            # -1 erases the tolerance per control_msgs/JointTolerance (0 would
            # mean "keep the default"). JTC's cubic re-spline of a
            # velocity-carrying trajectory overshoots its window otherwise, which
            # is the concrete Gazebo trap in §11.3.
            goal.path_tolerance = [
                JointTolerance(name=n, position=-1.0) for n in self.joint_names
            ]
        tolerance = float(self.get_parameter("goal_tolerance_rad").value)
        if tolerance > 0.0:
            goal.goal_tolerance = [
                JointTolerance(name=n, position=tolerance) for n in self.joint_names
            ]
        return goal

    def _run(self, plan, goal_handle, sweep, watch, phase, request):
        """Send one trajectory and block until it finishes, aborts, or the
        watchdog trips. Returns ``(ok, reason, detail)``."""
        if not self._traj_client.wait_for_server(timeout_sec=5.0):
            return False, "controller_unavailable", (
                f"{self.get_parameter('controller_name').value}"
                f"/follow_joint_trajectory did not come up"
            )

        send = self._traj_client.send_goal_async(self._to_msg(plan))
        handle = self._spin_until(send, timeout=5.0)
        if handle is None:
            return False, "controller_silent", (
                f"the controller never answered the {phase} trajectory goal"
            )
        if not handle.accepted:
            return False, "controller_rejected", (
                f"the controller refused the {phase} trajectory — usually a joint "
                f"name or ordering mismatch with what it is configured for"
            )
        self._active_goal = handle

        result_future = handle.get_result_async()
        outcome = self._monitor(
            result_future, plan, goal_handle, sweep, watch, phase, request
        )
        self._active_goal = None
        return outcome

    def _monitor(self, result_future, plan, goal_handle, sweep, watch, phase, request):
        """Poll the trajectory to completion, running the watchdog and publishing
        feedback + diagnostics as it goes."""
        period = 1.0 / max(float(self.get_parameter("diagnostics_rate_hz").value), 1.0)
        replan_allowed = (
            phase == "sweep"
            and bool(self.get_parameter("orientation_replan_enabled").value)
            and self._replans < int(self.get_parameter("orientation_max_replans").value)
        )
        # Proportional slack, not a flat margin: whatever slows a trajectory down
        # slows a long one more than a short one. Deliberately loose -- see
        # trajectory_timeout_factor.
        factor = float(self.get_parameter("trajectory_timeout_factor").value)
        pad = float(self.get_parameter("trajectory_timeout_pad_s").value)
        started = self._now()
        deadline = started + plan.duration * factor + pad
        last = time.time()

        while rclpy.ok():
            if result_future.done():
                status = result_future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    # Log the ratio: the arm consistently runs slower than its
                    # trajectory asks for, and this is the only place that gap is
                    # visible. If it ever approaches trajectory_timeout_factor the
                    # watchdog is about to start firing on healthy motion again.
                    actual = self._now() - started
                    self.get_logger().info(
                        f"{phase} leg done in {actual:.1f}s "
                        f"(planned {plan.duration:.1f}s, "
                        f"{actual / max(plan.duration, 1e-3):.1f}x)."
                    )
                    return True, "", ""
                with self._lock:
                    aborted = self._abort_reason
                if aborted is not None:
                    return False, aborted[0], aborted[1]
                return False, "controller_failed", (
                    f"{phase} trajectory finished with status {status}"
                )

            with self._lock:
                aborted = self._abort_reason
            if aborted is not None:
                handle = self._active_goal
                if handle is not None:
                    handle.cancel_goal_async()
                return False, aborted[0], aborted[1]

            if watch:
                problem = self._watchdog_tick()
                if problem is not None:
                    self._abort(*problem)

            # Orientation drift: preempt so the remainder can be regenerated with a
            # corrected plate orientation (§4.2). Only during the scan itself --
            # the traverse and retract deliberately leave the wall, so the plate is
            # not meant to be parallel to it there.
            if replan_allowed and self._orientation_drifted():
                self._abort("orientation_replan", "plate drifted off the wall")

            now = time.time()
            if now - last >= period:
                last = now
                self._publish_diagnostics(request, sweep, phase)
                self._publish_feedback(goal_handle, sweep, phase)
            if self._now() > deadline:
                self._abort(
                    "controller_timeout",
                    f"{phase} trajectory has run {self._now() - started:.1f}s against "
                    f"a planned {plan.duration:.1f}s without reporting a result "
                    f"(limit {plan.duration * factor + pad:.1f}s); the controller "
                    f"looks hung rather than slow",
                )
            time.sleep(0.02)

        return False, "shutdown", "node shut down mid-sweep"

    # ------------------------------------------------------------------
    # Watchdog and diagnostics
    # ------------------------------------------------------------------
    def _watchdog_tick(self):
        """Both independent contact signals, per §7.B.

        Excessive force aborts IMMEDIATELY -- it is the one failure where waiting
        out a dwell is itself the damage. Everything else needs the loss to
        persist for ``sweep_contact_loss_timeout_s``: the range sensors are
        specular against some surfaces and the FT reading is noisy, so a single
        bad frame must not abandon a partition.
        """
        force = self.press_force()
        too_hard = float(self.get_parameter("contact_max_force_n").value)
        if force is not None and force <= too_hard:
            return "excessive_force", (
                f"press force {force:.1f} N is past the {too_hard:.1f} N limit; "
                f"stopping immediately rather than waiting out a dwell"
            )

        # Distance: the plate has come off the wall.
        limit = float(self.get_parameter("contact_max_distance_m").value)
        distance = self.plate_wall_distance()
        lost_distance = distance is None or distance > limit
        detail = (f"plate {'out of range' if distance is None else f'{distance:.3f} m'} "
                  f"off the wall (limit {limit:.2f} m)")

        # Force: the press has relaxed back toward zero. Independent of the
        # distance signal by design -- a plate resting on a ledge reads close but
        # unloaded, and one on a dark surface reads unloaded but is in contact.
        # Only consulted where a wrench actually exists.
        minimum = float(self.get_parameter("contact_min_force_n").value)
        lost_force = force is not None and force > minimum
        if lost_force:
            detail = f"press force {force:.1f} N has relaxed past {minimum:.1f} N"
        if lost_distance and lost_force:
            detail = (f"plate {'out of range' if distance is None else f'{distance:.3f} m'} "
                      f"off the wall AND press force {force:.1f} N relaxed")

        if not (lost_distance or lost_force):
            self._contact_lost_since = None
            return None

        since = self._contact_lost_since
        if since is None:
            self._contact_lost_since = time.time()
            return None
        held = time.time() - since
        if held < float(self.get_parameter("sweep_contact_loss_timeout_s").value):
            return None
        return "contact_lost", f"{detail}, sustained for {held:.2f}s"

    def _limits(self):
        """Validation limits for `plan_sweep`, in one place so the initial plan and
        every replanned remainder are held to identical standards."""
        return dict(
            spacing=float(self.get_parameter("waypoint_spacing_m").value),
            max_joint_step=float(self.get_parameter("max_joint_step_rad").value),
            min_singular_value=float(self.get_parameter("min_singular_value").value),
            max_interpolation_error=float(
                self.get_parameter("max_interpolation_error_m").value
            ),
            velocity_margin=float(self.get_parameter("joint_velocity_margin").value),
        )

    def _orientation_drifted(self):
        """True when the plate has been off the wall by more than the threshold
        for long enough to be drift rather than noise.

        Three gates, and the middle one is the one that matters:

        - **threshold** -- how far off parallel counts as wrong;
        - **dwell** -- how long it must stay there. At the sweep standoff the tilt
          channel carries ~1 deg of jitter (three ultrasonics, no redundancy), so
          an instantaneous threshold fires on noise: measured, 2.0 deg with no
          dwell triggered 65 times in 119 s of flat wall;
        - **cooldown + hysteresis** -- so a value sitting on the threshold cannot
          preempt the trajectory over and over (§4.2).
        """
        alignment = self.wall_alignment()
        if alignment is None:
            return False
        threshold = float(self.get_parameter("orientation_replan_threshold_deg").value)
        hysteresis = float(self.get_parameter("orientation_replan_hysteresis_deg").value)

        if alignment.tilt_deg <= threshold - hysteresis:
            self._tilt_armed = True          # recovered: allow the next trigger
        if alignment.tilt_deg <= threshold:
            self._tilt_over_since = None
            return False

        now = self._now()
        if self._tilt_over_since is None:
            self._tilt_over_since = now
            return False
        if not self._tilt_armed:
            return False
        if now - self._tilt_over_since < float(
                self.get_parameter("orientation_replan_dwell_s").value):
            return False
        if now - self._last_replan < float(
                self.get_parameter("orientation_replan_min_interval_s").value):
            return False

        self._tilt_armed = False
        self._tilt_over_since = None
        self._last_replan = now
        self.get_logger().warn(
            f"Plate is {alignment.tilt_deg:.2f} deg off the wall (threshold "
            f"{threshold:.1f}) and has stayed there; replanning the rest of the sweep."
        )
        return True

    def _replan_remainder(self, sweep):
        """Regenerate the rest of the sweep with a corrected plate orientation.

        From wherever the arm actually is to the ORIGINAL partition endpoint
        (§4.2 step 5), with the correction eased in over ``orientation_blend_m``
        rather than applied as a step. Returns a validated plan, or None with the
        reason logged -- a rejected correction leaves the sweep aborted rather
        than sending something unvalidated.
        """
        q = self.current_joints()
        alignment = self.wall_alignment()
        if q is None or alignment is None:
            self.get_logger().error("Cannot replan: no joints or no wall estimate.")
            return None

        current = fk(q)
        # The correction that nulls the measured wall normal, applied in the plate
        # frame -- the same geometry wall_parallel_controller uses to drive it.
        beta, gamma = correction_angles(alignment.normal)
        increment = R.from_euler("ZYX", [0.0, beta, gamma]).as_matrix()
        corrected = current[:3, :3] @ increment

        try:
            return plan_sweep(
                current[:3, 3], sweep.tcp[-1], current[:3, :3], q,
                speed=sweep.speed,
                rotation_to=corrected,
                blend_distance=float(self.get_parameter("orientation_blend_m").value),
                **self._limits(),
            )
        except SweepPlanError as e:
            self.get_logger().error(
                f"Orientation correction rejected [{e.reason}]: {e.detail}. "
                f"Leaving the sweep aborted rather than sending it unvalidated."
            )
            return None

    def _measured_tcp(self):
        q = self.current_joints()
        return None if q is None else fk(q)[:3, 3]

    def _progress(self, sweep):
        """Fraction of the partition swept, from the TCP projected onto the tangent.

        Projection rather than elapsed time: if the robot's speed scaling slowed
        the trajectory down, time would overstate how much wall was covered.
        """
        measured = self._measured_tcp()
        if measured is None:
            return 0.0
        tangent = sweep.tcp[-1] - sweep.tcp[0]
        length = float(np.linalg.norm(tangent))
        if length < 1e-9:
            return 0.0
        travelled = float(np.dot(measured - sweep.tcp[0], tangent / length))
        return float(np.clip(travelled / length, 0.0, 1.0))

    def _publish_feedback(self, goal_handle, sweep, phase):
        """Feedback is always reported against the SWEEP, never the lead-in, so
        `progress` means "fraction of wall covered" throughout -- it simply sits
        near zero while the arm travels to the partition start.

        ``phase`` is what tells the FSM when the plate actually starts crossing
        the wall, so it can start the GPR line then rather than at goal
        acceptance (which would record the lead-in travel as scan data).
        """
        measured = self._measured_tcp()
        feedback = SweepLine.Feedback()
        feedback.phase = phase
        feedback.progress = self._progress(sweep)
        feedback.wall_distance = self.plate_wall_distance() or float("nan")
        feedback.replans = int(self._replans)

        if measured is not None:
            tangent = sweep.tcp[-1] - sweep.tcp[0]
            tangent = tangent / np.linalg.norm(tangent)
            nominal = sweep.tcp[0] + tangent * (feedback.progress * sweep.length)
            error = measured - nominal
            feedback.lateral_error = float(np.dot(error, tangent))
            feedback.normal_error = float(np.linalg.norm(error - tangent * feedback.lateral_error))

            feedback.lateral_speed = float(self._lateral_speed)

        goal_handle.publish_feedback(feedback)

    # ------------------------------------------------------------------
    # Diagnostics (ARM_SWEEP_PLAN §7.B)
    # ------------------------------------------------------------------
    def _open_csv(self, request):
        """Start a per-sweep CSV, if a directory is configured."""
        directory = str(self.get_parameter("diagnostics_csv_dir").value).strip()
        self._close_csv()
        if not directory:
            return
        try:
            os.makedirs(directory, exist_ok=True)
            name = (f"sweep_p{request.partition_index:03d}_"
                    f"{datetime.now():%Y%m%d_%H%M%S}.csv")
            path = os.path.join(directory, name)
            self._csv_file = open(path, "w", newline="")
            self._csv = csv.writer(self._csv_file)
            self._csv.writerow(self.CSV_COLUMNS)
            self.get_logger().info(f"Recording sweep diagnostics to {path}")
        except OSError as e:
            # A failed log must never fail a scan.
            self.get_logger().warn(f"Could not open the diagnostics CSV: {e}")
            self._csv, self._csv_file = None, None

    def _close_csv(self):
        if self._csv_file is not None:
            try:
                self._csv_file.close()
            except OSError:
                pass
        self._csv, self._csv_file = None, None

    CSV_COLUMNS = (
        "t", "phase", "progress", "lateral_travelled",
        "lateral_speed", "lateral_speed_desired", "lateral_error", "normal_error",
        "desired_x", "desired_y", "desired_z", "measured_x", "measured_y", "measured_z",
        "wall_tilt_deg", "orientation_error_deg", "wall_fit_valid",
        "plate_wall_distance", "n_sensors_used",
        "d0", "d1", "d2", "d3", "d4", "d5",
        "press_force", "fx", "fy", "fz", "replans",
    )

    def _publish_diagnostics(self, request, sweep, phase):
        """One SweepDiagnostics sample, and a CSV row when recording.

        Everything is best-effort: a missing sensor, a missing wrench or a closed
        file must never interrupt a sweep, so each channel is guarded and reports
        its own validity flag rather than throwing.
        """
        if not bool(self.get_parameter("diagnostics_enabled").value):
            return

        msg = SweepDiagnostics()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.partition_index = int(request.partition_index)
        msg.partition_count = int(request.partition_count)
        msg.line_z = float(request.start.z)
        msg.phase = phase
        msg.replans = int(self._replans)
        msg.lateral_speed_desired = float(sweep.speed)

        tangent = sweep.tcp[-1] - sweep.tcp[0]
        tangent = tangent / np.linalg.norm(tangent)
        measured = self._measured_tcp()
        if measured is not None:
            progress = self._progress(sweep)
            nominal = sweep.tcp[0] + tangent * (progress * sweep.length)
            error = measured - nominal
            msg.progress = progress
            msg.lateral_travelled = float(np.dot(measured - sweep.tcp[0], tangent))
            msg.lateral_error = float(np.dot(error, tangent))
            msg.normal_error = float(np.linalg.norm(error - tangent * msg.lateral_error))
            for point, target in ((nominal, msg.desired_position),
                                  (measured, msg.measured_position)):
                target.x, target.y, target.z = (float(point[0]), float(point[1]),
                                                float(point[2]))
            # Single producer: the feedback reads this back rather than
            # recomputing it. Both consuming _last_sample meant whichever ran
            # first ate the delta and the other always saw zero.
            previous, now = self._last_sample, time.time()
            if previous is not None and now > previous[0]:
                self._lateral_speed = float(
                    np.dot(measured - previous[1], tangent) / (now - previous[0])
                )
            self._last_sample = (now, measured)
            msg.lateral_speed = float(self._lateral_speed)

            # Commanded vs achieved plate orientation. RX/RY are not compliant, so
            # this is the plate drifting off the trajectory, NOT off the wall.
            q = self.current_joints()
            if q is not None:
                achieved = fk(q)[:3, :3]
                relative = achieved.T @ sweep.rotation
                cos = (np.trace(relative) - 1.0) / 2.0
                msg.orientation_error_deg = float(
                    np.rad2deg(np.arccos(np.clip(cos, -1.0, 1.0)))
                )

        # Wall geometry, read-only. This is the channel S6 will trigger on, which
        # is why it is recorded now: §9 says its threshold has to come from data,
        # not from the 2.0 deg the plan guesses.
        distances = self.fresh_distances()
        if distances is not None:
            msg.distances = distances.tolist()
            used = usable_mask(distances)
            msg.distance_used = [bool(v) for v in used]
            msg.n_sensors_used = int(used.sum())
        alignment = self.wall_alignment()
        if alignment is not None:
            msg.wall_fit_valid = True
            msg.wall_tilt_deg = alignment.tilt_deg
            msg.wall_normal_plate = [float(v) for v in alignment.normal]
        distance = self.plate_wall_distance()
        msg.plate_wall_distance = float("nan") if distance is None else distance

        with self._lock:
            wrench = self._wrench
        if wrench is not None:
            msg.ft_wrench = wrench
            msg.ft_valid = True
            msg.press_force = float(wrench.force.z)

        self._diag_pub.publish(msg)
        self._publish_pose(self._desired_pub, msg.desired_position)
        self._publish_pose(self._measured_pub, msg.measured_position)

        if self._csv is not None:
            try:
                self._csv.writerow([
                    f"{self._now() - self._goal_started:.4f}", phase,
                    f"{msg.progress:.5f}", f"{msg.lateral_travelled:.5f}",
                    f"{msg.lateral_speed:.5f}", f"{msg.lateral_speed_desired:.5f}",
                    f"{msg.lateral_error:.5f}", f"{msg.normal_error:.5f}",
                    f"{msg.desired_position.x:.5f}", f"{msg.desired_position.y:.5f}",
                    f"{msg.desired_position.z:.5f}", f"{msg.measured_position.x:.5f}",
                    f"{msg.measured_position.y:.5f}", f"{msg.measured_position.z:.5f}",
                    f"{msg.wall_tilt_deg:.4f}", f"{msg.orientation_error_deg:.4f}",
                    int(msg.wall_fit_valid), f"{msg.plate_wall_distance:.5f}",
                    msg.n_sensors_used,
                    *[f"{v:.5f}" for v in msg.distances],
                    f"{msg.press_force:.4f}", f"{msg.ft_wrench.force.x:.4f}",
                    f"{msg.ft_wrench.force.y:.4f}", f"{msg.ft_wrench.force.z:.4f}",
                    msg.replans,
                ])
            except (OSError, ValueError) as e:
                self.get_logger().warn(f"Diagnostics CSV write failed: {e}")
                self._close_csv()

    def _publish_pose(self, publisher, point):
        """Republish one diagnostics Point as a PoseStamped, for RViz."""
        msg = PoseStamped()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position = point
        msg.pose.orientation.w = 1.0
        publisher.publish(msg)

    def _publish_planned_path(self, sweep):
        """The planned sweep as an RViz Path, published before anything moves.

        Catches a bad tangent or a mis-transformed partition without waiting for
        the arm to travel it (§11.4).
        """
        path = Path()
        path.header.frame_id = self.base_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for point in sweep.tcp:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
                float(point[0]), float(point[1]), float(point[2])
            )
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._path_pub.publish(path)

    # ------------------------------------------------------------------
    def _spin_until(self, future, timeout):
        """Wait on a future from inside an action callback.

        The executor is multi-threaded, so the future is completed by another
        thread; blocking on it here is safe and keeps ``_execute`` readable as a
        straight-line sequence.
        """
        deadline = time.time() + timeout
        while rclpy.ok() and not future.done() and time.time() < deadline:
            time.sleep(0.01)
        return future.result() if future.done() else None


def main(args=None):
    rclpy.init(args=args)
    node = WallSweepExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # SIGINT (how the FSM stops this node) already shuts the context down via
        # rclpy's signal handler; a second shutdown() raises RCLError -> exit 1.
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
