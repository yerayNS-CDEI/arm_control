#!/usr/bin/env python3
"""
Robust wall-parallel controller.

Keeps the sensor plate parallel to the wall using all six distance sensors
(3 ultrasonic + 3 ToF) via a weighted, robust least-squares plane fit instead
of a single 3-point triangle. Designed to stay parallel *without shaking*:

  read 6 ranges -> validity filter -> weighted robust plane fit (IRLS/Huber)
  -> EMA filter on the wall normal -> deadband -> low-gain P with per-cycle
  slew limit -> orientation command -> closed-form IK -> JointTrajectory.

Sensor array convention (matches arduino_sensors[_sim].py 'distance_sensors'):
  data = [C, A, B, D, E, F]  (metres)
  ultrasonic: C(0, .15)  A(-.15,-.15)  B(.15,-.15)
  ToF:        D(.15,.15)  E(-.15,.15)   F(0,-.15)
Range is measured along the plate +Z axis (toward the wall).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial.transform import Rotation as R
import tf2_ros

from planner.planner_lib.closed_form_algorithm import closed_form_algorithm
from sensors.wall_alignment_estimator import (
    SENSOR_NAMES,
    SENSOR_XY,
    WallAlignmentEstimator,
    correction_angles,
    usable_mask,
    valid_mask,
)


class WallParallelController(Node):

    def __init__(self):
        super().__init__('wall_parallel_controller')

        # --- Tunables -------------------------------------------------------
        self.declare_parameter('control_rate', 30.0)        # Hz (high for smooth direct streaming)
        self.declare_parameter('ideal_distance', 0.50)      # m (standoff target)
        self.declare_parameter('compute_distance_correction', False)  # Z left to force_mode
        self.declare_parameter('kp', 0.7)                   # proportional gain on tilt error
        self.declare_parameter('ema_alpha', 0.3)            # normal low-pass (0..1, lower=smoother)
        self.declare_parameter('deadband_deg', 1.0)         # below this tilt -> no correction
        self.declare_parameter('max_step_deg', 3.0)         # per-cycle slew limit on each angle
        self.declare_parameter('huber_k', 1.5)              # robust threshold (in robust sigmas)
        # Goal horizon. Keep close to the control period so each streamed goal executes
        # almost fully before the next replaces it (long horizons make the arm crawl
        # through only the ease-in of each mini-trajectory).
        self.declare_parameter('traj_time', 0.3)            # s, JointTrajectory point time
        # Output mode (pick to match the active ros2_control controller):
        #  'position'    -> Float64MultiArray of raw joint positions to the
        #                   forward_position_controller. Use in Gazebo simulation
        #                   (no trajectory ease-in -> immediate, responsive tracking).
        #  'passthrough' -> JointTrajectory to the 'planned_trajectory' topic. The
        #                   publisher_joint_trajectory_planned bridge node forwards it
        #                   as a FollowJointTrajectory goal to the
        #                   passthrough_trajectory_controller (UR onboard executor,
        #                   respects speed scaling). Use on the REAL robot. Mirrors how
        #                   align_ee_to_wall.py drives the PTC, so the bridge node must
        #                   be running (its 'controller_name' set to
        #                   passthrough_trajectory_controller).
        self.declare_parameter('output_mode', 'passthrough')
        self.declare_parameter('position_command_topic', '/forward_position_controller/commands')
        self.declare_parameter('trajectory_topic', 'planned_trajectory')
        # The current pose is base_frame->tool0 (the plate TCP) from TF, but the analytic IK
        # solves for its own DH end-frame, which differs from tool0 by a fixed rigid
        # transform (a 0.3 m offset + a 120 deg rotation here). We calibrate that constant
        # offset once at startup from FK(current joints) vs the live tool0 TF, so we never
        # have to guess flange/tool0/DH conventions.
        self.declare_parameter('tool0_frame', 'arm_tool0')
        self.declare_parameter('base_frame', 'arm_base')   # IK base frame (UR DH base)

        self.control_rate = self.get_parameter('control_rate').value
        self.ideal_distance = self.get_parameter('ideal_distance').value
        self.compute_distance_correction = self.get_parameter('compute_distance_correction').value
        self.kp = self.get_parameter('kp').value
        self.ema_alpha = self.get_parameter('ema_alpha').value
        self.deadband = np.deg2rad(self.get_parameter('deadband_deg').value)
        self.max_step = np.deg2rad(self.get_parameter('max_step_deg').value)
        self.huber_k = self.get_parameter('huber_k').value
        self.traj_time = self.get_parameter('traj_time').value
        self.output_mode = self.get_parameter('output_mode').value
        self.position_command_topic = self.get_parameter('position_command_topic').value
        self.trajectory_topic = self.get_parameter('trajectory_topic').value
        if self.output_mode not in ('position', 'passthrough'):
            raise ValueError(
                f"output_mode must be 'position' (sim/FPC) or 'passthrough' (real/PTC), "
                f"got '{self.output_mode}'")
        self.tool0_frame = self.get_parameter('tool0_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # --- Sensor geometry, noise model and the plane fit -----------------
        # All of it lives in sensors/wall_alignment_estimator, because
        # wall_sweep_executor needs the identical numbers WITHOUT commanding the
        # arm (ARM_SWEEP_PLAN §4.2). Two copies of this geometry would drift, and
        # the two nodes would then disagree about where the wall is.
        self.estimator = WallAlignmentEstimator(
            ema_alpha=self.ema_alpha, huber_k=self.huber_k
        )

        # --- State ----------------------------------------------------------
        self.distances = None
        self.current_joint_state = None
        self.joint_indices = None
        self.T_tool0_dhend = None                  # static tool0 -> IK DH end-frame (calibrated)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.expected_joint_names = [
            'arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint',
            'arm_wrist_1_joint', 'arm_wrist_2_joint', 'arm_wrist_3_joint',
        ]

        # --- ROS I/O --------------------------------------------------------
        if self.output_mode == 'position':
            self.cmd_pub = self.create_publisher(Float64MultiArray, self.position_command_topic, 10)
            out = self.position_command_topic
        else:  # 'passthrough' -> bridge node forwards to passthrough_trajectory_controller
            self.cmd_pub = self.create_publisher(JointTrajectory, self.trajectory_topic, 10)
            out = self.trajectory_topic
        self.create_subscription(Float32MultiArray, 'distance_sensors', self.distance_cb, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_cb, 10)

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_step)
        self.get_logger().info(
            f"WallParallelController up: mode={self.output_mode} "
            f"rate={self.control_rate}Hz kp={self.kp} ema={self.ema_alpha} "
            f"deadband={np.rad2deg(self.deadband):.2f}deg "
            f"max_step={np.rad2deg(self.max_step):.2f}deg -> {out}")

    # --- Callbacks ----------------------------------------------------------
    def distance_cb(self, msg):
        if len(msg.data) == 6:
            self.distances = np.array(msg.data, dtype=float)

    def joint_cb(self, msg):
        if self.joint_indices is None:
            idx = []
            for name in self.expected_joint_names:
                try:
                    idx.append(msg.name.index(name))
                except ValueError:
                    return
            self.joint_indices = idx
        self.current_joint_state = msg

    # UR10e DH parameters (must match planner_lib.closed_form_algorithm)
    _DH_D = (0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655)
    _DH_A = (0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0)
    _DH_ALPHA = (np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0)

    def fk(self, q):
        """Forward kinematics in the IK's own DH convention: arm_base -> DH end-frame."""
        T = np.eye(4)
        for i in range(6):
            ct, st = np.cos(q[i]), np.sin(q[i])
            ca, sa = np.cos(self._DH_ALPHA[i]), np.sin(self._DH_ALPHA[i])
            a, d = self._DH_A[i], self._DH_D[i]
            Ti = np.array([[ct, -st * ca,  st * sa, a * ct],
                           [st,  ct * ca, -ct * sa, a * st],
                           [0.0,      sa,      ca,      d],
                           [0.0,     0.0,     0.0,    1.0]])
            T = T @ Ti
        return T

    def calibrate_tool0_dhend(self, base_frame, q_current):
        """Compute the constant tool0 -> DH-end transform from FK(q) vs the live tool0 TF.
        Sidesteps all flange/tool0/DH-convention differences. Cached after first success."""
        if self.T_tool0_dhend is not None:
            return self.T_tool0_dhend
        try:
            tr = self.tf_buffer.lookup_transform(base_frame, self.tool0_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tr.transform.translation
        q = tr.transform.rotation
        T_base_tool0 = np.eye(4)
        T_base_tool0[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T_base_tool0[:3, 3] = [t.x, t.y, t.z]

        T_base_dhend = self.fk(np.asarray(q_current))
        self.T_tool0_dhend = np.linalg.inv(T_base_tool0) @ T_base_dhend
        self.get_logger().info(
            f"Calibrated {self.tool0_frame}->DH-end "
            f"(trans={np.round(self.T_tool0_dhend[:3,3],4).tolist()})")
        return self.T_tool0_dhend

    def get_current_tool0(self):
        """Fresh current tool0 pose (rotation, position) in base_frame from TF, or None.
        Looked up every control cycle so the loop closes on the real orientation, not a
        stale 1 Hz pose topic (which makes the correction stall)."""
        try:
            tr = self.tf_buffer.lookup_transform(
                self.base_frame, self.tool0_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tr.transform.translation
        q = tr.transform.rotation
        return R.from_quat([q.x, q.y, q.z, q.w]), np.array([t.x, t.y, t.z])

    # --- Control ------------------------------------------------------------
    def control_step(self):
        if self.distances is None:
            # Loud on purpose: a starved controller silently leaves the plate in
            # whatever pose pre-approach parked it (e.g. sensors out of range ->
            # arduino_sensors never publishes the combined array).
            self.get_logger().warn(
                "No distance_sensors data — plate is NOT aligning to the wall.",
                throttle_duration_sec=5.0)
            return
        if self.current_joint_state is None or self.joint_indices is None:
            self.get_logger().warn("Waiting for joint_states...", throttle_duration_sec=2.0)
            return

        cur = self.get_current_tool0()
        if cur is None:
            self.get_logger().warn(
                f"Waiting for {self.base_frame}->{self.tool0_frame} TF...",
                throttle_duration_sec=2.0)
            return
        curr_rot, p_current = cur

        # Per-sensor diagnostic table: name, plate (x,y), distance, valid?
        # Tilt the plate a known way on the bench and watch which sensor reads
        # closest -> confirms the layout is correct (the near corner's sensor
        # must be the one whose (x,y) is on that side).
        d = self.distances
        # (x) = outside the sensor's own validity window.
        # (p) = in range, but not on the same plane as the others, so the fit
        #       excluded it -- e.g. an ultrasonic seeing past the edge of the
        #       wall. Distinguished because the two mean different things on the
        #       bench: (x) is a sensor problem, (p) is a geometry problem.
        in_window = valid_mask(d)
        used = usable_mask(d, max_residual=self.estimator.max_residual)
        table = "  ".join(
            f"{SENSOR_NAMES[i]}@({SENSOR_XY[i,0]:+.2f},{SENSOR_XY[i,1]:+.2f})="
            f"{d[i]*100:5.1f}cm"
            f"{'' if used[i] else ('(x)' if not in_window[i] else '(p)')}"
            for i in range(6))
        self.get_logger().info(table, throttle_duration_sec=1.0)

        # Shared estimator: plane fit + EMA on the fitted normal (not on raw
        # ranges -- filtering ranges individually smears the plane when only some
        # sensors are noisy).
        est = self.estimator.update(d)
        if est is None:
            self.get_logger().warn(
                f"No usable plane fit ({int(in_window.sum())}/6 in range, "
                f"{int(used.sum())} on a common plane) — holding orientation.",
                throttle_duration_sec=2.0)
            return
        nwf, mean_d, n_valid, tilt = est.normal, est.distance, est.n_valid, est.tilt

        if tilt <= self.deadband:
            # Parallel enough: hold orientation, don't chase noise (anti-shake).
            goal_rot = curr_rot
        else:
            # Pitch/yaw error to null the wall normal, in EE frame.
            # Low gain + per-cycle slew limit -> smooth, non-oscillatory.
            beta, gamma = correction_angles(nwf, kp=self.kp, max_step=self.max_step)

            inc = R.from_euler('ZYX', [0.0, beta, gamma], degrees=False)
            goal_rot = curr_rot * inc

            # Roll compensation: keep plate Y aligned with vertical (as in align node)
            z_ee = goal_rot.apply([0, 0, 1])
            y_desired = np.array([0.0, 0.0, 1.0])
            y_proj = y_desired - np.dot(y_desired, z_ee) * z_ee
            if np.linalg.norm(y_proj) > 1e-6:
                y_proj /= np.linalg.norm(y_proj)
                x_ee = np.cross(y_proj, z_ee); x_ee /= np.linalg.norm(x_ee)
                y_ee = np.cross(z_ee, x_ee)
                goal_rot = R.from_matrix(np.column_stack((x_ee, y_ee, z_ee)))

        q = goal_rot.as_quat()

        # Position: optional standoff correction (off by default; Z owned by force_mode)
        p_new = p_current.copy()
        if self.compute_distance_correction:
            nw_global = curr_rot.apply(nwf)
            p_new = p_new + (mean_d - self.ideal_distance) * nw_global

        q_current = np.array([self.current_joint_state.position[i] for i in self.joint_indices])

        # Build the desired tool0 (plate TCP) pose, then convert to the IK's DH end-frame:
        # T_base_dhend = T_base_tool0 * T_tool0_dhend  (calibrated constant offset).
        T_tool0_dhend = self.calibrate_tool0_dhend(self.base_frame, q_current)
        if T_tool0_dhend is None:
            self.get_logger().warn(
                f"Waiting for {self.base_frame}->{self.tool0_frame} TF to calibrate...",
                throttle_duration_sec=2.0)
            return
        T_base_tool0 = np.eye(4)
        T_base_tool0[:3, :3] = R.from_quat(q).as_matrix()
        T_base_tool0[:3, 3] = p_new
        T = T_base_tool0 @ T_tool0_dhend

        joint_values = closed_form_algorithm(T, q_current, type=0)
        if np.any(np.isnan(joint_values)):
            self.get_logger().error("IK solution contains NaN. Skipping.")
            return

        if self.output_mode == 'position':
            # Direct joint-position setpoint -> forward_position_controller (no ease-in)
            self.cmd_pub.publish(Float64MultiArray(data=joint_values.tolist()))
        else:
            # JointTrajectory -> 'planned_trajectory'; the bridge node re-times it and
            # sends it to the passthrough_trajectory_controller action. (The bridge
            # recomputes time_from_start, so traj_time here is only a placeholder.)
            traj = JointTrajectory()
            traj.joint_names = self.expected_joint_names
            pt = JointTrajectoryPoint()
            pt.positions = joint_values.tolist()
            pt.time_from_start.sec = int(self.traj_time)
            pt.time_from_start.nanosec = int((self.traj_time % 1.0) * 1e9)
            traj.points.append(pt)
            self.cmd_pub.publish(traj)

        self.get_logger().info(
            f"valid={n_valid}/6 tilt={np.rad2deg(tilt):.2f}deg "
            f"dist={mean_d*100:.1f}cm nw=[{nwf[0]:+.3f},{nwf[1]:+.3f},{nwf[2]:+.3f}]",
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = WallParallelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # SIGINT (how the FSM stops this node) already shuts the context down via
        # rclpy's signal handler; a second shutdown() raises RCLError -> exit 1.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
