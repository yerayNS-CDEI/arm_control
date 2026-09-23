#!/usr/bin/env python3

########################################################

#### Node using an action client to send goals

########################################################

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from copy import deepcopy
from math import copysign
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class PublisherJointTrajectoryActionClient(Node):

    def __init__(self):
        super().__init__("publisher_joint_trajectory_action_client")

        self.declare_parameter("controller_name", "joint_trajectory_controller")
        self.declare_parameter("joints", ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint",
                                          "arm_elbow_joint", "arm_wrist_1_joint",
                                          "arm_wrist_2_joint", "arm_wrist_3_joint"])
        self.declare_parameter("check_starting_point", False)

        # --- Motion-profile parameters -------------------------------------
        # The arm behaves differently in Gazebo and on the real UR10e, so the
        # timing gains are declared twice and the active pair is selected from
        # "sim".  arm.launch.py sets it to true only for pure Gazebo; the real
        # robot and hybrid simulation (URSim through the passthrough
        # controller) both use the "_real" values.
        self.declare_parameter("sim", False)
        self.declare_parameter("max_joint_speed_sim", 0.9)    # rad/s
        self.declare_parameter("max_joint_speed_real", 0.5)   # rad/s
        self.declare_parameter("min_segment_time_sim", 0.1)   # s
        self.declare_parameter("min_segment_time_real", 0.4)  # s
        # Blend through intermediate waypoints instead of stopping at each one.
        self.declare_parameter("blend_waypoint_velocities", True)

        controller_name = self.get_parameter("controller_name").value
        self.joints = self.get_parameter("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value
        self.sim = bool(self.get_parameter("sim").value)

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is required')
        
        self.starting_point = {}
        if self.check_starting_point:
            for name in self.joints:
                param = "starting_point_limits." + name
                self.declare_parameter(param, [-2 * 3.14159, 2 * 3.14159])
                self.starting_point[name] = self.get_parameter(param).value

            for name in self.joints:
                if len(self.starting_point[name]) != 2:
                    raise Exception('"starting_point" parameter is not set correctly!')
                
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
                
        self.joint_state_sub = self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.trajectory_sub = self.create_subscription(JointTrajectory, "planned_trajectory", self.trajectory_callback, 10)
        self.status_pub = self.create_publisher(Bool, "execution_status", 10)
        self.emergency_sub = self.create_subscription(Bool, "emergency_stop", self.emergency_callback, qos)
        # Hand the arm over to another node without stopping this one.
        #
        # While held, this bridge cancels whatever it is running and dispatches
        # nothing. wall_sweep_executor drives the trajectory controller directly
        # during a wall sweep, and two publishers on one controller is not
        # survivable: each new goal preempts the other's, so the sweep comes back
        # CANCELED. Killing wall_parallel_controller is not enough -- its last
        # trajectory is still sitting here, and this node re-dispatches it the
        # moment a slot opens.
        #
        # Transient-local so a late-starting executor's hold is not missed.
        self.hold_sub = self.create_subscription(Bool, "trajectory_bridge/hold", self.hold_callback, qos)

        self.emergency_srv = self.create_service(Trigger, "emergency_stop", self.handle_emergency_service)

        action_topic = f"{controller_name}/follow_joint_trajectory"
        self._action_client = ActionClient(self, FollowJointTrajectory, action_topic)

        self.current_joint_state = None
        self.starting_point_ok = not self.check_starting_point
        self.planned_trajectory = None
        self.trajectory_received = False
        self.execution_complete = True
        self.current_goal_handle = None
        self.prev_status = True
        self.held = False

        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server already available.")
        self.get_logger().info("\033[1;32mEmergency stop service: ros2 service call /emergency_stop std_srvs/srv/Trigger\033[0m")
        self.timer = self.create_timer(0.1, self.timer_callback)

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

        if self.check_starting_point:
            limit_exceeded = False
            for idx, name in enumerate(msg.name):
                if name in self.starting_point:
                    pos = msg.position[idx]
                    low, high = self.starting_point[name]
                    if not (low <= pos <= high):
                        self.get_logger().warn(f"Joint {name} position {pos:.3f} out of limits {low:.3f}, {high:.3f}")
                        limit_exceeded = True
            self.starting_point_ok = not limit_exceeded
            self.check_starting_point = False       # to just check once at the start

    def trajectory_callback(self, msg):
        if self.held:
            # Dropped, not queued: another node owns the arm, and a trajectory
            # planned against a pose it has since moved away from is worse than no
            # trajectory at all.
            return
        if not self.starting_point_ok:
            self.get_logger().warn("Received trajectory but robot not in valid starting configuration.")
            return
        self.planned_trajectory = msg
        self.trajectory_received = True
        self.get_logger().info("Trajectory received and stored.")

    def hold_callback(self, msg):
        """Stand down (or resume) so another node can command the arm."""
        held = bool(msg.data)
        if held == self.held:
            return
        self.held = held
        if not held:
            self.get_logger().info("Trajectory bridge released; dispatching again.")
            return

        self.get_logger().info(
            "Trajectory bridge held: another node is driving the arm. Cancelling "
            "any active goal and dropping the pending trajectory."
        )
        # Drop the pending trajectory too. Without this, releasing the hold would
        # fire whatever wall_parallel_controller happened to publish last -- a
        # target from before the sweep, now stale by a whole partition.
        self.trajectory_received = False
        self.planned_trajectory = None
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.execution_complete = True

    def emergency_callback(self, msg):
        if msg.data and self.current_goal_handle:
            self.get_logger().warn("Emergency stop received! Cancelling active trajectory...")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info("Goal cancel requested."))
            self.execution_complete = True

    def handle_emergency_service(self, request, response):
        if self.current_goal_handle:
            self.get_logger().warn("Emergency stop requested via service! Cancelling trajectory...")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info("Goal cancel requested."))
            self.execution_complete = True
            response.success = True
            response.message = "Trajectory cancelled successfully."
        else:
            response.success = False
            response.message = "No active trajectory to cancel."
        return response

    def timer_callback(self):
        status_msg = Bool()
        status_msg.data = self.execution_complete
        if self.prev_status != self.execution_complete:
            self.status_pub.publish(status_msg)
            self.prev_status = self.execution_complete

        # Only dispatch a goal when the previous one has finished. The
        # passthrough_trajectory_controller executes a trajectory atomically and
        # rejects any goal received mid-execution ("A trajectory is already
        # executing."). Streaming nodes (e.g. wall_parallel_controller) publish a
        # fresh trajectory many times per second; trajectory_callback keeps only the
        # latest, so when a slot opens we always send the freshest target instead of
        # spamming the controller with goals that just get rejected.
        if self.held:
            return
        if self.trajectory_received and self.starting_point_ok and self.execution_complete:
            self.send_trajectory_goal()
            self.trajectory_received = False
            self.execution_complete = False

    def _current_positions_for_joint_order(self, joint_order):
        """Return current joint positions aligned with the provided joint-name order."""
        if self.current_joint_state is None or not joint_order:
            return None

        index_by_name = {name: idx for idx, name in enumerate(self.current_joint_state.name)}
        missing = [name for name in joint_order if name not in index_by_name]
        if missing:
            self.get_logger().warn(
                f"Current joint_state is missing joints {missing}; "
                "falling back to minimum first-segment time."
            )
            return None

        return [self.current_joint_state.position[index_by_name[name]] for name in joint_order]

    def _motion_profile(self):
        """Return (max_joint_speed, min_segment_time) for the active sim/real profile.

        Both pairs are read fresh on every goal so they can be retuned live with
        ``ros2 param set`` without restarting the node.
        """
        suffix = "_sim" if self.sim else "_real"
        return (
            float(self.get_parameter("max_joint_speed" + suffix).value),
            float(self.get_parameter("min_segment_time" + suffix).value),
        )

    @staticmethod
    def _blend_velocities(waypoints, segment_durations, start_positions, max_joint_speed):
        """Per-waypoint joint velocities that let the arm pass *through* waypoints.

        ``waypoints`` are the joint positions of the trajectory points and
        ``segment_durations[i]`` is the time allotted to reach ``waypoints[i]``.
        ``start_positions`` is where the arm actually is when execution begins
        (the implicit point at t=0), or None when it is unknown.

        The velocity at an interior waypoint is the *minmod* of the incoming and
        outgoing chord slopes: zero whenever the joint reverses direction (it has
        to stop there anyway) and otherwise the smaller of the two slopes, signed.
        Bounding the tangent by the smaller adjacent slope is exactly the
        condition that keeps a cubic Hermite segment monotone, so the spline can
        no longer overshoot past a waypoint - which is what made plain
        finite-difference velocities trip PATH_TOLERANCE_VIOLATED in Gazebo.
        The last waypoint keeps zero velocity so the arm comes to rest on the goal.
        """
        n = len(waypoints)
        n_joints = len(waypoints[0])

        # Node list including the implicit start point: nodes[k] is reached at the
        # end of segment_durations[k - 1], so nodes[k + 1] == waypoints[k].
        if start_positions is not None and len(start_positions) != n_joints:
            start_positions = None
        nodes = [list(start_positions) if start_positions is not None else None]
        nodes.extend(list(p) for p in waypoints)

        velocities = []
        for k in range(n):
            if k == n - 1 or nodes[k] is None:
                # Final waypoint rests on the goal; without a known start position
                # the first waypoint has no incoming slope, so keep it at rest.
                velocities.append([0.0] * n_joints)
                continue

            dt_in = segment_durations[k]
            dt_out = segment_durations[k + 1]
            if dt_in <= 0.0 or dt_out <= 0.0:
                velocities.append([0.0] * n_joints)
                continue

            point_velocities = []
            for j in range(n_joints):
                slope_in = (nodes[k + 1][j] - nodes[k][j]) / dt_in
                slope_out = (nodes[k + 2][j] - nodes[k + 1][j]) / dt_out
                if slope_in * slope_out <= 0.0:
                    point_velocities.append(0.0)
                else:
                    magnitude = min(abs(slope_in), abs(slope_out), max_joint_speed)
                    point_velocities.append(copysign(magnitude, slope_in))
            velocities.append(point_velocities)

        return velocities

    def send_trajectory_goal(self):
        goal_msg = FollowJointTrajectory.Goal()

        trajectory = deepcopy(self.planned_trajectory)
        joint_order = trajectory.joint_names if trajectory.joint_names else self.joints
        current_positions = self._current_positions_for_joint_order(joint_order)

        # Timing: each segment is sized so that the peak joint velocity during the
        # cubic segment stays at or below max_joint_speed.  For a zero-velocity
        # cubic, peak velocity ≈ 1.5 * (delta / T), so T = 1.5 * delta / max_joint_speed.
        # Blended segments peak lower than that, so the factor stays conservative.
        max_joint_speed, min_segment_time = self._motion_profile()
        blend = bool(self.get_parameter("blend_waypoint_velocities").value)

        n = len(trajectory.points)

        # --- Pass 1: compute per-segment durations ---
        segment_durations = []
        for i, point in enumerate(trajectory.points):
            if i == 0:
                # First waypoint: calculate time from CURRENT robot position
                if current_positions is not None and len(current_positions) == len(point.positions):
                    max_delta = max(
                        abs(p2 - p1) for p1, p2 in zip(current_positions, point.positions)
                    )
                    seg_time = max(min_segment_time, 1.5 * max_delta / max_joint_speed)
                    segment_durations.append(seg_time)
                else:
                    segment_durations.append(min_segment_time)
            else:
                prev = trajectory.points[i - 1]
                max_delta = max(
                    abs(p2 - p1) for p1, p2 in zip(prev.positions, point.positions)
                )
                # Factor of 1.5: peak velocity of a zero-velocity cubic spline
                seg_time = max(min_segment_time, 1.5 * max_delta / max_joint_speed)
                segment_durations.append(seg_time)

        # Log current vs first waypoint for debugging
        if current_positions is not None and len(trajectory.points) > 0:
            if len(current_positions) == len(trajectory.points[0].positions):
                self.get_logger().info(f"Joint order used for timing: {joint_order}")
                self.get_logger().info(f"Current joint positions: {[f'{p:.3f}' for p in current_positions]}")
                self.get_logger().info(f"First waypoint positions: {[f'{p:.3f}' for p in trajectory.points[0].positions]}")
                deltas = [abs(p2 - p1) for p1, p2 in zip(current_positions, trajectory.points[0].positions)]
                self.get_logger().info(f"Joint deltas to first waypoint: {[f'{d:.3f}' for d in deltas]}")

        self.get_logger().info(
            f"Trajectory: {n} waypoints, segment times: "
            f"{[f'{t:.2f}' for t in segment_durations]} s, "
            f"total: {sum(segment_durations):.2f} s "
            f"(profile: {'sim' if self.sim else 'real'}, "
            f"max_joint_speed={max_joint_speed:.3f} rad/s, "
            f"min_segment_time={min_segment_time:.3f} s, "
            f"blending={'on' if blend else 'off'})"
        )

        # --- Pass 2: assign timestamps ---
        accumulated_time = 0.0
        for i, point in enumerate(trajectory.points):
            accumulated_time += segment_durations[i]
            secs = int(accumulated_time)
            nsecs = int((accumulated_time - secs) * 1e9)
            point.time_from_start = Duration(sec=secs, nanosec=nsecs)

        # --- Pass 3: waypoint velocities ---
        if blend and n > 0:
            # Non-zero velocities at intermediate waypoints let the controller
            # carry momentum through them instead of decelerating to a stop and
            # accelerating again.  Accelerations are left empty on purpose: with
            # positions + velocities the controller interpolates with a cubic
            # Hermite spline, whereas supplying accelerations too would force a
            # quintic that is pinned to zero acceleration at every waypoint.
            blended = self._blend_velocities(
                [point.positions for point in trajectory.points],
                segment_durations,
                current_positions,
                max_joint_speed,
            )
            for point, point_velocities in zip(trajectory.points, blended):
                point.velocities = point_velocities
                point.accelerations = []
        else:
            # Legacy behaviour: rest at every waypoint.  Each segment becomes an
            # independent accelerate-from-0 / decelerate-to-0 profile.
            for point in trajectory.points:
                point.velocities = [0.0] * len(point.positions)
                point.accelerations = [0.0] * len(point.positions)

        goal_msg.trajectory = trajectory
        goal_msg.goal_time_tolerance = Duration(sec=2, nanosec=0)
        goal_msg.goal_tolerance = [
            JointTolerance(position=0.01, velocity=0.05, name=name) for name in joint_order
        ]

        self.get_logger().info("Sending trajectory goal with added times and velocities...")
        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            # raise RuntimeError("Goal rejected :(")
            self.execution_complete = True
            return

        self.get_logger().info("Goal accepted. Waiting for result...")
        self.current_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Done with result: {self.status_to_str(status)}")
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Trajectory execution succeeded.")
        else:
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(f"Done with result: {self.error_code_to_str(result.error_code)}")
            # raise RuntimeError("Executing trajectory failed. " + result.error_string)   # To avoid node shutdown can be commented out
            self.get_logger().error(f"Executing trajectory failed. {result.error_string}")
        self.execution_complete = True

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"

    @staticmethod
    def status_to_str(error_code):
        if error_code == GoalStatus.STATUS_UNKNOWN:
            return "UNKNOWN"
        if error_code == GoalStatus.STATUS_ACCEPTED:
            return "ACCEPTED"
        if error_code == GoalStatus.STATUS_EXECUTING:
            return "EXECUTING"
        if error_code == GoalStatus.STATUS_CANCELING:
            return "CANCELING"
        if error_code == GoalStatus.STATUS_SUCCEEDED:
            return "SUCCEEDED"
        if error_code == GoalStatus.STATUS_CANCELED:
            return "CANCELED"
        if error_code == GoalStatus.STATUS_ABORTED:
            return "ABORTED"

def main(args=None):
    rclpy.init(args=args)
    node = PublisherJointTrajectoryActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as err:
        node.get_logger().error(str(err))
    except SystemExit:
        rclpy.logging.get_logger("jtc_client").info("Done")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
