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

        controller_name = self.get_parameter("controller_name").value
        self.joints = self.get_parameter("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value

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

        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server already available.")
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
        if not self.starting_point_ok:
            self.get_logger().warn("Received trajectory but robot not in valid starting configuration.")
            return
        self.planned_trajectory = msg
        self.trajectory_received = True
        self.get_logger().info("Trajectory received and stored.")

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

        if self.trajectory_received and self.starting_point_ok:
            self.send_trajectory_goal()
            self.trajectory_received = False
            self.execution_complete = False

    def send_trajectory_goal(self):
        goal_msg = FollowJointTrajectory.Goal()

        trajectory = deepcopy(self.planned_trajectory)

        # Each waypoint gets zero velocities.  The joint_trajectory_controller then
        # uses cubic Hermite interpolation and generates a smooth bell-shaped velocity
        # profile (accelerate from 0 → decelerate to 0) for every segment.  This is
        # the only reliable way to avoid PATH_TOLERANCE_VIOLATED in Gazebo:
        # finite-difference velocities at intermediate points cause the cubic spline
        # to overshoot and push the actual joint position outside the tolerance window.
        #
        # Timing: each segment is sized so that the peak joint velocity during the
        # cubic segment stays at or below max_joint_speed.  For a zero-velocity
        # cubic, peak velocity ≈ 1.5 * (delta / T), so T = 1.5 * delta / max_joint_speed.
        max_joint_speed  = 0.5   # rad/s — intentionally slow for smooth, safe motion
        min_segment_time = 0.5   # seconds — floor for very small moves

        n = len(trajectory.points)

        # --- Pass 1: compute per-segment durations ---
        segment_durations = []
        for i, point in enumerate(trajectory.points):
            if i == 0:
                segment_durations.append(min_segment_time)
            else:
                prev = trajectory.points[i - 1]
                max_delta = max(
                    abs(p2 - p1) for p1, p2 in zip(prev.positions, point.positions)
                )
                # Factor of 1.5: peak velocity of a zero-velocity cubic spline
                seg_time = max(min_segment_time, 1.5 * max_delta / max_joint_speed)
                segment_durations.append(seg_time)

        self.get_logger().info(
            f"Trajectory: {n} waypoints, segment times: "
            f"{[f'{t:.2f}' for t in segment_durations]} s, "
            f"total: {sum(segment_durations):.2f} s"
        )

        # --- Pass 2: assign timestamps; zero velocity at every waypoint ---
        accumulated_time = 0.0
        for i, point in enumerate(trajectory.points):
            accumulated_time += segment_durations[i]
            secs = int(accumulated_time)
            nsecs = int((accumulated_time - secs) * 1e9)
            point.time_from_start = Duration(sec=secs, nanosec=nsecs)
            point.velocities = [0.0] * len(point.positions)
            point.accelerations = [0.0] * len(point.positions)

        goal_msg.trajectory = trajectory
        goal_msg.goal_time_tolerance = Duration(sec=2, nanosec=0)
        goal_msg.goal_tolerance = [
            JointTolerance(position=0.01, velocity=0.05, name=name) for name in self.joints
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