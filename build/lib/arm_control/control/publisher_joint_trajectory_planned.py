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

class PublisherJointTrajectoryActionClient(Node):

    def __init__(self):
        super().__init__("publisher_joint_trajectory_action_client")

        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter("joints", ["shoulder_pan_joint", "shoulder_lift_joint",
                                          "elbow_joint", "wrist_1_joint",
                                          "wrist_2_joint", "wrist_3_joint"])
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
                
        self.joint_state_sub = self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.trajectory_sub = self.create_subscription(JointTrajectory, "planned_trajectory", self.trajectory_callback, 10)
        self.status_pub = self.create_publisher(Bool, "execution_status", 10)
        self.emergency_sub = self.create_subscription(Bool, "emergency_stop", self.emergency_callback, 10)

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
        duration_between_points = 1.0

        for i, point in enumerate(trajectory.points):
            total_sec = duration_between_points * (i + 1)
            secs = int(total_sec)
            nsecs = int((total_sec - secs) * 1e9)
            point.time_from_start = Duration(sec=secs, nanosec=nsecs)
            # if not point.velocities or len(point.velocities) != len(point.positions):
            #     point.velocities = [0.0] * len(point.positions)
            if i > 0:
                prev = trajectory.points[i - 1]
                dt = duration_between_points
                point.velocities = [
                    (p2 - p1) / dt for p1, p2 in zip(prev.positions, point.positions)
                ]
            else:
                point.velocities = [0.0] * len(point.positions)

        goal_msg.trajectory = trajectory
        goal_msg.goal_time_tolerance = Duration(sec=0, nanosec=200_000_000)
        goal_msg.goal_tolerance = [
            JointTolerance(position=0.01, velocity=0.01, name=name) for name in self.joints
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