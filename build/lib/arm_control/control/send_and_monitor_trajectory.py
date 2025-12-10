import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import threading
import sys
import termios
import tty

class UR10eTrajectoryClient(Node):
    def __init__(self):
        super().__init__('ur10e_trajectory_client')
        self.client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.client.wait_for_server()
        self.get_logger().info('Connected to action server.')

    def send_trajectory(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # safe pose
        point.time_from_start.sec = 3

        goal_msg.trajectory.points = [point]

        self.future_goal = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self.future_goal.add_done_callback(self.goal_response_callback)

        # Thread to listening keyboard for cancelling
        cancel_thread = threading.Thread(target=self.listen_for_cancel)
        cancel_thread.daemon = True
        cancel_thread.start()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Actual positions: {feedback.actual.positions}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self._goal_handle = goal_handle

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info('Trajectory succeeded!')
        else:
            self.get_logger().warn(f'Trajectory failed: {result.error_string}')
        rclpy.shutdown()

    def listen_for_cancel(self):
        print("\n Press 'c' to cancel ongoing trajectory.")
        while True:
            if self.get_char() == 'c':
                self.get_logger().warn('Cancelling trajectory...')
                self._goal_handle.cancel_goal_async()
                return

    def get_char(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def main(args=None):
    rclpy.init(args=args)
    node = UR10eTrajectoryClient()
    node.send_trajectory()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
