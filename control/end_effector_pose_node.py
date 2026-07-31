#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_ros
from scipy.spatial.transform import Rotation as R

class EndEffectorListener(Node):
    def __init__(self):
        super().__init__('end_effector_pose_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.frame_candidates = [
            ('arm_base', 'arm_tool0'),
            ('arm_base', 'arm_tool0_controller'),
            ('arm_base_link', 'arm_tool0'),
            ('arm_base_link', 'arm_tool0_controller'),
            ('base', 'tool0'),
            ('base_link', 'tool0'),
        ]

        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz
        self.publisher_ = self.create_publisher(PoseStamped, 'end_effector_pose', 10)

    def timer_callback(self):
        try:
            trans = None
            selected_frames = None
            for base_frame, tool_frame in self.frame_candidates:
                try:
                    trans = self.tf_buffer.lookup_transform(
                        base_frame, tool_frame, rclpy.time.Time())
                    selected_frames = (base_frame, tool_frame)
                    break
                except Exception:
                    continue

            if trans is None:
                raise RuntimeError(
                    "Unable to find transform for any candidate frame pair: "
                    f"{self.frame_candidates}"
                )
            
            pos = trans.transform.translation
            rot = trans.transform.rotation

            # self.get_logger().info(f"End Effector Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}")
            # self.get_logger().info(f"Orientation (quat): x={rot.x:.3f}, y={rot.y:.3f}, z={rot.z:.3f}, w={rot.w:.3f}")

            # Convert quaternion to RPY
            euler = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_euler('xyz')
            # self.get_logger().info(f"Orientation (rpy): roll={euler[0]:.2f}, pitch={euler[1]:.2f}, yaw={euler[2]:.2f}")

            # Convert original orientation to rotation matrix
            original_rot = R.from_quat([rot.x, rot.y, rot.z, rot.w])

            # Define additional rotation of 45 degrees (pi/4 rad) around Z
            rotation_z = R.from_euler('z', 0, degrees=True)

            # Compose both rotations
            new_rot = original_rot * rotation_z

            # Convert back to quaternion
            new_quat = new_rot.as_quat()  # Returns [x, y, z, w]

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = selected_frames[0]
            pose_msg.pose.position.x = pos.x
            pose_msg.pose.position.y = pos.y
            pose_msg.pose.position.z = pos.z
            pose_msg.pose.orientation.x = new_quat[0]
            pose_msg.pose.orientation.y = new_quat[1]
            pose_msg.pose.orientation.z = new_quat[2]
            pose_msg.pose.orientation.w = new_quat[3]


            self.publisher_.publish(pose_msg)

        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
