import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Pose
import tf2_ros
from scipy.spatial.transform import Rotation as R

class EndEffectorListener(Node):
    def __init__(self):
        super().__init__('end_effector_pose_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.publisher_ = self.create_publisher(Pose, 'end_effector_pose', 10)

    def timer_callback(self):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'arm/base', 'arm/tool0', rclpy.time.Time())     # tool0_controller for real robot, tool0 for simulation     # if no namespace is needed, erase "arm/" in both
            
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

            pose_msg = Pose()
            pose_msg.position.x = pos.x
            pose_msg.position.y = pos.y
            pose_msg.position.z = pos.z
            pose_msg.orientation.x = new_quat[0]
            pose_msg.orientation.y = new_quat[1]
            pose_msg.orientation.z = new_quat[2]
            pose_msg.orientation.w = new_quat[3]


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
