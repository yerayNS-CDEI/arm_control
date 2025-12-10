import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from scipy.spatial.transform import Rotation as R
from robotic_arm_planner.planner_lib.closed_form_algorithm import closed_form_algorithm

class SensorsOrientation(Node):

    def __init__(self):
        super().__init__('sensors_orientation')

        # Node Variables
        self.end_effector_pose = None
        self.ideal_distance = 15.0  # cm
        self.toggle = 1
        # 3 sensors: A left, B right, C top 
        # Positions on the plate 
        # Define the positions of the sensors
        # xA, yA = -15, -15 # Position of sensor A 
        # xB, yB = 15, -15 # Position of sensor B 
        # xC, yC = 0.0, 15 # Position of sensor C 
        xA, yA = 15, 15 # Position of sensor A 
        xB, yB = -15, 15 # Position of sensor B 
        xC, yC = 0.0, -15 # Position of sensor C 
        self.pA= np.array([xA, yA, 0.0])
        self.pB= np.array([xB, yB, 0.0])
        self.pC= np.array([xC, yC, 0.0])
        self.dA = 0
        self.dB = 0
        self.dC = 0

        self.last_x_ee = None

        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/planned_trajectory', 10)

        self.subscriptor_ = self.create_subscription(Pose, '/end_effector_pose', self.end_effector_pose_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.create_subscription(Float32MultiArray, "/distance_sensors", self.listener_distance_callback, 10)

    def end_effector_pose_callback(self, msg):
        self.end_effector_pose = msg

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def listener_distance_callback(self, msg):
        if len(msg.data) == 6 and  all(v > 0.0 for v in msg.data) and all(v < 255.0 for v in msg.data):
            ultra1, ultra2, ultra3 = msg.data[0:3]
            s1, s2, s3 = msg.data[3:6]
            self.dA = s1*100
            self.dB = s2*100
            self.dC = s3*100
            # self.dA = ultra3*100    
            # self.dB = ultra2*100
            # self.dC = ultra1*100

            self.get_logger().info(
                f"Ultrasound: [{ultra1:.2f}, {ultra2:.2f}, {ultra3:.2f}] m | "
                f"VL6180X: [{s1:.3f}, {s2:.3f}, {s3:.3f}] m"
            )

            # normal vector to plate
            nV = np.array([0.0, 0.0, 1.0])

            # Wall intersection points
            wA = self.pA + self.dA * nV
            wB = self.pB + self.dB * nV
            wC = self.pC + self.dC * nV
            self.get_logger().info(f"Projected points: {wA}, {wB}, {wC}")

            # Plane vectors
            v1= wB - wA
            v2= wC - wA

            #Wall normal
            nW0= np.cross(v1,v2)
            self.get_logger().info(f"Wall normal vector: {nW0}")
            nw= nW0/np.linalg.norm(nW0)

            # Distance from center (0,0,0) to wall plane
            distance1 = abs(np.dot(nw, wA))
            distance2 = abs(np.dot(nw, wB))
            distance3 = abs(np.dot(nw, wC))
            distance = np.mean([distance1, distance2, distance3])
            self.get_logger().info(f"Distance to wall from center: {distance:.2f} cm")

            # Pitch and yaw error angles 
            gamma = -np.arctan2(nw[1],nw[2])
            beta = np.arctan2(nw[0],nw[2])
            gamma_deg = np.rad2deg(gamma)
            beta_deg = np.rad2deg(beta)
            self.get_logger().info(f"Pitch_ee: {beta_deg:.2f} degrees")
            self.get_logger().info(f"Roll_ee: {gamma_deg:.2f} degrees")

            # Current rotation of the EE w.r.t. base frame
            rot = self.end_effector_pose.orientation
            curr_orn_rot = R.from_quat([rot.x, rot.y, rot.z, rot.w])
            curr_orn = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_euler('ZYX', degrees=True)
            self.get_logger().info(f"Current Orientation (rpy w.r.t. base frame): Roll={curr_orn[0]:.2f}, Pitch={curr_orn[1]:.2f}, Yaw={curr_orn[2]:.2f}")

            # Incremental rotation in EE frame Rotación (order ZYX = Intrinsic roll-pitch-yaw)
            increment_rot_ee = R.from_euler('ZYX', [0, beta, gamma], degrees=False)

            # Rotation Composition: R_goal = R_base * R_increment(EE_frame)
            goal_rot = curr_orn_rot * increment_rot_ee
            # goal_rot_euler = goal_rot.as_euler('ZYX', degrees=True)
            # self.get_logger().info(f"Demanded Orientation (rpy w.r.t. base frame): Roll={goal_rot_euler[0]:.2f}, Pitch={goal_rot_euler[1]:.2f}, Yaw={goal_rot_euler[2]:.2f}")

            # Roll Compensation 
            z_ee = goal_rot.apply([0, 0, 1])    # obtaining Z_EE in base frame
            y_desired = np.array([0.0, 0.0, 1.0])  # obtaining ideal Y axis (-Z on base frame) - vertical vector in the sensors setup
            y_proj = y_desired - np.dot(y_desired, z_ee) * z_ee     # project y_desired on the sensors plane
            norm = np.linalg.norm(y_proj)
            if norm < 1e-6:
                self.get_logger().warn("y_proj is nearly zero: y_desired is aligned with Z_EE.")
            else:
                y_proj /= norm
                y_proj /= np.linalg.norm(y_proj)
                # Reconstruct ortonormal base (X = Y × Z)
                x_ee = np.cross(y_proj, z_ee)
                x_ee /= np.linalg.norm(x_ee)
                y_ee = np.cross(z_ee, x_ee)
                # New rotation with fixed Z_EE and corrected Y_EE
                R_corrected = np.column_stack((x_ee, y_ee, z_ee))
                goal_rot = R.from_matrix(R_corrected)

            # Final quaternion
            q = goal_rot.as_quat()
            final_euler = goal_rot.as_euler('ZYX', degrees=True)
            self.get_logger().info(f"Final Corrected Orientation (rpy w.r.t. base frame): Roll={final_euler[0]:.2f}, Pitch={final_euler[1]:.2f}, Yaw={final_euler[2]:.2f}")

            # Current end effector position
            pos = self.end_effector_pose.position
            p_current = np.array([pos.x, pos.y, pos.z])

            # Compute corrected position
            nw_global = curr_orn_rot.apply(nw)
            self.get_logger().warn(f"Toggle Value: {self.toggle}")
            p_new = p_current + 0*(self.toggle)*(distance - self.ideal_distance) * nw_global / 100     # in meters!!
            # self.toggle *= -1

            # Rotation matrix and transform matrix
            T = np.eye(4)
            T[:3, :3] = R.from_quat(q).as_matrix()
            T[:3, 3] = p_new

            q_current = np.array([self.current_joint_state.position[-1], self.current_joint_state.position[0], self.current_joint_state.position[1], self.current_joint_state.position[2], self.current_joint_state.position[3], self.current_joint_state.position[4]])
            joint_values = closed_form_algorithm(T, q_current, type=0)
            if np.any(np.isnan(joint_values)):
                self.get_logger().error("IK solution contains NaN. Aborting.")
                return
            # if {
            #     (joint_values[0] < -1.57 or joint_values[0] > 1.57) or 
            #     (joint_values[1] < -1.57 or joint_values[1] > 1.57) or 
            #     (joint_values[2] < -1.57 or joint_values[2] > 1.57) or 
            #     (joint_values[3] < -1.57 or joint_values[3] > 1.57) or 
            #     (joint_values[4] < -1.57 or joint_values[4] > 1.57) or 
            #     (joint_values[5] < -1.57 or joint_values[5] > 1.57)
            #     }:
            #     self.get_logger().error("Selected solution with possible collision. Joint values outside of safety margins. Aborting.")
            #     return
            # joint_values[5] = 0.0   # NEEDS TO BE MODIFIED IN CASE ANOTHER INITIAL SENSORS POSITION IS USED!!!
            
            # Publish GoalPose
            traj_msg = JointTrajectory()
            traj_msg.joint_names = [
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint'
            ]
            time_from_start = 0.5
            goal_pose = JointTrajectoryPoint()
            goal_pose.positions = joint_values.tolist()
            # goal_pose.positions[5] += (0.7854+1.5708)
            goal_pose.time_from_start.sec = int(time_from_start)
            goal_pose.time_from_start.nanosec = int((time_from_start % 1.0) * 1e9)
            traj_msg.points.append(goal_pose)
            self.trajectory_pub.publish(traj_msg)

        else:
            self.get_logger().warn("Message recevied was incomplete.")

        if not self.end_effector_pose:
            self.get_logger().info("No end effector pose retrieved yet")
            return

def main(args=None):
    rclpy.init(args=args)
    sensors_orientation = SensorsOrientation()
    rclpy.spin(sensors_orientation)
    sensors_orientation.destroy_node()      # destroy the node 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
