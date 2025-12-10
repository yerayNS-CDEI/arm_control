import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
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
        self.current_joint_state = None
        self.min_distance = 18.0    # cm
        self.max_distance = 22.0    # cm
        self.ideal_distance = 20.0  # cm
        self.toggle = 1
        # 3 sensors: A left, B right, C top 
        # Positions on the plate 
        # Define the positions of the sensors
        xA, yA = -10.0, -1.0 # Position of sensor A 
        xB, yB = 10.0, -1.0 # Position of sensor B 
        xC, yC = 0.0, 10.0 # Position of sensor C 
        self.pA= np.array([xA, yA, 0.0])
        self.pB= np.array([xB, yB, 0.0])
        self.pC= np.array([xC, yC, 0.0])

        self.trajectory_pub = self.create_publisher(JointTrajectory, '/arm/planned_trajectory', 10)

        self.subscriptor_ = self.create_subscription(Pose, '/arm/end_effector_pose', self.end_effector_pose_callback, 10)
        self.create_subscription(JointState, "/arm/joint_states", self.joint_state_callback, 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def end_effector_pose_callback(self, msg):
        self.end_effector_pose = msg

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def timer_callback(self):

        if not self.end_effector_pose:
            self.get_logger().info("No end effector pose retrieved yet")
            return
        
        if not self.current_joint_state:
            self.get_logger().info("No joint state received yet")
            return

        if not self.current_joint_state.position or len(self.current_joint_state.position) < 6:
            self.get_logger().warn("JointState has no positions or fewer than 6 joints")
            return

        # Distance readings from sensors
        dA = np.random.uniform(self.min_distance, self.max_distance)
        dB = np.random.uniform(self.min_distance, self.max_distance)
        dC = np.random.uniform(self.min_distance, self.max_distance)
        # dB = dA
        # dC = dA

        # normal vector to plate
        nV = np.array([0.0, 0.0, 1.0])

        # Wall intersection points
        wA = self.pA + dA * nV
        wB = self.pB + dB * nV
        wC = self.pC + dC * nV

        # Plane vectors
        v1= wB - wA
        v2= wC - wA

        #Wall normal
        nW0= np.cross(v1,v2)
        nw= nW0/np.linalg.norm(nW0)

        # Distance from center (0,0,0) to wall plane
        distance1 = abs(np.dot(nw, wA))
        distance2 = abs(np.dot(nw, wB))
        distance3 = abs(np.dot(nw, wC))
        distance = np.mean([distance1, distance2, distance3])
        self.get_logger().info(f"Distance to wall from center: {distance:.2f} cm")

        # Pitch and yaw error angles 
        yaw = np.arctan2(nw[0],nw[2])
        pitch = np.arctan2(nw[1],nw[2])
        yaw_deg = np.rad2deg(yaw)
        pitch_deg = np.rad2deg(pitch)
        self.get_logger().info(f"Pitch: {pitch_deg:.2f} degrees")
        self.get_logger().info(f"Yaw: {yaw_deg:.2f} degrees")
        
        # Current rotation of the EE w.r.t. base frame
        rot = self.end_effector_pose.orientation
        curr_orn_rot = R.from_quat([rot.x, rot.y, rot.z, rot.w])
        curr_orn = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_euler('ZYX', degrees=True)
        self.get_logger().info(f"Current Orientation (rpy w.r.t. base frame): Roll={curr_orn[0]:.2f}, Pitch={curr_orn[1]:.2f}, Yaw={curr_orn[2]:.2f}")

        # Incremental rotation in EE frame Rotación (order ZYX = Intrinsic roll-pitch-yaw)
        increment_rot_ee = R.from_euler('ZYX', [0, pitch, yaw], degrees=False)

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
        p_new = p_current - (self.toggle)*abs(distance - self.ideal_distance) * nw_global / 100
        self.toggle *= -1

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
        goal_pose.time_from_start.sec = int(time_from_start)
        goal_pose.time_from_start.nanosec = int((time_from_start % 1.0) * 1e9)
        traj_msg.points.append(goal_pose)
        self.trajectory_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    sensors_orientation = SensorsOrientation()
    rclpy.spin(sensors_orientation)
    sensors_orientation.destroy_node()      # destroy the node 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    