import numpy as np
import pybullet as p
from .base import RobotBase
from .assets import UR10E_URDF
from ament_index_python.packages import get_package_share_directory
import os
import tempfile
import re


class UR10E(RobotBase):
    def __init__(self, simulator, base_pos=None, base_orn=None):
        super().__init__(simulator, base_pos, base_orn)
        
        ur_desc_share = get_package_share_directory("ur_description")
        ur_desc_meshes = os.path.join(ur_desc_share, "meshes")

        # Make a temporary URDF with absolute mesh paths so PyBullet can't fail resolving them
        with open(UR10E_URDF, "r") as f:
            urdf_txt = f.read()

        # Replace filename="meshes/..." or filename='meshes/...'
        urdf_txt = re.sub(r'filename="meshes/', f'filename="{ur_desc_meshes}/', urdf_txt)
        urdf_txt = re.sub(r"filename='meshes/", f"filename='{ur_desc_meshes}/", urdf_txt)

        tmp = tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".urdf")
        tmp.write(urdf_txt)
        tmp.close()

        print(f"[UR10E] URDF (original): {UR10E_URDF}")
        print(f"[UR10E] URDF (patched):  {tmp.name}")
        print(f"[UR10E] Mesh root:       {ur_desc_meshes}")

        self._robot_id = self.sim.bullet_client.loadURDF(
            tmp.name, self.base_pos, self.base_orn, useFixedBase=True
        )
        os.unlink(tmp.name)

        num_joints = self.sim.bullet_client.getNumJoints(self._robot_id)
        print("Num joints in model:", num_joints)
        for i in range(num_joints):
            # print(i, self.sim.bullet_client.getJointInfo(self._robot_id, i)[1])
            info = self.sim.bullet_client.getJointInfo(self.robot_id, i)
            print(f"Joint {i} - Name: {info[1].decode()} --> Link: {info[12].decode()}, limits: {info[8]} to {info[9]}")

        end_effector_link_name = "tool0"
        end_effector_link_index = None

        for i in range(num_joints):
            joint_info = self.sim.bullet_client.getJointInfo(self.robot_id, i)
            link_name = joint_info[12].decode("utf-8")
            if link_name == end_effector_link_name:
                end_effector_link_index = i
                break

        print(f"End effector link index: {end_effector_link_index}")

        # Verificar el offset entre wrist_3_link y tool0
        wrist_3_index = None
        for i in range(num_joints):
            link_name = self.sim.bullet_client.getJointInfo(self.robot_id, i)[12].decode("utf-8")
            if link_name == "wrist_3_link":
                wrist_3_index = i
                break

        if wrist_3_index is not None:
            tool_pos = self.sim.bullet_client.getLinkState(self.robot_id, end_effector_link_index)[0]
            wrist_pos = self.sim.bullet_client.getLinkState(self.robot_id, wrist_3_index)[0]
            z_offset = tool_pos[2] - wrist_pos[2]
            print(f"[DEBUG] Offset TCP (tool0) - wrist_3_link en Z: {z_offset:.5f} m")
        else:
            print("[WARNING] No se encontró wrist_3_link")

        # robot properties
        self._end_effector_link_id = end_effector_link_index      # index of TCP
        self._arm_joint_ids = [2, 3, 4, 5, 6, 7]    # movable joints
        self.home_conf = [1.57, -1.57, 1.57, -1.57, -1.57, 0.0]

        # for i in range(self.sim.bullet_client.getNumJoints(self.robot_id)):
        #     print(i, self.sim.bullet_client.getJointInfo(self.robot_id, i)[12])

        # determine joint limits
        limits = []
        for i in self.arm_joint_ids:
            limits.append(self.sim.bullet_client.getJointInfo(self.robot_id, i)[8:10])
        self._joint_limits = np.asarray(limits)

        # set initial robot configuration
        self.reset_joint_pos(self.home_conf)

    @property
    def range_radius(self) -> float:
        return 1.35     # it should be 1.4 to be more than the actual reach

    @property
    def range_z(self) -> float:
        return 1.6     # it should be 1.4 to be more than the actual reach

    @property
    def end_effector_link_id(self):
        return self._end_effector_link_id

    @property
    def robot_id(self):
        return self._robot_id

    @property
    def arm_joint_ids(self):
        return self._arm_joint_ids

    @property
    def joint_limits(self):
        return self._joint_limits

    def in_self_collision(self):
        # # check self-collision
        # ignore_links = []  # they do not have a collision shape
        # first_links = [0, 1, 2, 3, 4]  # 5 cannot collide with the fingers due to kinematics

        # for first_link in first_links:
        #     # skip links that are next to each other (supposed to be in contact) plus all the ignore links
        #     check_links = [link for link in np.arange(first_link + 2, self.end_effector_link_id + 1) if
        #                    link not in ignore_links]
        #     for check_link in check_links:
        #         collision = self.sim.links_in_collision(self.robot_id, first_link, self.robot_id, check_link)
        #         if collision:
        #             return True
        # return False

        collision_links = [2, 3, 4, 5, 6, 7]

        for i, link_a in enumerate(collision_links):
            for link_b in collision_links[i + 1:]:
                if abs(link_a - link_b) == 1:
                    continue  # saltar vecinos directos
                if self.sim.links_in_collision(self.robot_id, link_a, self.robot_id, link_b):
                    # name_a = self.sim.bullet_client.getJointInfo(self.robot_id, link_a)[12].decode()
                    # name_b = self.sim.bullet_client.getJointInfo(self.robot_id, link_b)[12].decode()
                    # print(f"[Collision] Detected between link {link_a} ({name_a}) and link {link_b} ({name_b})")
                    # input("Press Enter to continue...")
                    return True
        return False

        # # If the robot changes the collision links can be detected automatically
        # collision_links = [
        #     i for i in range(self.sim.bullet_client.getNumJoints(self.robot_id))
        #     if self.sim.bullet_client.getCollisionShapeData(self.robot_id, i)
        # ]

        # for i, link_a in enumerate(collision_links):
        #     for link_b in collision_links[i + 1:]:
        #         if abs(link_a - link_b) == 1:
        #             continue  # skip neighbors
        #         if self.sim.links_in_collision(self.robot_id, link_a, self.robot_id, link_b):
        #             return True
        # return False

    def _do_inverse_kinematics(self, pos, quat, start_q, rest_q, n_iterations, threshold):
        self.reset_joint_pos(start_q)

        # get joint ranges and limits
        lower_limits = self.joint_limits[:, 0]
        upper_limits = self.joint_limits[:, 1]
        joint_ranges = upper_limits - lower_limits
        rest_poses = rest_q.tolist()

        # transform them to lists (if not, there are errors in pybullet)
        lower_limits = lower_limits.tolist()
        upper_limits = upper_limits.tolist()
        joint_ranges = joint_ranges.tolist()

        # # include finger joints
        # lower_limits = lower_limits.tolist() + [0.0]*6
        # upper_limits = upper_limits.tolist() + [0.0]*6
        # joint_ranges = joint_ranges.tolist() + [0.0]*6
        # rest_poses = rest_q.tolist() + [0.0]*6

        # print("Robot ID:", self.robot_id)
        # print("EE link ID:", self.end_effector_link_id)
        # print("Pos:", pos)
        # print("Quat:", quat)
        # print("Lower:", lower_limits)
        # print("Upper:", upper_limits)
        # print("Ranges:", joint_ranges)
        # print("Rest poses:", rest_poses)
        # print("Num joints:", len(self.arm_joint_ids))

        # execute IK
        full_joint_positions = self.sim.bullet_client.calculateInverseKinematics(
            self.robot_id, self.end_effector_link_id, pos, quat,
            lowerLimits=lower_limits, upperLimits=upper_limits, jointRanges=joint_ranges, restPoses=rest_poses,
            maxNumIterations=n_iterations, residualThreshold=threshold)
        # full_joint_positions = self.sim.bullet_client.calculateInverseKinematics(
        #     self.robot_id, self.end_effector_link_id, pos, quat)

        # print("Returned IK length:", len(full_joint_positions))
        # print("arm_joint_ids:", self.arm_joint_ids)
        
        return np.array(full_joint_positions)
