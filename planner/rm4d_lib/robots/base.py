import numpy as np
from abc import ABC, abstractmethod
from .simulator import Simulator


class RobotBase(ABC):
    @abstractmethod
    def __init__(self, simulator, base_pos=None, base_orn=None):
        """
        A Base Class for a robot.

        :param simulator: Simulation instance (pybullet client)
        :param base_pos: [x, y, z], list of base position
        :param base_orn: [rx, ry, rz, rw], base orientation as quaternion
        """
        self._sim = simulator
        self.base_pos = base_pos or [0, 0, 0.01]
        self.base_orn = base_orn or [0, 0, 0, 1]
        # assert self.base_pos[2] == 0.01

    @property
    def sim(self) -> Simulator:
        """
        :return: simulator, needs to have a bullet_client property.
        """
        return self._sim

    @property
    @abstractmethod
    def range_radius(self) -> float:
        """
        :return: float, upper bound radius of workspace
        """
        pass

    @property
    @abstractmethod
    def range_z(self) -> float:
        """
        :return: float, upper bound z-coordinate for workspace
        """
        pass

    @property
    @abstractmethod
    def end_effector_link_id(self):
        """
        :return: int, the link id of the end-effector
        """
        pass

    @property
    @abstractmethod
    def robot_id(self):
        """
        :return: int, body_id of robot in simulation
        """
        pass

    @property
    @abstractmethod
    def arm_joint_ids(self):
        """
        the joint ids that make up the configuration of the arm, i.e. excluding gripper joints etc
        :return: list
        """

    @property
    @abstractmethod
    def joint_limits(self):
        """
        Gives the joint limits of the robot arm (excluding gripper).

        :return: (n, 2) ndarray, lower and upper limit for each of the n joints
        """
        pass

    def reset_joint_pos(self, q):
        """
        resets the joint position of the robot to the given configuration q.

        :param q: desired joint position
        """
        assert len(q) == len(self.arm_joint_ids), 'invalid joint position list (number of DOF does not match)'
        for i, pos in zip(self.arm_joint_ids, q):
            self.sim.bullet_client.resetJointState(self.robot_id, i, pos)

    def joint_pos(self):
        """
        :return: Gives the current joint position of the robot.
        """
        return np.asarray([self.sim.bullet_client.getJointState(self.robot_id, i)[0] for i in self.arm_joint_ids])

    def forward_kinematics(self, joint_pos=None):
        """
        Calculates the forward kinematics for the given `joint_pos`, or the current joint pos if none given.
        """
        if joint_pos is not None:
            self.reset_joint_pos(joint_pos)
        pos, quat, *_ = self.sim.bullet_client.getLinkState(
            self.robot_id, self.end_effector_link_id, computeForwardKinematics=True)
        return pos, quat

    def inverse_kinematics(self, pos, quat, threshold=25.0, trials=100, seed=None):
        """
        Attempts to find a collision-free, inverse kinematics solution. We perform multiple attempts from random
        starting configurations, with randomly chosen rest configurations for null-space control. Every configuration
        is checked whether it is valid, i.e. close enough to the goal and free from any collisions. Only once a valid
        configuration has been found, it will be returned. If no valid configuration has been found within the number
        of trials, None is returned.

        :param pos: target EE position
        :param quat: target EE orientation
        :param threshold: float, acceptance threshold in combined mm/deg
        :param trials: int, number of attempts
        :param seed: int, random seed (or None)

        :return: valid configuration, or None
        """
        rng = None
        if seed is not None:
            rng = np.random.default_rng(seed)

        for _ in range(trials):
            # use different random starting and resting configurations in each trial
            start_q = self.get_random_joint_config(prevent_collisions=False, rng=rng)
            rest_q = self.get_random_joint_config(prevent_collisions=True, rng=rng)

            # perform IK
            config = self._do_inverse_kinematics(pos, quat, start_q, rest_q, 100, 0.00001)

            # check pose difference
            actual_pos, actual_orn = self.forward_kinematics(config)
            pos_diff = np.linalg.norm(np.asarray(pos) - np.asarray(actual_pos)) * 1000  # m to mm
            orn_diff = self.sim.angle_between_quaternions(quat, actual_orn, as_degree=True)

            if pos_diff + orn_diff > threshold:
                # print("Not below threshold")      # debugging
                continue

            if not self.in_collision():
                return config  # success!
            # return config

        # all trials were unsuccessful
        return None

    @abstractmethod
    def _do_inverse_kinematics(self, pos, quat, start_q, rest_q, n_iterations, threshold):
        pass

    @abstractmethod
    def in_self_collision(self):
        """
        Determines whether the robot is currently in a self-colliding configuration.

        :return: bool, True if in self collision
        """
        pass

    def in_collision_with_plane(self):
        """
        Determines whether the robot is currently in collision with the ground plane.
        :return: bool, True if in collision with plane
        """
        return self.sim.bodies_in_collision(self.robot_id, self.sim.plane_id)

    def in_collision(self):
        """
        Determines whether the robot is in either self-collision or collision with plane.
        :return: bool, True, if the robot is in collision
        """
        # print("Selfcollision: ",self.in_self_collision(),". Plane collision: ",self.in_collision_with_plane())    # debugging
        return self.in_self_collision() or self.in_collision_with_plane()

    def get_random_joint_config(self, prevent_collisions=True, rng=None):
        """
        Returns a random configuration within joint limits, uniformly sampled from C-space.

        :param prevent_collisions: bool, if True, will ensure the configuration is not in collision. Note that
                                   the robot's state will be reset to allow for the check.
        :param rng: random number generator, optional (if None, a default one will be used)
        :return: ndarray, the configuration
        """
        if rng is None:
            rng = np.random.default_rng()

        q_rand = rng.uniform(low=self.joint_limits[:, 0], high=self.joint_limits[:, 1])

        if prevent_collisions:
            self.reset_joint_pos(q_rand)
            while self.in_collision():
                q_rand = rng.uniform(low=self.joint_limits[:, 0], high=self.joint_limits[:, 1])
                self.reset_joint_pos(q_rand)

        return q_rand

    def remove_from_sim(self):
        """
        Removes this robot from the simulator.
        Note that after this operation, most functions will not work properly anymore.
        """
        self.sim.bullet_client.removeBody(self.robot_id)
        self._sim = None
