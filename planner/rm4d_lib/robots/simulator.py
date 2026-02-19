import numpy as np
import pybullet as p
import pybullet_data
from pybullet_utils import bullet_client
from scipy.spatial.transform import Rotation

from .assets import FRAME_VIS_URDF


class Simulator:
    """
    Simulator class which wraps around pybullet and provides a few convenience functions.

    :param with_gui: If set to True, it will show the simulation in GUI mode.
    """
    def __init__(self, with_gui=False):
        self.verbose = with_gui
        self.dt = 1. / 240.  # this is the default and should not be changed light-heartedly
        self.SOLVER_STEPS = 100  # a bit more than default helps in contact-rich tasks
        self.TIME_SLEEP = self.dt * 2  # for visualization

        self._recording_config = None

        self._p = None
        self._reset()
        self.plane_id = self._load_plane_and_gravity()

    @property
    def bullet_client(self):
        """Gives the bullet_client for direct control."""
        return self._p

    def reset(self):
        """
        Completely reset the simulation environment.
        """
        self._reset()
        self.plane_id = self._load_plane_and_gravity()

    def _reset(self):
        """
        Ensures a fresh bullet-client is connected.
        """
        if self._p is None:
            # connect using bullet client makes sure we can connect to multiple servers in parallel
            self._p = bullet_client.BulletClient(connection_mode=p.GUI if self.verbose else p.DIRECT)
            self._p.setAdditionalSearchPath(pybullet_data.getDataPath())
        else:
            self._p.resetSimulation()

        self._p.setPhysicsEngineParameter(fixedTimeStep=self.dt, numSolverIterations=self.SOLVER_STEPS)
        if self.verbose:
            self._p.resetDebugVisualizerCamera(cameraDistance=1.7, cameraYaw=50, cameraPitch=-40,
                                               cameraTargetPosition=[0, 0, 0.4])
            self._p.configureDebugVisualizer(self._p.COV_ENABLE_GUI, 0)

    def _load_plane_and_gravity(self):
        """
        Loads a plane and sets gravity.
        :return: int, body ID of the plane
        """
        plane_id = self._p.loadURDF("plane.urdf")
        self._p.setGravity(0, 0, -9.81)
        self._p.changeDynamics(plane_id, -1, lateralFriction=1.0)
        return plane_id

    def add_frame(self, pos=None, orn=None, scale=0.1):
        """
        Adds a coordinate system (frame) for visualization purposes.

        :param pos: [x, y, z] position
        :param orn: [rx, ry, rz, rw] quaternion
        :param scale: float, scaling factor to adjust size of the coordinate system
        :return: int, body ID of the added frame
        """
        pos = pos if pos is not None else [0, 0, 0]
        orn = orn if orn is not None else [0, 0, 0, 1]
        pose = self.pos_quat_to_tf(pos, orn)

        body_id = self.bullet_client.loadURDF(FRAME_VIS_URDF, useFixedBase=True, globalScaling=scale)

        # correct pose to origin instead of COM
        com = np.array(self.bullet_client.getDynamicsInfo(body_id, -1)[3])
        com_correction_tf = np.eye(4)
        com_correction_tf[0:3, 3] = com
        start_pose = pose @ com_correction_tf
        pos, quat = self.tf_to_pos_quat(start_pose)
        self.bullet_client.resetBasePositionAndOrientation(body_id, pos, quat)

        return body_id

    def add_sphere(self, pos, radius, color):
        """
        Adds a sphere visualization to the simulation.
        :param pos: [x, y, z] position
        :param radius: float, radius
        :param color: [r, g, b, a] colour, can also provide [r, g, b], then [1.0] is added for alpha
        """
        if len(color) == 3:
            rgba = color + [1.0]
        else:
            assert len(color) == 4
            rgba = color
        visual_shape = self.bullet_client.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=rgba)
        body_id = self.bullet_client.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape, basePosition=pos)
        return body_id
    
    def add_sphere_v2(self, pos, radius, color, visual_shape_id=None):
        if visual_shape_id is None:
            if len(color) == 3:
                rgba = color + [1.0]
            else:
                rgba = color
            visual_shape_id = self.bullet_client.createVisualShape(
                p.GEOM_SPHERE, radius=radius, rgbaColor=rgba)

        return self.bullet_client.createMultiBody(
            baseMass=0, baseVisualShapeIndex=visual_shape_id, basePosition=pos)

    def bodies_in_collision(self, body1, body2, threshold=-0.001):
        """
        checks if two bodies are in collision with each other.

        :param body1: first body id
        :param body2: second body id
        :param threshold: float, distance upon which we recognise it as a collision

        :return: bool, True if the two bodies are in collision
        """
        # in contrast to getContactPoints, this also works before stepSimulation or performCollisionDetection
        max_distance = 0.1  # do not return any points for objects that are farther apart than this
        points = self._p.getClosestPoints(body1, body2, max_distance)

        for point in points:
            distance = point[8]
            if distance < threshold:
                return True

        return False

    def links_in_collision(self, body1, link1, body2, link2, threshold=-0.001):
        """
        checks if two links are in collision with each other.

        :param body1: first body id
        :param link1: link id of first link
        :param body2: second body id
        :param link2: link id of second link
        :param threshold: float, distance upon which we recognise it as a collision

        :return: bool, True if the two links are in collision
        """
        # in contrast to getContactPoints, this also works before stepSimulation or performCollisionDetection
        max_distance = 0.1  # do not return any points for objects that are farther apart than this
        points = self._p.getClosestPoints(body1, body2, max_distance, linkIndexA=link1, linkIndexB=link2)

        for point in points:
            distance = point[8]
            if distance < threshold:
                return True

        return False

    def disconnect(self):
        """
        This method shall be called when the simulation is not needed anymore as it cleans up the object.
        """
        self._p.disconnect()

    @staticmethod
    def tf_to_pos_quat(tf):
        pos = tf[:3, 3]
        quat = Rotation.from_matrix(tf[:3, :3]).as_quat()
        return pos, quat

    @staticmethod
    def pos_quat_to_tf(pos, quat):
        tf = np.eye(4)
        tf[:3, 3] = np.array(pos)
        tf[:3, :3] = Rotation.from_quat(quat).as_matrix()
        return tf

    @staticmethod
    def angle_between_quaternions(q1, q2, as_degree=False):
        """
        calculates the angle between two quaternions that have pybullet (xyzw) convention. (per default in radian)
        """
        diff_quat = p.getDifferenceQuaternion(q1, q2)
        angle = 2 * np.arccos(np.clip(np.abs(diff_quat[3]), 0, 1))

        if as_degree:
            angle = np.rad2deg(angle)
        return angle
