import numpy as np
from tqdm import tqdm
from .robots import RobotBase
from .rmap import MapBase


class JointSpaceConstructor:
    def __init__(self, rmap: MapBase, robot: RobotBase, seed=0):
        self.map = rmap
        self.robot = robot
        self.rng = np.random.default_rng(seed=seed)

    def sample(self, n_samples=10000, prevent_collisions=True, hit_stats=None):
        max_pz = -np.inf  # al principio de sample()

        for _ in tqdm(range(n_samples)):
            # sample random collision-free config from robot
            q_rand = self.robot.get_random_joint_config(prevent_collisions=prevent_collisions, rng=self.rng)

            # get EE pose as TF_EE
            ee_pos, ee_quat = self.robot.forward_kinematics(q_rand)
            tf_ee = self.robot.sim.pos_quat_to_tf(ee_pos, ee_quat)

            # get maximum p_z observed
            pz = tf_ee[2, 3]
            if pz > max_pz:
                max_pz = pz
                print(f"[INFO] Nuevo p_z máximo observado: {pz:.4f}")

            idcs = self.map.get_indices_for_ee_pose(tf_ee)
            self.map.mark_reachable(idcs)

            if hit_stats is not None:
                hit_stats.record_access(idcs)
