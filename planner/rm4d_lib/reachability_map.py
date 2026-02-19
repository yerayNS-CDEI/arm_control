import numpy as np
from matplotlib import pyplot as plt

from .rmap import MapBase


class ReachabilityMap4D(MapBase):
    def __init__(self, xy_limits=None, z_limits=None, voxel_res=0.05, n_bins_theta=36, no_map=False):
        if xy_limits is None:
            xy_limits = [-1.05, 1.05]
        if z_limits is None:
            z_limits = [0, 1.35]

        # [min, max]
        self.xy_limits = xy_limits
        self.z_limits = z_limits
        self.theta_limits = [0, np.pi]

        # get dimensions
        self.voxel_res = voxel_res
        self.n_bins_xy = int(np.ceil((self.xy_limits[1] - self.xy_limits[0])/voxel_res))
        self.n_bins_z = int(np.ceil((self.z_limits[1] - self.z_limits[0])/voxel_res))
        self.n_bins_theta = n_bins_theta

        # check achieved resolution
        print(voxel_res,self.n_bins_z)
        assert np.isclose((self.xy_limits[1] - self.xy_limits[0])/self.n_bins_xy, self.voxel_res)
        assert np.isclose((self.z_limits[1] - self.z_limits[0])/self.n_bins_z, self.voxel_res)
        self.theta_res = (self.theta_limits[1] - self.theta_limits[0])/self.n_bins_theta

        # create map
        if no_map:
            self.map = None
        else:
            self.map = np.zeros(
                shape=(self.n_bins_z, self.n_bins_theta, self.n_bins_xy, self.n_bins_xy),
                dtype=bool
            )

    def to_file(self, filename):
        save_dict = {
            'map': self.map,
            'xy_limits': self.xy_limits,
            'z_limits': self.z_limits,
            'voxel_res': self.voxel_res,
            'n_bins_theta': self.n_bins_theta,
        }
        np.save(filename, save_dict)

    @classmethod
    def from_file(cls, filename):
        d = np.load(filename, allow_pickle=True).item()
        rm = cls(xy_limits=d['xy_limits'],
                 z_limits=d['z_limits'],
                 voxel_res=d['voxel_res'],
                 n_bins_theta=d['n_bins_theta'],
                 no_map=True)
        rm.map = d['map']

        print(f'{cls.__name__} loaded from {filename}')
        rm.print_structure()
        return rm

    def print_structure(self):
        print(f'structure of reachability map:')
        print(f'\txy: {self.xy_limits}, {self.n_bins_xy} bins, {self.voxel_res} resolution')
        print(f'\t z: {self.z_limits}, {self.n_bins_z} bins, {self.voxel_res} resolution')
        print(f'\tth: {self.theta_limits}, {self.n_bins_theta} bins, {self.theta_res} resolution')
        print(f'total elements: {self.map.size}')
        print(f'memory required: {self.map.nbytes / 1024 / 1024:.2f}MB')

    def get_p_z(self, tf_ee):
        """
        Gets the z-coordinate of the EE position.
        """
        return tf_ee[2, 3]

    def get_rotation_2d(self, tf_ee):
        """
        Gives the rotation that aligns tf_ee such that its z-axis is in the x+z plane as 2d rotation matrix.
        """
        rz_x, rz_y = tf_ee[:2, 2]  # first two components of the z-axis
        # get angle of rotation
        psi = np.arctan2(rz_y, rz_x)
        # build inverse rotation matrix to rotate back by psi
        rot_mat_2d = np.array([
            [np.cos(psi), np.sin(psi)],
            [-np.sin(psi), np.cos(psi)]
        ])

        return rot_mat_2d

    def get_theta(self, tf_ee):
        """
        Gets the angle between EE's r_z and the world z axis in rad.
        """
        # dot product: [0, 0, 1] dot [rz_x, rz_y, rz_z] -- simplifies to rz_z
        rz_z = tf_ee[2, 2]
        theta = np.arccos(rz_z)
        return theta

    def get_canonical_base_position(self, tf_ee):
        """
        Calculates (x*, y*) for a given EE pose.
        """
        p_x, p_y = tf_ee[:2, 3]
        rot2d = self.get_rotation_2d(tf_ee)
        x_star, y_star = rot2d @ np.array([-p_x, -p_y])
        return x_star, y_star

    def get_z_index(self, p_z):
        """
        Given a p_z, gives the corresponding index in the map.
        """
        z_idx = int((p_z - self.z_limits[0]) / self.voxel_res)
        if z_idx < 0:
            raise IndexError(f'z idx < 0 -- {p_z}')
        if z_idx >= self.n_bins_z:
            raise IndexError(f'z idx too large -- {p_z}')
        return z_idx

    def get_theta_index(self, theta):
        """
        Given the value of theta, gives the corresponding index.
        """
        # if theta is pi, we want it to be included in the last bin
        if np.isclose(theta, np.pi):
            return self.n_bins_theta - 1

        theta_idx = int((theta - self.theta_limits[0]) / self.theta_res)
        if theta_idx < 0:
            raise IndexError(f'theta idx < 0 -- {theta}')
        if theta_idx >= self.n_bins_theta:
            raise IndexError(f'theta idx too large -- {theta}')
        return theta_idx

    def get_xy_index(self, xy):
        """
        Given x* or y* from the canonical base position, gives the corresponding index.
        """
        x_idx = int((xy - self.xy_limits[0]) / self.voxel_res)
        if x_idx < 0:
            raise IndexError(f'xy_idx < 0 -- {xy}')
        if x_idx >= self.n_bins_xy:
            raise IndexError(f'xy idx too large -- {xy}')
        return x_idx

    def get_indices_for_ee_pose(self, tf_ee):
        """
        Gives the indices to the element of the reachability map that corresponds to the given end-effector pose.
        May throw an IndexError if the pose is not in the map.

        :param tf_ee: ndarray (4, 4), end-effector pose
        :returns: tuple, indices for the map
        """
        # perform the dimensionality reduction
        p_z = self.get_p_z(tf_ee)
        theta = self.get_theta(tf_ee)
        x_star, y_star = self.get_canonical_base_position(tf_ee)

        # determine indices
        z_idx = self.get_z_index(p_z)
        theta_idx = self.get_theta_index(theta)
        x_idx = self.get_xy_index(x_star)
        y_idx = self.get_xy_index(y_star)

        return z_idx, theta_idx, x_idx, y_idx

    @property
    def shape(self):
        return self.map.shape

    def mark_reachable(self, map_indices):
        self.map[map_indices] = 1

    def is_reachable(self, map_indices):
        return self.map[map_indices]

    def _get_xy_points(self):
        """
        gives a set of points, where each point is in the centre of an xy bin
        :returns: ndarray, (n_bins_xy, n_bins_xy, 2)
        """
        points = np.empty(shape=(self.n_bins_xy, self.n_bins_xy, 2))
        for i in range(self.n_bins_xy):
            coord = self.xy_limits[0] + (i + 0.5) * self.voxel_res  # use center point of bin
            points[i, :, 0] = coord
            points[:, i, 1] = coord

        return points

    def get_base_positions(self, tf_ee, as_3d=False):
        """
        Inverse operation - retrieves the base position given an end-effector pose.
        :param tf_ee: (4, 4) ndarray, requested pose of end-effector.
        :param as_3d: bool, if True, will add a 0 z-coordinate to the points
        :return: (n, 3) ndarray, containing (x, y, score); (x, y, z, score) if as_3d is set to True
        """
        # IDENTIFICATION
        # given the tf_ee, we need to determine z_ee and theta, such that we can retrieve the x/y slice
        p_z = self.get_p_z(tf_ee)
        z_idx = self.get_z_index(p_z)

        theta = self.get_theta(tf_ee)
        theta_idx = self.get_theta_index(theta)

        map_2d = self.map[z_idx, theta_idx]

        # BACK-PROJECTION
        # we want to transform the x/y slice into 3d points w/ additional field for the value
        # the 3d points get back-projected, i.e. un-rotate, and then shift xy
        pts = self._get_xy_points()  # (n_bins_xy, n_bins_xy, 2)
        pts = pts.reshape(-1, 2)

        # apply inverse rotation
        rot_2d = self.get_rotation_2d(tf_ee)
        pts = (np.linalg.inv(rot_2d) @ pts.T).T

        # apply offset
        pts[:, 0] += tf_ee[0, 3]
        pts[:, 1] += tf_ee[1, 3]

        # shape back to concatenate with reachability scores
        pts = pts.reshape(self.n_bins_xy, self.n_bins_xy, 2)

        # elevate to 3d and add reachability scores as 4th dim
        if as_3d:
            points_3d = np.concatenate([pts,
                                        np.zeros((self.n_bins_xy, self.n_bins_xy, 1)),
                                        map_2d.reshape(self.n_bins_xy, self.n_bins_xy, 1)],
                                       axis=-1)
            return points_3d.reshape(-1, 4)
        else:
            points_2d = np.concatenate([pts,
                                        map_2d.reshape(self.n_bins_xy, self.n_bins_xy, 1)],
                                       axis=-1)
            return points_2d.reshape(-1, 3)

    def show_occupancy_per_dim(self):
        """
        shows occupancy per each dimension of the map as a bar plot
        """
        print(f'OCCUPANCY OF REACHABILITY MAP')
        print(f'total elements: {self.map.size}')
        print(f'non-zero elements: {np.count_nonzero(self.map)}')
        print(f'percentage: {np.count_nonzero(self.map) / self.map.size}')
        print(f'average: {np.mean(self.map)}, min: {np.min(self.map)}, max: {np.max(self.map)}')

        # sum the occupancies
        z_data = np.apply_over_axes(np.sum, self.map, [1, 2, 3]).flatten()
        theta_data = np.apply_over_axes(np.sum, self.map, [0, 2, 3]).flatten()
        x_data = np.apply_over_axes(np.sum, self.map, [0, 1, 3]).flatten()
        y_data = np.apply_over_axes(np.sum, self.map, [0, 1, 2]).flatten()

        # visualise
        fig, axes = plt.subplots(nrows=2, ncols=2)
        axes[0, 0].bar(np.arange(self.n_bins_z), z_data)
        axes[0, 1].bar(np.arange(self.n_bins_theta), theta_data)
        axes[1, 0].bar(np.arange(self.n_bins_xy), x_data)
        axes[1, 1].bar(np.arange(self.n_bins_xy), y_data)

        axes[0, 0].set_title(f'z {self.z_limits}')
        axes[0, 1].set_title(f'theta {self.theta_limits}')
        axes[1, 0].set_title(f'x {self.xy_limits}')
        axes[1, 1].set_title(f'y {self.xy_limits}')
        plt.show()

    def visualize_in_sim(self, sim, tf_ee, skip_zero=True):
        points_4d = self.get_base_positions(tf_ee, as_3d=True)
        max_val = np.max(points_4d[:, 3])

        for point in points_4d:
            val = point[3]
            if skip_zero and val == 0:
                continue
            sim.add_sphere(list(point[:3]), radius=0.025, color=[1.0 - val/max_val, val/max_val, 0.0])
            # visual_shape = sim.bullet_client.createVisualShape(p.GEOM_BOX,
            #                                                        halfExtents=[self.x_res/2, self.y_res/2, 0.005],
            #                                                        rgbaColor=[1.0 - val/max_val, val/max_val, 0.0])
            # x = self.x_limits[0] + (i + 0.5) * self.x_res
            # y = self.y_limits[0] + (j + 0.5) * self.y_res
            # sim.bullet_client.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape, basePosition=[x, y, 0])
