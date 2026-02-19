import os

_assets_dir = os.path.dirname(os.path.realpath(__file__))

FRAME_VIS_URDF = os.path.join(_assets_dir, 'frame_vis.urdf')
UR5E_URDF = os.path.join(_assets_dir, 'ur5e_2f85.urdf')
UR10E_URDF = os.path.join(_assets_dir, 'ur10e.urdf')
FRANKA_URDF = os.path.join(_assets_dir, 'franka_panda/panda166.urdf')
FRANKA_180_URDF = os.path.join(_assets_dir, 'franka_panda/panda180.urdf')
FRANKA_160_URDF = os.path.join(_assets_dir, 'franka_panda/panda160.urdf')
FRANKA_150_URDF = os.path.join(_assets_dir, 'franka_panda/panda150.urdf')
