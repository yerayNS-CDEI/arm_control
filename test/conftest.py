"""Put the package root on sys.path so the tests import the sources directly.

arm_control installs its Python modules as symlinks into dist-packages, so an
installed workspace works too -- but the offline kinematics tests must run
without one (ARM_SWEEP_PLAN §11.5 S4 is validated before anything is built).
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
