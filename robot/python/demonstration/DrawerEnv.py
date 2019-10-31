import sapyen
import sapyen_robot
import os
import transforms3d
import numpy as np
import warnings

PARTNET_DIR = '/home/sim/project/mobility-v0-prealpha3/mobility_verified'
RGBD_CAMERA_THRESHOLD = 10
CAMERA_TO_LINK = np.zeros([4, 4])
CAMERA_TO_LINK[[0, 1, 2, 3], [2, 0, 1, 3]] = [1, -1, -1, 1]


class DrawerEnv(MOVOSapienEnv):
    def __init__(self, partnet_id: str, root_pose=((3, 0, 0.5), (1, 0, 0, 0))):
        # Partnet Object
        super(DrawerEnv, self).__init__(partnet_id, root_pose)
