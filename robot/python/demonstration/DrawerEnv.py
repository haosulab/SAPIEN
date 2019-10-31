from .MOVOSapienEnv import MOVOSapienEnv


class DrawerEnv(MOVOSapienEnv):
    def __init__(self, partnet_id: str, root_pose=((3, 0, 0.5), (1, 0, 0, 0))):
        # Partnet Object
        super(DrawerEnv, self).__init__(partnet_id, root_pose)
