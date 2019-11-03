import sapyen
import sapyen_robot
from .base_env import BaseEnv
from typing import List, Union
import numpy as np


class BaseRobotEnv(BaseEnv):
    """
    Virtual Class for robot holder, it can not be initiate on any circumstance.
    It will not load any specific controllers
    """

    def __init__(self, urdf_path: str, material: sapyen.PxMaterial):
        super(BaseRobotEnv, self).__init__(False)
        self._load_robot(urdf_path, material)
        raise NotImplementedError

    def robot_name2link(self, name: str) -> sapyen.PxRigidBody:
        return self.__robot_name2link[name]

    def _load_controller(self):
        """
        Pure virtual function for loading controllers. It should always be overloaded by child class
        """
        self.manger = sapyen_robot.ControllerManger("virtual", None)
        raise NotImplementedError

    def _load_robot(self, urdf_path: str, material: sapyen.PxMaterial):

        # By default, the robot will loaded with balanced passive force
        self.loader.fix_loaded_object = True
        self.loader.balance_passive_force = True
        self.robot = self.loader.load(urdf_path, material)
        self.robot.set_root_pose([0, 0, 0], [1, 0, 0, 0])

        # Link mapping
        links = self.robot.get_links()
        link_names = self.robot.get_link_names()
        self.__robot_name2link = dict(zip(link_names, links))

        # Load controllers, should be implemented with specific robot
        self._load_controller()

    @property
    def robot_joint_names(self) -> List[str]:
        return self.robot.get_drive_joint_names()

    @property
    def robot_link_names(self) -> List[str]:
        return self.robot.get_link_names()

    @property
    def robot_links(self) -> List[sapyen.PxRigidBody]:
        return self.robot.get_links()

    @property
    def robot2global(self) -> sapyen.Pose:
        """
        Get robot root pose in the global coordinate
        :return: Robot root pose
        """
        return self.robot_name2link("right_ee_link").get_global_pose()

    def set_robot_base_pose(self, pose: Union[np.ndarray, List]) -> None:
        assert len(pose) == 7, "Pose should be in Position: x y z, Quaternion: w x y z format"
        self.manger.move_base(pose[0:3], pose[3:7])
