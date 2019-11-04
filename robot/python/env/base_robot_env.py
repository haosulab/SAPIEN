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

    def __init__(self):
        super(BaseRobotEnv, self).__init__(True)

    def robot_name2link(self, name: str) -> sapyen.PxRigidBody:
        return self.__robot_name2link[name]

    def _load_ros_controller(self):
        """
        Pure virtual function for loading controllers. It should always be overloaded by child class
        """
        self.manger = sapyen_robot.ControllerManger("virtual", None)
        raise NotImplementedError

    def _load_controller_parameters(self):
        self.robot.set_drive_qpos(None)
        raise NotImplementedError

    def _load_robot(self, urdf_path: str, material: sapyen.PxMaterial) -> None:
        # By default, the robot will loaded with balanced passive force
        self.loader.fix_loaded_object = True
        self.loader.balance_passive_force = True
        self.robot: sapyen.ArticulationWrapper = self.loader.load(urdf_path, material)
        self.robot.set_root_pose([0, 0, 0], [1, 0, 0, 0])

        # Link mapping, remember to set the self._base_link_name if your robot base is not that name
        links: List[sapyen.PxRigidBody] = self.robot.get_links()
        link_names: List[str] = self.robot.get_link_names()
        self.__robot_name2link = dict(zip(link_names, links))
        self._base_link_name = "base_link"

        # Load controller parameters but not init all controllers. Since recorder may have already these.
        self._load_controller_parameters()

        # Set mapping and other staff for the camera loaded with robot urdf
        self._init_camera_cache()

    def get_robot_link_global_pose_by_name(self, name: str) -> sapyen.Pose:
        """
        :param name: link_name
        :return: Link global pose
        """
        return self.__robot_name2link[name].get_global_pose()

    def get_robot_link_local_pose_by_name(self, name: str) -> sapyen.Pose:
        """
        :param name: link_name
        :return: Link local pose
        """
        robot_pose: sapyen.Pose = self.__robot_name2link[self._base_link_name].get_global_pose()
        link_pose = self.__robot_name2link[name].get_global_pose()
        return robot_pose.inv().transform(link_pose)

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
        return self.robot_name2link(self._base_link_name).get_global_pose()

    def set_robot_base_pose(self, pose: Union[np.ndarray, List]) -> None:
        assert len(pose) == 7, "Pose should be in Position: x y z, Quaternion: w x y z format"
        self.manger.move_base(pose[0:3], pose[3:7])
