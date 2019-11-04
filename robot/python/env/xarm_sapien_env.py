from .xarm_env import XArmEnv
from .base_env import SapienSingleObjectEnv
import sapyen_robot
from typing import List, Union


class XarmSapienEnv(XArmEnv, SapienSingleObjectEnv):
    def __init__(self, dataset_dir: str, data_id: Union[str, int], on_screening_rendering: bool):
        SapienSingleObjectEnv.__init__(self, dataset_dir, data_id, on_screening_rendering)
        self._init_robot()

        # Init
        self.dump_data = []
        self.control_signal = []
        self.object_force_array = []
        self.robot_force_array = []

    def step(self):
        self._step()

    def generate_header(self):
        header = {}
        header.update({"robot_joint_name": self.robot.get_joint_names()})
        header.update({"robot_link_name": self.robot.get_link_names()})
        header.update({"object_joint_name": self.object.get_joint_names()})
        header.update({"object_link_name": self.object.get_link_names()})
        return header


class XArmRecorder(XArmEnv, SapienSingleObjectEnv):
    def __init__(self, dataset_dir: str, data_id: Union[str, int], on_screening_rendering: bool):
        SapienSingleObjectEnv.__init__(self, dataset_dir, data_id, on_screening_rendering)
        self._init_robot()

        # Init
        self.dump_data = []
        self.control_signal = []
        self.object_force_array = []
        self.robot_force_array = []

    def step(self):
        self._step()

    def generate_header(self):
        header = {}
        header.update({"robot_joint_name": self.robot.get_joint_names()})
        header.update({"robot_link_name": self.robot.get_link_names()})
        header.update({"object_joint_name": self.object.get_joint_names()})
        header.update({"object_link_name": self.object.get_link_names()})
        return header
