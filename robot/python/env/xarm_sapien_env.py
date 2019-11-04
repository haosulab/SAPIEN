from .xarm_env import XArmEnv
from .base_env import SapienSingleObjectEnv
import sapyen_robot
from typing import List, Union
import numpy as np


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


class XArmRecorder(XArmEnv, SapienSingleObjectEnv):
    def __init__(self, dataset_dir: str, data_id: Union[str, int], on_screening_rendering: bool):
        SapienSingleObjectEnv.__init__(self, dataset_dir, data_id, on_screening_rendering)
        self._init_robot()

        wrapper = self.sim.create_controllable_articulation(self.robot)
        self.manger = sapyen_robot.ControllerManger("xarm6", wrapper)
        self.ps3 = sapyen_robot.XArm6PS3(self.manger)
        self.ps3.set_demonstration_mode()

        # Cache robot pose
        self.root_theta = 0
        self.root_pos = np.array([0, 0], dtype=np.float)
        self.init_qpos = np.zeros(12)

        # Tune PD controller
        self.robot.set_pd(2000, 300, 300, np.arange(6))
        self.robot.set_pd(500, 100, 300, np.arange(6, 12))
        self.robot.set_drive_qpos(self.init_qpos)
        self.robot.set_qpos(self.init_qpos)
        self.sim.step()
        # Init
        self.dump_data = []
        self.control_signal = []
        self.object_force_array = []
        self.robot_force_array = []

    def step(self):
        self._step()
        self.ps3.step()

        # Cache
        if self.ps3.start_record():
            print("Recording")
            self.control_signal.append(self.ps3.get_cache())
            self.dump_data.append(self.sim.dump())
            self.object_force_array.append(self.object.get_cfrc_ext())
            self.robot_force_array.append(self.robot.get_cfrc_ext())

    def generate_header(self):
        header = {}
        header.update({"robot_joint_name": self.robot.get_joint_names()})
        header.update({"robot_link_name": self.robot.get_link_names()})
        header.update({"object_joint_name": self.object.get_joint_names()})
        header.update({"object_link_name": self.object.get_link_names()})
        return header
