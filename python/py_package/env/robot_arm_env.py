from .base_env import BaseEnv, pysapien
import os
from .path_utils import get_assets_path
from typing import List
import numpy as np
from gym import utils, spaces
import warnings


def render_error():
    raise RuntimeError("On Screen Render was set to false, no renderer.")


class RobotArmEnv(BaseEnv):
    def __init__(self, sim):
        BaseEnv.__init__(self, sim)
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

        # Setup Robot
        movo_material = self.sim.create_physical_material(3.0, 2.0, 0.01)
        self._load_robot(os.path.join(get_assets_path(), "robot/all_robot.urdf"), movo_material)
        self._actuator_index = np.array([], dtype=np.int)
        self._actuator_range = np.zeros([0, 2])

        # Setup actuator
        for qname in self._q_names:
            self.add_force_actuator(qname, -50, 50)
        self.action_space = spaces.Box(low=self._actuator_range[:, 0], high=self._actuator_range[:, 1],
                                       dtype=np.float32)

        # Setup Simulator
        self._frame_skip = 1
        self._init_qpos = [0.25, -1.381, 0, 0.05, -0.9512, 0.387, 0.608, 2.486, 1.05, -1.16, 0, 0, 0]
        self.robot.set_qpos(self._init_qpos)
        self.scene.step()
        self.__init_cache = np.stack(
            [self.robot.get_qpos(), self.robot.get_qvel(), self.robot.get_qacc(), self.robot.get_qf()], axis=0)

    def _load_robot(self, urdf_path: str, material: pysapien.PxMaterial) -> None:
        # By default, the robot will loaded with balanced passive force
        self.loader.fix_base = True
        self.robot: pysapien.Articulation = self.loader.load(urdf_path, material)
        self.robot.set_root_pose(pysapien.Pose([0, 0, 0], [1, 0, 0, 0]))

        # Link mapping, remember to set the self._base_link_name if your robot base is not that name
        self._q_names = [j.name for j in self.robot.get_joints() if j.get_dof()]
        self._base_link_name = "base_link"

        # Set mapping and other staff for the camera loaded with robot urdf
        self._init_camera_cache()

    def do_simulation(self, a: np.ndarray):
        for i in range(self._frame_skip):
            self.apply_actuator(a)
            self._step()

    def reset(self):
        for i in range(2):
            self.robot.set_qpos(self.__init_cache[0, :])
            self.robot.set_qvel(self.__init_cache[1, :])
            self.robot.set_qacc(self.__init_cache[2, :])
            self.robot.set_qf(self.__init_cache[3, :])

    def get_observation(self):
        return np.concatenate([self.robot.get_qpos(), self.robot.get_qvel()])

    def step(self, a: np.ndarray):
        self.do_simulation(a)
        return self.get_observation()

    def add_force_actuator(self, joint_name: str, low: float, high: float):
        # TODO: support ball joint
        if joint_name not in self._q_names:
            warnings.warn("Joint name {} is not a valid joint for robot.".format(joint_name))
            return
        joint_index = self._q_names.index(joint_name)
        if joint_index in self._actuator_index:
            warnings.warn("Joint with Index {} have already been added an actuator".format(joint_index))
            return
        self._actuator_index = np.append(self._actuator_index, [joint_index])
        self._actuator_range = np.append(self._actuator_range, np.array([[low, high]]), axis=0)

    def apply_actuator(self, a: np.ndarray):
        assert len(a) == len(self._actuator_index), "Action dimension must equal to the number of actuator"
        qf = np.zeros(len(self._q_names))
        qf[self._actuator_index] = a
        self.robot.set_qf(qf)
