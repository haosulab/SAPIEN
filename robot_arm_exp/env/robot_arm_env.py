from .base_env import BaseEnv, sapyen
import os
from .path_utils import get_assets_path
from typing import List
import numpy as np
from gym import utils, spaces


def render_error():
    raise RuntimeError("On Screen Render was set to false, no renderer.")


class RobotArmEnv(BaseEnv):
    def __init__(self, on_screen_render: bool = True):
        BaseEnv.__init__(self, on_screen_render)
        if on_screen_render:
            self.render = lambda: self.renderer.render
        else:
            self.render = render_error

            # Setup Robot
        movo_material = self.sim.create_material(3.0, 2.0, 0.01)
        self._load_robot(os.path.join(get_assets_path(), "robot/all_robot.urdf"), movo_material)
        for q_name in self._q_names:
            self.robot.add_force_actuator(q_name, -200, 200)
        bounds = self.robot.get_force_actuator_range()
        low = bounds[:, 0]
        high = bounds[:, 1]
        self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)

        # Setup Simulator
        self._frame_skip = 1
        self._init_qpos = [0.25, -1.381, 0, 0.05, -0.9512, 0.387, 0.608, 2.486, 1.05, -1.16, 0, 0, 0]
        self.robot.set_qpos(self._init_qpos)
        self.sim.step()
        self.__init_cache = self.sim.dump()

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
        self._q_names = self.robot.get_qnames()
        self._base_link_name = "base_link"

        # Set mapping and other staff for the camera loaded with robot urdf
        self._init_camera_cache()

    def do_simulation(self, a: np.ndarray):
        for i in range(self._frame_skip):
            self.robot.apply_actuator(a)
            self.sim.step()

    def reset(self):
        for i in range(3):
            self.sim.pack(self.__init_cache)

    def get_observation(self):
        return np.concatenate([self.robot.get_qpos(), self.robot.get_qvel()])

    def step(self, a: np.ndarray):
        self.do_simulation(a)
        return self.get_observation()
