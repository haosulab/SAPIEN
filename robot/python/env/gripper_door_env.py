import sapyen_robot
import sapyen
from .single_gripper_env import SingleGripperBaseEnv
from .base_env import SapienSingleObjectEnv
from gym import utils, spaces

from typing import List, Union
import numpy as np
import os
from .path_utils import get_assets_path

class DoorEnv(SingleGripperBaseEnv, SapienSingleObjectEnv):
    def __init__(self, dataset_dir: str, data_id: Union[int, str], on_screening_rendering: bool, initial_state: np.array):
        SapienSingleObjectEnv.__init__(self, dataset_dir, data_id, on_screening_rendering)
        self._init_robot()
        print('here!!!!!!', self.object, self.robot)
        self.initial_state = initial_state
        low = np.array([-1., -1., -1., -3.14, -3.14, -3.14])
        high = -1 * low
        self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
        #print('action space', self.action_space.low, self.action_space.high)
        print('here!!!!!!', self.object, self.robot)
        self.reset()
        self.frame_skip = 5
        self.simulation_count = 0

    def reset(self):
        self.simulation_count = 0
        self.object.set_qf(np.ones(self.object.dof()) * -1)
        self.sim.pack(self.initial_state)
        # frame skip
        for i in range(self.frame_skip):
            self.sim.step()
        return self._get_obs()

    def _get_obs(self):
        return np.concatenate([self.robot.get_qpos().flat,
                               self.robot.get_qvel().flat,
                               self.object.get_qpos().flat,
                               self.object.get_qvel().flat])

    def judge_close(self):
        qlimits = self.robot.get_qlimits()[-3:]
        print('qlimits for gripper', qlimits)
        current_pos = self.robot.get_qpos()[-3:]
        relative_pos = np.array([current_pos[0]-current_pos[1], current_pos[1]-current_pos[2],
                                 current_pos[2]-current_pos[0]])
        relative_pos_norm = np.linalg.norm(relative_pos)
        if relative_pos_norm < 0.5 and np.linalg.norm(qlimits-current_pos) < 1:
            return True
        else:
            return False


    def step(self, action):
        self.simulation_count += 1
        self.object.set_qf(np.ones(self.object.dof()) * -1)
        self.close_gripper(3)
        joint_before = self.object.get_qpos()
        pos_before = self.robot.get_qpos()
        pos_after = pos_before.copy()
        pos_after[:6] = pos_before[:6] + action
        print('pos before, pos after', pos_before, pos_after, action)

        self.robot.set_drive_qpos(pos_after)
        for i in range(self.frame_skip):
            self.sim.step()

        next_state = self._get_obs()
        joint_after = self.object.get_qpos()
        off_object = self.judge_close()
        done = off_object
        if off_object:
            reward = -10
        else:
            reward = (joint_after-joint_before).sum()

        if self.simulation_count == 500:
            done = True
        return next_state, reward, done, None


