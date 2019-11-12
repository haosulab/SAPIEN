import sapyen_robot
import sapyen
from .single_gripper_env import SingleGripperBaseEnv
from .base_env import SapienSingleObjectEnv
from gym import utils, spaces

from typing import List, Union
import numpy as np
import transforms3d
import os
from .path_utils import get_assets_path


class DoorEnv(SingleGripperBaseEnv, SapienSingleObjectEnv):
    def __init__(self, dataset_dir: str, data_id: Union[int, str], on_screening_rendering: bool,
                 initial_state: np.array, target_link_index: int):
        SapienSingleObjectEnv.__init__(self, dataset_dir, data_id, on_screening_rendering)
        self._init_robot()
        self.initial_state = initial_state
        self.target_link_index = target_link_index
        action_low = np.array([-2., -2., -2., -3.14, -3.14, -3.14])
        action_high = -1 * action_low
        high = np.array([
            np.finfo(np.float32).max,
            np.finfo(np.float32).max])
        self.observation_space = spaces.Box()
        self.action_space = spaces.Box(low=action_low, high=action_high, dtype=np.float32)
        self.frame_skip = 20
        self.simulation_count = 0
        self._max_episode_steps = 150
        self.reset()

        pose = np.eye(4)
        pose[0:3, 3] = [3, -3, 2.5]
        pose[:3, :3] = transforms3d.euler.euler2mat(0, 5, 1.507)
        self.add_camera("right_view", pose, 640, 480)

    def reset(self):
        self.simulation_count = 0
        self.sim.pack(self.initial_state)
        self.sim.clear()
        # print('reset before', self.robot.get_qpos())
        self.robot.set_drive_qpos(self.robot.get_qpos())

        # frame skip
        for i in range(self.frame_skip):
            self.object.set_qf(np.ones(self.object.dof()) * -1)
            self.gripper_controller.move_joint(self._gripper_joint, 5)
            self._step()
        # print('reset after', self.robot.get_qpos())
        return self._get_obs()

    def _get_obs(self):
        self.object.get_link_joint_pose()
        return np.concatenate([self.robot.get_qpos().flat,
                               self.robot.get_qvel().flat,
                               self.object.get_qpos().flat,
                               self.object.get_qvel().flat])

    def judge_close(self):
        qlimits = self.robot.get_qlimits()[-3:, 1]
        current_pos = self.robot.get_qpos()[-3:]
        relative_pos = np.array([current_pos[0] - current_pos[1], current_pos[1] - current_pos[2],
                                 current_pos[2] - current_pos[0]])
        relative_pos_norm = np.linalg.norm(relative_pos)
        if relative_pos_norm < 0.1 and np.linalg.norm(qlimits - current_pos) < 0.1:
            return True
        else:
            return False

    def step(self, action):
        self.simulation_count += 1
        # print('action', action)
        joint_before = self.object.get_qpos()
        pos_before = self.robot.get_qpos()
        pos_after = pos_before.copy()
        pos_after[:6] = pos_before[:6] + action
        # print('pos before, pos after', pos_before, pos_after, action, self.simulation_count)

        for i in range(self.frame_skip):
            self.object.set_qf(np.ones(self.object.dof()) * -1)
            self.translation_controller.move_joint(self._translation_joint[0:1], action[0])
            self.translation_controller.move_joint(self._translation_joint[1:2], action[1])
            self.translation_controller.move_joint(self._translation_joint[2:3], action[2])
            self.rotation_controller.move_joint(self._rotation_joint[3:4], action[3])
            self.rotation_controller.move_joint(self._rotation_joint[4:5], action[4])
            self.rotation_controller.move_joint(self._rotation_joint[5:6], action[5])
            self.gripper_controller.move_joint(self._gripper_joint, 5)
            self._step()

        next_state = self._get_obs()
        joint_after = self.object.get_qpos()
        off_object = self.judge_close()
        # print('in step', off_object)
        done = off_object
        if off_object:
            reward = -2
        else:
            reward = 100 * (joint_after - joint_before).sum()

        if self.simulation_count == self._max_episode_steps:
            done = True
        return next_state, reward, done, None

    def __get_normal_estimation_fn(self):
        camera_pose = self.calculate_pose_in_front_of_object_link(self.target_link_index, category=None,
                                                                  horizontal_offset=1.5, vertical_offset=0)
        self.add_camera("front_view", camera_pose, width=640, height=480)
        self.sim.step()
        mean_normal = self.get_mean_normal_with_seg_id(0, self.object_link_segmentation_ids[
            self.target_link_index]).reshape(3, 1)

        joint_index = self.object.get_link_joint_indices()[self.target_link_index]
        axis_pose = self.object.get_link_joint_pose(self.target_link_index)
        axis_orientation = transforms3d.quaternions.quat2mat(axis_pose.q)
        joint_index = int(np.sum(self.object.get_joint_dofs()[0:joint_index + 1]) - 1)
        init_qpos = self.object.get_qpos()[joint_index]
        local_mean_normal = np.linalg.inv(axis_orientation) @ mean_normal

        def get_target_normal_fn():
            qpos = self.object.get_qpos()[joint_index]
            relative_transform = transforms3d.euler.euler2mat(qpos - init_qpos, 0, 0)
            return axis_orientation @ relative_transform @ local_mean_normal

        return get_target_normal_fn
