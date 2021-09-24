"""Lift environment."""

import numpy as np
from gym import spaces

import sapien.core as sapien
from sapien.core import Pose
from sapien.utils.viewer import Viewer
from sapien_env import SapienEnv


class LiftEnv(SapienEnv):
    def __init__(self):
        self.init_qpos = [0, 0.19634954084936207, 0.0, -2.617993877991494,
                          0.0, 2.941592653589793, 0.7853981633974483, 0, 0]
        self.table_height = 0.8
        super().__init__(control_freq=20, timestep=0.01)

        self.robot = self.get_articulation('panda')
        self.end_effector = self.robot.get_links()[8]
        self.dof = self.robot.dof
        assert self.dof == 9, 'Panda should have 9 DoF'
        self.active_joints = self.robot.get_active_joints()
        self.cube = self.get_actor('cube')

        # The arm is controlled by the internal velocity drive
        for joint in self.active_joints[:5]:
            joint.set_drive_property(stiffness=0, damping=4.8)
        for joint in self.active_joints[5:7]:
            joint.set_drive_property(stiffness=0, damping=0.72)
        # The gripper will be controlled directly by the torque

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=[self.dof * 2 + 13], dtype=np.float32)
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=[self.dof], dtype=np.float32)

    # ---------------------------------------------------------------------------- #
    # Simulation world
    # ---------------------------------------------------------------------------- #
    def _build_world(self):
        physical_material = self._scene.create_physical_material(1.0, 1.0, 0.0)
        self._scene.default_physical_material = physical_material
        self._scene.add_ground(0.0)

        # table top
        builder = self._scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.4, 0.4, 0.025])
        builder.add_box_visual(half_size=[0.4, 0.4, 0.025])
        table = builder.build_kinematic(name='table')
        table.set_pose(Pose([0, 0, self.table_height - 0.025]))

        # cube
        builder = self._scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.02])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.02], color=[1, 0, 0])
        cube = builder.build(name='cube')
        cube.set_pose(Pose([0, 0, self.table_height + 0.02]))

        # robot
        loader = self._scene.create_urdf_loader()
        loader.fix_root_link = True
        robot = loader.load('../assets/robot/panda/panda.urdf')
        robot.set_name('panda')
        robot.set_root_pose(Pose([-0.16 - 0.4, 0, self.table_height]))
        robot.set_qpos(self.init_qpos)

    # ---------------------------------------------------------------------------- #
    # RL
    # ---------------------------------------------------------------------------- #
    def step(self, action):
        # Use internal velocity drive
        for idx in range(7):
            self.active_joints[idx].set_drive_velocity_target(action[idx])

        # Control the gripper directly by torque
        qf = self.robot.compute_passive_force(True, True, False)
        qf[-2:] += action[-2:]
        self.robot.set_qf(qf)

        for i in range(self.control_freq):
            self._scene.step()

        obs = self._get_obs()
        reward = self._get_reward()
        done = self.cube.get_pose().p[2] > self.table_height + 0.04
        if done:
            reward += 100.0

        return obs, reward, done, {}

    def reset(self):
        self.robot.set_qpos(self.init_qpos)
        self.cube.set_pose(Pose(
            [np.random.randn() * 0.05, np.random.randn() * 0.05, self.table_height + 0.02]))
        self._scene.step()
        return self._get_obs()

    def _get_obs(self):
        qpos = self.robot.get_qpos()
        qvel = self.robot.get_qvel()
        cube_pose = self.cube.get_pose()
        ee_pose = self.end_effector.get_pose()
        cube_to_ee = ee_pose.p - cube_pose.p
        return np.hstack([qpos, qvel, cube_pose.p, cube_pose.q, cube_to_ee])

    def _get_reward(self):
        # reaching reward
        cube_pose = self.cube.get_pose()
        ee_pose = self.end_effector.get_pose()
        distance = np.linalg.norm(ee_pose.p - cube_pose.p)
        reaching_reward = 1 - np.tanh(10.0 * distance)

        # lifting reward
        lifting_reward = max(
            0, self.cube.pose.p[2] - self.table_height - 0.02) / 0.02

        return reaching_reward + lifting_reward

    # ---------------------------------------------------------------------------- #
    # Visualization
    # ---------------------------------------------------------------------------- #
    def _setup_lighting(self):

        self._scene.set_ambient_light([.4, .4, .4])
        self._scene.add_directional_light([1, -1, -1], [0.3, 0.3, 0.3])
        self._scene.add_directional_light([0, 0, -1], [1, 1, 1])

    def _setup_viewer(self):
        self._setup_lighting()
        self.viewer = Viewer(self._renderer)
        self.viewer.set_scene(self._scene)
        self.viewer.set_camera_xyz(x=1.5, y=0.0, z=2.0)
        self.viewer.set_camera_rpy(y=3.14, p=-0.5, r=0)


def main():
    env = LiftEnv()
    env.reset()
    for episode in range(10):
        for step in range(100):
            env.render()
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
            env.step(action)
            if done:
                print(f'Done at step {step}')
                break
        obs = env.reset()
    env.close()


if __name__ == '__main__':
    main()
