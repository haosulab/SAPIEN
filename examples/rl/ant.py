"""Ant environment.

Notes:
    It is not a full reproduction of Mujoco-based ant.

References:
    https://github.com/openai/gym/blob/master/gym/envs/mujoco/ant.py
"""

import numpy as np
from transforms3d.quaternions import axangle2quat
from gym import spaces

import sapien.core as sapien
from sapien.core import Pose
from sapien.utils.viewer import Viewer
from sapien_env import SapienEnv


class AntEnv(SapienEnv):
    def __init__(self):
        super().__init__(control_freq=5, timestep=0.01)

        self.actuator = self.get_articulation('ant')
        self._scene.step()  # simulate one step for steady state
        self._init_state = self.actuator.pack()

        dof = self.actuator.dof
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=[5 + dof + 6 + dof], dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=[dof], dtype=np.float32)
        # Following the original implementation, we scale the action (qf)
        self._action_scale_factor = 50.0

    # ---------------------------------------------------------------------------- #
    # Simulation world
    # ---------------------------------------------------------------------------- #
    def _build_world(self):
        physical_material = self._scene.create_physical_material(1.0, 1.0, 0.0)
        self._scene.default_physical_material = physical_material
        render_material = self._renderer.create_material()
        render_material.set_base_color([0.8, 0.9, 0.8, 1])
        self._scene.add_ground(0.0, render_material=render_material)
        ant = self.create_ant(self._scene)
        ant.set_pose(Pose([0., 0., 0.55]))

    @staticmethod
    def create_ant(scene: sapien.Scene, color=(0.8, 0.6, 0.4), 
                   friction=0.0, damping=1.0, density=20.0):
        builder = scene.create_articulation_builder()

        # torso
        torso = builder.create_link_builder()
        torso.set_name('torso')
        torso.add_sphere_collision(Pose(), 0.25, density=density)
        torso.add_sphere_visual(Pose(), 0.25, color=color)
        torso.add_capsule_collision(Pose([0.141, 0, 0]), 0.08, 0.141, density=density)
        torso.add_capsule_visual(Pose([0.141, 0, 0]), 0.08, 0.141, color=color)
        torso.add_capsule_collision(Pose([-0.141, 0, 0]), 0.08, 0.141, density=density)
        torso.add_capsule_visual(Pose([-0.141, 0, 0]), 0.08, 0.141, color=color)
        rot_z90 = axangle2quat([0, 0, 1], np.pi / 2)
        torso.add_capsule_collision(Pose([0, 0.141, 0], rot_z90), 0.08, 0.141, density=density)
        torso.add_capsule_visual(Pose([0, 0.141, 0], rot_z90), 0.08, 0.141, color=color)
        torso.add_capsule_collision(Pose([0, -0.141, 0], rot_z90), 0.08, 0.141, density=density)
        torso.add_capsule_visual(Pose([0, -0.141, 0], rot_z90), 0.08, 0.141, color=color)

        thigh_1 = builder.create_link_builder(torso)
        thigh_1.set_name('thigh_1')
        thigh_1.add_capsule_collision(Pose(), 0.08, 0.141, density=density)
        thigh_1.add_capsule_visual(Pose(), 0.08, 0.141, color=color)
        thigh_1.set_joint_name('hip_1')
        thigh_1.set_joint_properties(
            'revolute',
            [[-0.5236, 0.5236]],
            Pose([0.282, 0, 0], [0.7071068, 0, 0.7071068, 0]),
            Pose([0.141, 0, 0], [-0.7071068, 0, 0.7071068, 0]),
            friction=friction, damping=damping,
        )

        thigh_2 = builder.create_link_builder(torso)
        thigh_2.set_name("thigh_2")
        thigh_2.set_joint_name("hip_2")
        thigh_2.set_joint_properties(
            'revolute',
            [[-0.5236, 0.5236]],
            Pose([-0.282, 0, 0], [0, -0.7071068, 0, 0.7071068]),
            Pose([0.141, 0, 0], [-0.7071068, 0, 0.7071068, 0]),
            friction=friction, damping=damping,
        )
        thigh_2.add_capsule_collision(Pose(), 0.08, 0.141, density=density)
        thigh_2.add_capsule_visual(Pose(), 0.08, 0.141, color=color)

        thigh_3 = builder.create_link_builder(torso)
        thigh_3.set_name("thigh_3")
        thigh_3.set_joint_name("hip_3")
        thigh_3.set_joint_properties(
            'revolute',
            [[-0.5236, 0.5236]],
            Pose([0, 0.282, 0], [0.5, -0.5, 0.5, 0.5]),
            Pose([0.141, 0, 0], [0.7071068, 0, -0.7071068, 0]),
            friction=friction, damping=damping,
        )
        thigh_3.add_capsule_collision(Pose(), 0.08, 0.141, density=density)
        thigh_3.add_capsule_visual(Pose(), 0.08, 0.141, color=color)

        thigh_4 = builder.create_link_builder(torso)
        thigh_4.set_name("thigh_4")
        thigh_4.set_joint_name("hip_4")
        thigh_4.set_joint_properties(
            'revolute',
            [[-0.5236, 0.5236]],
            Pose([0, -0.282, 0], [0.5, 0.5, 0.5, -0.5]),
            Pose([0.141, 0, 0], [0.7071068, 0, -0.7071068, 0]),
            friction=friction, damping=damping,
        )
        thigh_4.add_capsule_collision(Pose(), 0.08, 0.141, density=density)
        thigh_4.add_capsule_visual(Pose(), 0.08, 0.141, color=color)

        calf_1 = builder.create_link_builder(thigh_1)
        calf_1.set_name("calf_1")
        calf_1.set_joint_name("knee_1")
        calf_1.set_joint_properties(
            'revolute',
            [[0.5236, 1.222]],
            Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
            Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]),
            friction=friction, damping=damping,
        )
        calf_1.add_capsule_collision(Pose(), 0.08, 0.282, density=density)
        calf_1.add_capsule_visual(Pose(), 0.08, 0.282, color=color)

        calf_2 = builder.create_link_builder(thigh_2)
        calf_2.set_name("calf_2")
        calf_2.set_joint_name("knee_2")
        calf_2.set_joint_properties(
            'revolute',
            [[0.5236, 1.222]],
            Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
            Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]),
            friction=friction, damping=damping,
        )
        calf_2.add_capsule_collision(Pose(), 0.08, 0.282, density=density)
        calf_2.add_capsule_visual(Pose(), 0.08, 0.282, color=color)

        calf_3 = builder.create_link_builder(thigh_3)
        calf_3.set_name("calf_3")
        calf_3.set_joint_name("knee_3")
        calf_3.set_joint_properties(
            'revolute',
            [[0.5236, 1.222]],
            Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
            Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]),
            friction=friction, damping=damping,
        )
        calf_3.add_capsule_collision(Pose(), 0.08, 0.282, density=density)
        calf_3.add_capsule_visual(Pose(), 0.08, 0.282, color=color)

        calf_4 = builder.create_link_builder(thigh_4)
        calf_4.set_name("calf_4")
        calf_4.set_joint_name("knee_4")
        calf_4.set_joint_properties(
            'revolute',
            [[0.5236, 1.222]],
            Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
            Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]),
            friction=friction, damping=damping,
        )
        calf_4.add_capsule_collision(Pose(), 0.08, 0.282, density=density)
        calf_4.add_capsule_visual(Pose(), 0.08, 0.282, color=color)

        ant = builder.build(fix_root_link=False)
        ant.set_name('ant')

        return ant

    # ---------------------------------------------------------------------------- #
    # RL
    # ---------------------------------------------------------------------------- #
    def step(self, action):
        ant = self.actuator

        x_before = ant.pose.p[0]
        ant.set_qf(action * self._action_scale_factor)
        for i in range(self.control_freq):
            self._scene.step()
        x_after = ant.pose.p[0]

        forward_reward = (x_after - x_before) / self.dt
        ctrl_cost = 0.5 * np.square(action).sum()
        survive_reward = 1.0
        # Note that we do not include contact cost as the original version
        reward = forward_reward - ctrl_cost + survive_reward

        state = self.state_vector()
        is_healthy = (np.isfinite(state).all() and 0.2 <= state[2] <= 1.0)
        done = not is_healthy

        obs = self._get_obs()

        return obs, reward, done, dict(
            reward_forward=forward_reward,
            reward_ctrl=-ctrl_cost,
            reward_survive=survive_reward)

    def reset(self):
        self.actuator.unpack(self._init_state)
        # add some random noise
        init_qpos = self.actuator.get_qpos()
        init_qvel = self.actuator.get_qvel()
        qpos = init_qpos + self.np_random.uniform(size=self.actuator.dof, low=-0.1, high=0.1)
        qvel = init_qvel + self.np_random.normal(size=self.actuator.dof) * 0.1
        self.actuator.set_qpos(qpos)
        self.actuator.set_qvel(qvel)
        obs = self._get_obs()
        return obs

    def _get_obs(self):
        state = self.state_vector()
        obs = state[2:]  # exclude current xy position
        return obs

    def state_vector(self):
        ant = self.actuator
        torso = ant.get_links()[0]

        pos = torso.pose.p
        quat = torso.pose.q
        vel = torso.get_velocity()
        ang_vel = torso.get_angular_velocity()

        qpos = ant.get_qpos()
        qvel = ant.get_qvel()

        return np.hstack([pos, quat, qpos, vel, ang_vel, qvel])

    # ---------------------------------------------------------------------------- #
    # Visualization
    # ---------------------------------------------------------------------------- #
    def _setup_lighting(self):

        self._scene.set_ambient_light([.4, .4, .4])
        self._scene.add_point_light([2, 2, 2], [1, 1, 1])
        self._scene.add_point_light([2, -2, 2], [1, 1, 1])
        self._scene.add_point_light([-2, 0, 2], [1, 1, 1])

    def _setup_viewer(self):
        self._setup_lighting()
        self.viewer = Viewer(self._renderer)
        self.viewer.set_scene(self._scene)
        self.viewer.set_camera_xyz(x=0, y=-3, z=0.3)
        self.viewer.set_camera_rpy(y=1.57, p=0.0, r=0)
        self.viewer.focus_entity(self.actuator)


def main():
    env = AntEnv()
    env.reset()
    for step in range(1000):
        env.render()
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        if done:
            print(f'Done at step {step}')
            obs = env.reset()
    env.close()


if __name__ == '__main__':
    main()
