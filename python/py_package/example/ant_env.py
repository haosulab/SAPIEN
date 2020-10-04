import sapien.core as sapien
from sapien.core import Pose
import numpy as np
from typing import List, Tuple


def create_ant_builder(scene):
    from transforms3d.quaternions import axangle2quat as aa
    builder = scene.create_articulation_builder()
    body = builder.create_link_builder()
    body.add_sphere_shape(Pose(), 0.25)
    body.add_sphere_visual(Pose(), 0.25)
    body.add_capsule_shape(Pose([0.141, 0, 0]), 0.08, 0.141)
    body.add_capsule_visual(Pose([0.141, 0, 0]), 0.08, 0.141)
    body.add_capsule_shape(Pose([-0.141, 0, 0]), 0.08, 0.141)
    body.add_capsule_visual(Pose([-0.141, 0, 0]), 0.08, 0.141)
    body.add_capsule_shape(Pose([0, 0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141)
    body.add_capsule_visual(Pose([0, 0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141)
    body.add_capsule_shape(Pose([0, -0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141)
    body.add_capsule_visual(Pose([0, -0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141)
    body.set_name("body")

    l1 = builder.create_link_builder(body)
    l1.set_name("l1")
    l1.set_joint_name("hip_1")
    l1.set_joint_properties(sapien.ArticulationJointType.REVOLUTE, [[-0.5236, 0.5236]],
                            Pose([0.282, 0, 0], [0.7071068, 0, 0.7071068, 0]),
                            Pose([0.141, 0, 0], [-0.7071068, 0, 0.7071068, 0]), 0.1)
    l1.add_capsule_shape(Pose(), 0.08, 0.141)
    l1.add_capsule_visual(Pose(), 0.08, 0.141)

    l2 = builder.create_link_builder(body)
    l2.set_name("l2")
    l2.set_joint_name("hip_2")
    l2.set_joint_properties(sapien.ArticulationJointType.REVOLUTE, [[-0.5236, 0.5236]],
                            Pose([-0.282, 0, 0], [0, -0.7071068, 0, 0.7071068]),
                            Pose([0.141, 0, 0], [-0.7071068, 0, 0.7071068, 0]), 0.1)
    l2.add_capsule_shape(Pose(), 0.08, 0.141)
    l2.add_capsule_visual(Pose(), 0.08, 0.141)

    l3 = builder.create_link_builder(body)
    l3.set_name("l3")
    l3.set_joint_name("hip_3")
    l3.set_joint_properties(sapien.ArticulationJointType.REVOLUTE, [[-0.5236, 0.5236]],
                            Pose([0, 0.282, 0], [0.5, -0.5, 0.5, 0.5]),
                            Pose([0.141, 0, 0], [0.7071068, 0, -0.7071068, 0]), 0.1)
    l3.add_capsule_shape(Pose(), 0.08, 0.141)
    l3.add_capsule_visual(Pose(), 0.08, 0.141)

    l4 = builder.create_link_builder(body)
    l4.set_name("l4")
    l4.set_joint_name("hip_4")
    l4.set_joint_properties(sapien.ArticulationJointType.REVOLUTE, [[-0.5236, 0.5236]],
                            Pose([0, -0.282, 0], [0.5, 0.5, 0.5, -0.5]),
                            Pose([0.141, 0, 0], [0.7071068, 0, -0.7071068, 0]), 0.1)
    l4.add_capsule_shape(Pose(), 0.08, 0.141)
    l4.add_capsule_visual(Pose(), 0.08, 0.141)

    f1 = builder.create_link_builder(l1)
    f1.set_name("f1")
    f1.set_joint_name("ankle_1")
    f1.set_joint_properties(sapien.ArticulationJointType.REVOLUTE, [[0.5236, 1.222]],
                            Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
                            Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]), 0.1)
    f1.add_capsule_shape(Pose(), 0.08, 0.282)
    f1.add_capsule_visual(Pose(), 0.08, 0.282)

    f2 = builder.create_link_builder(l2)
    f2.set_name("f2")
    f2.set_joint_name("ankle_2")
    f2.set_joint_properties(sapien.ArticulationJointType.REVOLUTE, [[0.5236, 1.222]],
                            Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
                            Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]), 0.1)
    f2.add_capsule_shape(Pose(), 0.08, 0.282)
    f2.add_capsule_visual(Pose(), 0.08, 0.282)

    f3 = builder.create_link_builder(l3)
    f3.set_name("f3")
    f3.set_joint_name("ankle_3")
    f3.set_joint_properties(sapien.ArticulationJointType.REVOLUTE, [[0.5236, 1.222]],
                            Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
                            Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]), 0.1)
    f3.add_capsule_shape(Pose(), 0.08, 0.282)
    f3.add_capsule_visual(Pose(), 0.08, 0.282)

    f4 = builder.create_link_builder(l4)
    f4.set_name("f4")
    f4.set_joint_name("ankle_4")
    f4.set_joint_properties(sapien.ArticulationJointType.REVOLUTE, [[0.5236, 1.222]],
                            Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
                            Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]), 0.1)
    f4.add_capsule_shape(Pose(), 0.08, 0.282)
    f4.add_capsule_visual(Pose(), 0.08, 0.282)

    return builder


class SapienControlEnv:
    def __init__(self, frame_skip, timestep=0.01, gravity=(0, 0, -9.8)):
        self.frame_skip = frame_skip
        self.timestep = timestep

        self._engine = sapien.Engine()
        self._renderer = sapien.VulkanRenderer()
        self._engine.set_renderer(self._renderer)
        self.sim = self._engine.create_scene()
        self.sim.set_timestep(timestep)
        self.viewer = self.viewer_setup()

        self.model, self._init_state = self._load_model()
        self._dof = self.model.dof
        self._root = self.model.get_links()[0]

        # Unlike MuJoCo MJCF, the actuator information is not defined in the xml file
        # So you should define it explicitly in Python
        from gym.spaces.box import Box
        actuator_info = self._load_actuator()
        self._actuator_name, _actuator_idx, low, high = list(zip(*actuator_info))
        self._actuator_idx = np.array(_actuator_idx)
        self.action_spec = Box(low=np.array(low), high=np.array(high), dtype=np.float32)

    def do_simulation(self, ctrl, n_frames):
        qf = np.zeros(self._dof, np.float32)
        qf[self._actuator_idx] = ctrl
        for _ in range(n_frames):
            self.model.set_qf(qf)
            self.sim.step()

    def render(self):
        self.sim.update_render()
        self.viewer.render()

    def close(self):
        self.viewer = None
        self.sim = None
        self._renderer = None

    def get_qpos(self):
        if not self._root:
            root = [self._root.pose.p, self._root.pose.q]
        else:
            root = []
        return np.concatenate(root + [self.model.get_qpos().ravel()])

    def get_qvel(self):
        if not self._root:
            root = {self._root.velocity, self._root.angular_velocity}
        else:
            root = []
        return np.concatenate(root + [self.model.get_qvel().ravel()])

    def state_vector(self):
        return np.concatenate([
            self.get_qpos(),
            self.get_qvel()
        ])

    @property
    def dt(self):
        return self.timestep * self.frame_skip

    def reset(self) -> np.ndarray:
        ob = self.reset_model()
        return ob

    def _load_model(self) -> Tuple[sapien.Articulation, np.ndarray]:
        raise NotImplementedError

    def _load_actuator(self) -> List[Tuple[str, int, float, float]]:
        raise NotImplementedError

    def _get_obs(self) -> np.ndarray:
        raise NotImplementedError

    def reset_model(self) -> np.ndarray:
        raise NotImplementedError

    def viewer_setup(self, width=640, height=480) -> sapien.VulkanController:
        raise NotImplementedError


class SapienAntEnv(SapienControlEnv):
    def __init__(self, frame_skip, timestep=0.01, gravity=(0, 0, -9.8)):
        SapienControlEnv.__init__(self, frame_skip, timestep, gravity)
        self.sim.add_ground(-1)

    def step(self, a):
        x_before = self._root.pose.p[0]
        self.do_simulation(a, self.frame_skip)
        x_after = self._root.pose.p[0]
        forward_reward = (x_after - x_before) / self.dt
        ctrl_cost = .5 * np.square(a).sum()
        survive_reward = 1.0
        reward = forward_reward - ctrl_cost + survive_reward
        state = self.state_vector()
        not_done = 0.2 <= state[2] <= 1.0
        ob = self._get_obs()
        return ob, reward, not not_done, dict(
            reward_forward=forward_reward,
            reward_ctrl=-ctrl_cost,
            reward_survive=survive_reward)

    def _load_model(self) -> Tuple[sapien.Articulation, np.ndarray]:
        ant_builder = create_ant_builder(self.sim)
        ant = ant_builder.build(fix_base=False)

        # Step once to make sure everything goes smoothly
        self.sim.step()
        init_state = ant.pack()
        return ant, init_state

    def _load_actuator(self) -> List[Tuple[str, int, float, float]]:
        assert self.model, "Ant model should be loaded before actuator"
        actuator_info = []
        idx = 0
        for joint in self.model.get_joints():
            if joint.get_dof() == 0:
                continue
            if joint.get_name().startswith("hip"):
                actuator_info.append((joint.get_name(), idx, -2000, 2000))
            elif joint.get_name().startswith("ankle"):
                actuator_info.append((joint.get_name(), idx, -1000, 1000))
            idx += joint.get_dof()

        return actuator_info

    def _get_obs(self) -> np.ndarray:
        return self.state_vector()[2:]

    def reset_model(self) -> np.ndarray:
        self.model.unpack(self._init_state)
        # Perturb the joint velocity a little bit
        self.model.set_qvel(self.model.get_qvel() + np.random.rand(self.model.dof) * 0.1)
        return self._get_obs()

    def viewer_setup(self, width=640, height=480) -> sapien.VulkanController:
        self.sim.set_ambient_light([.4, .4, .4])
        self.sim.set_shadow_light([1, -1, -1], [.5, .5, .5])
        self.sim.add_point_light([2, 2, 2], [1, 1, 1])
        self.sim.add_point_light([2, -2, 2], [1, 1, 1])
        self.sim.add_point_light([-2, 0, 2], [1, 1, 1])

        controller = sapien.VulkanController(self._renderer)
        controller.set_current_scene(self.sim)
        controller.set_free_camera_position(0, 1, 10)
        controller.set_free_camera_rotation(0, -1.5, 0)
        return controller


def main():
    env = SapienAntEnv(5, timestep=1 / 100)
    observation = env.reset()
    for i in range(1000):
        env.render()
        action = env.action_spec.sample()
        observation, reward, done, info = env.step(action)

    env.close()
    env.sim = None


if __name__ == '__main__':
    main()
