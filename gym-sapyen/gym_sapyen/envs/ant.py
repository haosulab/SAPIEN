from gym import utils, spaces
#from gym.envs.mujoco import mujoco_env
from . import sapyen_env
import numpy as np
try:
    import sys
    sys.path.append("/home/yikuan/source/physx_simulation/build")
    import sapyen
    from sapyen import PxTransform
except ImportError as e:
    raise error.DependencyNotInstalled("{}. (HINT: you need to install sapyen.)".format(e))

class AntEnv(sapyen_env.SapyenEnv, utils.EzPickle):
    def __init__(self):
        sapyen_env.SapyenEnv.__init__(self, 5, 0.01)
        self._set_renderer()
        self.sim = sapyen.Simulation()
        self.sim.set_renderer(self.renderer)
        self.sim.set_time_step(0.01)
        self.wrapper, self.body_link = self._build_ant()
        self.init_qpos = self.wrapper.get_qpos().ravel().copy()
        self.init_qvel = self.wrapper.get_qvel().ravel().copy()
        self.init_root_pose_p = self.body_link.get_global_pose().p
        self.init_root_pose_q = self.body_link.get_global_pose().q
        observation, _reward, done, _info = self.step(np.zeros(8))
        assert not done
        self.obs_dim = observation.size

        bounds = self.wrapper.get_force_actuator_range()
        low = bounds[:, 0] 
        high = bounds[:, 1]
        self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
        high = np.inf*np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high, dtype=np.float32)
        utils.EzPickle.__init__(self)

    def _set_renderer(self):
        self.renderer = sapyen.OptifuserRenderer()
        
        self.renderer.cam.set_position(np.array([0, 1, 10]))
        self.renderer.cam.set_forward(np.array([0, 1, 0]))
        self.renderer.cam.set_up(np.array([0, 0, 1]))
        self.renderer.cam.rotate_yaw_pitch(0, -1.5)

    def _build_ant(self):
        builder = self.sim.create_articulation_builder()
        PxIdentity = np.array([1, 0, 0, 0])
        density = 5
        body_link = builder.add_link(None, PxTransform(np.array([0, 0, 0]), PxIdentity), "body")
        builder.add_sphere_shape_to_link(body_link, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.25)
        builder.add_sphere_visual_to_link(body_link, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.25)
        builder.add_capsule_shape_to_link(body_link,  PxTransform(np.array([0.141, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.add_capsule_visual_to_link(body_link, PxTransform(np.array([0.141, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.add_capsule_shape_to_link(body_link,  PxTransform(np.array([-0.141, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.add_capsule_visual_to_link(body_link, PxTransform(np.array([-0.141, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.add_capsule_shape_to_link(body_link, PxTransform(np.array([0, 0.141, 0]), np.array([0.707107, 0, 0, 0.707107])), 0.08, 0.141)
        builder.add_capsule_visual_to_link(body_link, PxTransform(np.array([0, 0.141, 0]), np.array([0.707107, 0, 0, 0.707107])), 0.08, 0.141)
        builder.add_capsule_shape_to_link(body_link, PxTransform(np.array([0, -0.141, 0]), np.array([0.707107, 0, 0, 0.707107])), 0.08, 0.141)
        builder.add_capsule_visual_to_link(body_link, PxTransform(np.array([0, -0.141, 0]), np.array([0.707107, 0, 0, 0.707107])), 0.08, 0.141)
        builder.update_link_mass_and_inertia(body_link, density)
        l1 = builder.add_link(body_link,  PxTransform(np.array([0, 0, 0]), PxIdentity), "l1", "j1",
                                    sapyen.PxArticulationJointType.REVOLUTE, np.array([[-0.5236, 0.5236]]),
                                    PxTransform(np.array([0.282, 0, 0]), np.array([0.7071068, 0, 0.7071068, 0])),
                                    PxTransform(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
        builder.add_capsule_shape_to_link(l1,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.add_capsule_visual_to_link(l1,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.update_link_mass_and_inertia(l1, density)
        l2 = builder.add_link(body_link,  PxTransform(np.array([0, 0, 0]), PxIdentity), "l2", "j2",
                                    sapyen.PxArticulationJointType.REVOLUTE, np.array([[-0.5236, 0.5236]]),
                                    PxTransform(np.array([-0.282, 0, 0]), np.array([0, 0.7071068, 0, -0.7071068])),
                                    PxTransform(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
        builder.add_capsule_shape_to_link(l2,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.add_capsule_visual_to_link(l2,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.update_link_mass_and_inertia(l2, density)
        l3 = builder.add_link(body_link,  PxTransform(np.array([0, 0, 0]), PxIdentity), "l3", "j3",
                                    sapyen.PxArticulationJointType.REVOLUTE, np.array([[-0.5236, 0.5236]]),
                                    PxTransform(np.array([0, 0.282, 0]), np.array([0.5, -0.5, 0.5, 0.5])),
                                    PxTransform(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
        builder.add_capsule_shape_to_link(l3,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.add_capsule_visual_to_link(l3,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.update_link_mass_and_inertia(l3, density)
        l4 = builder.add_link(body_link,  PxTransform(np.array([0, 0, 0]), PxIdentity), "l4", "j4",
                                    sapyen.PxArticulationJointType.REVOLUTE, np.array([[-0.5236, 0.5236]]),
                                    PxTransform(np.array([0, -0.282, 0]), np.array([0.5, 0.5, 0.5, -0.5])),
                                    PxTransform(np.array([0.141, 0, 0]), np.array([ 0.7071068, 0, -0.7071068, 0])))
        builder.add_capsule_shape_to_link(l4,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.add_capsule_visual_to_link(l4,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.update_link_mass_and_inertia(l4, density)
        f1 = builder.add_link(l1,  PxTransform(np.array([0, 0, 0]), PxIdentity), "f1", "j11",
                                    sapyen.PxArticulationJointType.REVOLUTE, np.array([[0.5236, 1.222]]),
                                    PxTransform(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                                    PxTransform(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
        builder.add_capsule_shape_to_link(f1,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
        builder.add_capsule_visual_to_link(f1,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
        builder.update_link_mass_and_inertia(f1, density)
        f2 = builder.add_link(l2,  PxTransform(np.array([0, 0, 0]), PxIdentity), "f2", "j21",
                                    sapyen.PxArticulationJointType.REVOLUTE, np.array([[0.5236, 1.222]]),
                                    PxTransform(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                                    PxTransform(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
        builder.add_capsule_shape_to_link(f2,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
        builder.add_capsule_visual_to_link(f2,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
        builder.update_link_mass_and_inertia(f2, density)
        f3 = builder.add_link(l3,  PxTransform(np.array([0, 0, 0]), PxIdentity), "f3", "j31",
                                    sapyen.PxArticulationJointType.REVOLUTE, np.array([[0.5236, 1.222]]),
                                    PxTransform(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                                    PxTransform(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
        builder.add_capsule_shape_to_link(f3,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
        builder.add_capsule_visual_to_link(f3,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
        builder.update_link_mass_and_inertia(f3, density)
        f4 = builder.add_link(l4,  PxTransform(np.array([0, 0, 0]), PxIdentity), "f4", "j41",
                                    sapyen.PxArticulationJointType.REVOLUTE, np.array([[0.5236, 1.222]]),
                                    PxTransform(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                                    PxTransform(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
        builder.add_capsule_shape_to_link(f4,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
        builder.add_capsule_visual_to_link(f4,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
        builder.update_link_mass_and_inertia(f4, density)
        wrapper = builder.build(False)
        lower_bound = -10
        upper_bound = 10
        wrapper.add_force_actuator("j1", lower_bound, upper_bound)
        wrapper.add_force_actuator("j2", lower_bound, upper_bound)
        wrapper.add_force_actuator("j3", lower_bound, upper_bound)
        wrapper.add_force_actuator("j4", lower_bound, upper_bound)
        wrapper.add_force_actuator("j11", lower_bound, upper_bound)
        wrapper.add_force_actuator("j21", lower_bound, upper_bound)
        wrapper.add_force_actuator("j31", lower_bound, upper_bound)
        wrapper.add_force_actuator("j41", lower_bound, upper_bound)
        wrapper.set_root_pose(np.array([0, 0, 0.55]))
        ground = self.sim.add_ground(-1)
        return wrapper, body_link

    def step(self, a):
        xposbefore = self.body_link.get_global_pose().p[0]
        self.do_simulation(a, self.frame_skip)
        xposafter = self.body_link.get_global_pose().p[0]
        forward_reward = (xposafter - xposbefore)/self.dt
        ctrl_cost = .5 * np.square(a).sum()
        contact_cost = 0.5 * 1e-3 * np.sum(
            np.square(np.clip(self.wrapper.get_cfrc_ext(), -1, 1)))
        survive_reward = 1.0
        reward = forward_reward - ctrl_cost - contact_cost + survive_reward
        state = self.state_vector()
        notdone = np.isfinite(state).all() \
            and state[2] >= 0.2 and state[2] <= 1.0 
        done = not notdone
        ob = self._get_obs()
        return ob, reward, done, dict(
            reward_forward=forward_reward,
            reward_ctrl=-ctrl_cost,
            reward_contact=-contact_cost,
            reward_survive=survive_reward)

    def _get_obs(self):
        return np.concatenate([
            self.body_link.get_global_pose().p[2:],
            self.body_link.get_global_pose().q,
            self.wrapper.get_qpos().flat,
            self.body_link.get_linear_velocity(),
            self.body_link.get_angular_velocity(),
            self.wrapper.get_qvel().flat,
            np.clip(self.wrapper.get_cfrc_ext(), -1, 1).flat, # 54->84 ??
        ])

    def reset_model(self):
        qpos = self.init_qpos + self.np_random.uniform(size=8, low=-.1, high=.1)
        qvel = self.init_qvel + self.np_random.randn(8) * .1
        self.wrapper.set_qpos(qpos)
        self.wrapper.set_qvel(qvel)
        root_pose_p = self.init_root_pose_p #+ self.np_random.uniform(size=3, low=-.1, high=.1)
        root_pose_q = self.init_root_pose_q #+ self.np_random.uniform(size=4, low=-.1, high=.1)
        self.wrapper.set_root_pose(root_pose_p, root_pose_q)
        # TODO: reset root_velocity
        return self._get_obs()

    def viewer_setup(self):
        #self.viewer.cam.distance = self.model.stat.extent * 0.5
        pass
