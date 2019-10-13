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
        self.sim.setRenderer(self.renderer)
        self.sim.setTimestep(0.01)
        self.wrapper, self.body_link = self._build_ant()
        self.init_qpos = self.wrapper.get_qpos().ravel().copy()
        self.init_qvel = self.wrapper.get_qvel().ravel().copy()
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
        self.renderer.init()
        self.renderer.cam.set_position(np.array([0, 1, 10]))
        self.renderer.cam.setForward(np.array([0, 1, 0]))
        self.renderer.cam.setUp(np.array([0, 0, 1]))
        self.renderer.cam.rotateYawPitch(0, -1.5)

    def _build_ant(self):
        builder = self.sim.createArticulationBuilder()
        PxIdentity = np.array([0, 0, 0, 1])
        density = 5
        body_link = builder.addLink(None, PxTransform(np.array([0, 0, 0]), PxIdentity), "body")
        builder.addSphereShapeToLink(body_link, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.25)
        builder.addSphereVisualToLink(body_link, PxTransform(np.array([0, 0, 0]), PxIdentity), 0.25)
        builder.addCapsuleShapeToLink(body_link,  PxTransform(np.array([0.141, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.addCapsuleVisualToLink(body_link, PxTransform(np.array([0.141, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.addCapsuleShapeToLink(body_link,  PxTransform(np.array([-0.141, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.addCapsuleVisualToLink(body_link, PxTransform(np.array([-0.141, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.addCapsuleShapeToLink(body_link, PxTransform(np.array([0, 0.141, 0]), np.array([0.707107, 0, 0, 0.707107])), 0.08, 0.141)
        builder.addCapsuleVisualToLink(body_link, PxTransform(np.array([0, 0.141, 0]), np.array([0.707107, 0, 0, 0.707107])), 0.08, 0.141)
        builder.addCapsuleShapeToLink(body_link, PxTransform(np.array([0, -0.141, 0]), np.array([0.707107, 0, 0, 0.707107])), 0.08, 0.141)
        builder.addCapsuleVisualToLink(body_link, PxTransform(np.array([0, -0.141, 0]), np.array([0.707107, 0, 0, 0.707107])), 0.08, 0.141)
        builder.updateLinkMassAndInertia(body_link, density)
        l1 = builder.addLink(body_link,  PxTransform(np.array([0, 0, 0]), PxIdentity), "l1", "j1",
                                    sapyen.PxArticulationJointType.eREVOLUTE, np.array([[-0.5236, 0.5236]]),
                                    PxTransform(np.array([0.282, 0, 0]), np.array([0.7071068, 0, 0.7071068, 0])),
                                    PxTransform(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
        builder.addCapsuleShapeToLink(l1,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.addCapsuleVisualToLink(l1,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.updateLinkMassAndInertia(l1, density)
        l2 = builder.addLink(body_link,  PxTransform(np.array([0, 0, 0]), PxIdentity), "l2", "j2",
                                    sapyen.PxArticulationJointType.eREVOLUTE, np.array([[-0.5236, 0.5236]]),
                                    PxTransform(np.array([-0.282, 0, 0]), np.array([0, 0.7071068, 0, -0.7071068])),
                                    PxTransform(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
        builder.addCapsuleShapeToLink(l2,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.addCapsuleVisualToLink(l2,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.updateLinkMassAndInertia(l2, density)
        l3 = builder.addLink(body_link,  PxTransform(np.array([0, 0, 0]), PxIdentity), "l3", "j3",
                                    sapyen.PxArticulationJointType.eREVOLUTE, np.array([[-0.5236, 0.5236]]),
                                    PxTransform(np.array([0, 0.282, 0]), np.array([0.5, -0.5, 0.5, 0.5])),
                                    PxTransform(np.array([0.141, 0, 0]), np.array([0.7071068, 0, -0.7071068, 0])))
        builder.addCapsuleShapeToLink(l3,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.addCapsuleVisualToLink(l3,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.updateLinkMassAndInertia(l3, density)
        l4 = builder.addLink(body_link,  PxTransform(np.array([0, 0, 0]), PxIdentity), "l4", "j4",
                                    sapyen.PxArticulationJointType.eREVOLUTE, np.array([[-0.5236, 0.5236]]),
                                    PxTransform(np.array([0, -0.282, 0]), np.array([0.5, 0.5, 0.5, -0.5])),
                                    PxTransform(np.array([0.141, 0, 0]), np.array([ 0.7071068, 0, -0.7071068, 0])))
        builder.addCapsuleShapeToLink(l4,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.addCapsuleVisualToLink(l4,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.141)
        builder.updateLinkMassAndInertia(l4, density)
        f1 = builder.addLink(l1,  PxTransform(np.array([0, 0, 0]), PxIdentity), "f1", "j11",
                                    sapyen.PxArticulationJointType.eREVOLUTE, np.array([[0.5236, 1.222]]),
                                    PxTransform(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                                    PxTransform(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
        builder.addCapsuleShapeToLink(f1,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
        builder.addCapsuleVisualToLink(f1,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
        builder.updateLinkMassAndInertia(f1, density)
        f2 = builder.addLink(l2,  PxTransform(np.array([0, 0, 0]), PxIdentity), "f2", "j21",
                                    sapyen.PxArticulationJointType.eREVOLUTE, np.array([[0.5236, 1.222]]),
                                    PxTransform(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                                    PxTransform(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
        builder.addCapsuleShapeToLink(f2,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
        builder.addCapsuleVisualToLink(f2,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
        builder.updateLinkMassAndInertia(f2, density)
        f3 = builder.addLink(l3,  PxTransform(np.array([0, 0, 0]), PxIdentity), "f3", "j31",
                                    sapyen.PxArticulationJointType.eREVOLUTE, np.array([[0.5236, 1.222]]),
                                    PxTransform(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                                    PxTransform(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
        builder.addCapsuleShapeToLink(f3,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
        builder.addCapsuleVisualToLink(f3,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
        builder.updateLinkMassAndInertia(f3, density)
        f4 = builder.addLink(l4,  PxTransform(np.array([0, 0, 0]), PxIdentity), "f4", "j41",
                                    sapyen.PxArticulationJointType.eREVOLUTE, np.array([[0.5236, 1.222]]),
                                    PxTransform(np.array([-0.141, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])),
                                    PxTransform(np.array([0.282, 0, 0]), np.array([0, 0.7071068, 0.7071068, 0])))
        builder.addCapsuleShapeToLink(f4,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
        builder.addCapsuleVisualToLink(f4,  PxTransform(np.array([0, 0, 0]), PxIdentity), 0.08, 0.282)
        builder.updateLinkMassAndInertia(f4, density)
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
        ground = self.sim.addGround(-1)
        return wrapper, body_link

    def step(self, a):
        xposbefore = self.body_link.getGlobalPose().p[0]
        self.do_simulation(a, self.frame_skip)
        xposafter = self.body_link.getGlobalPose().p[0]
        forward_reward = (xposafter - xposbefore)/self.dt
        ctrl_cost = .5 * np.square(a).sum()
        contact_cost = 0.5 * 1e-3 * np.sum(
            np.square(np.clip(self.wrapper.get_cfrc_ext(), -1, 1)))
        survive_reward = 1.0
        reward = forward_reward - ctrl_cost - contact_cost + survive_reward
        state = self.state_vector()
        notdone = np.isfinite(state).all() #\
        #    and state[2] >= 0.2 and state[2] <= 1.0 # z ??
        done = not notdone
        ob = self._get_obs()
        return ob, reward, done, dict(
            reward_forward=forward_reward,
            reward_ctrl=-ctrl_cost,
            reward_contact=-contact_cost,
            reward_survive=survive_reward)

    def _get_obs(self):
        return np.concatenate([
            self.wrapper.get_qpos().flat[2:],
            self.wrapper.get_qvel().flat,
            np.clip(self.wrapper.get_cfrc_ext(), -1, 1).flat, 
        ])

    def reset_model(self):
        qpos = self.init_qpos + self.np_random.uniform(size=8, low=-.1, high=.1)
        qvel = self.init_qvel + self.np_random.randn(8) * .1
        self.wrapper.set_qpos(qpos)
        self.wrapper.set_qvel(qvel)
        return self._get_obs()

    def viewer_setup(self):
        #self.viewer.cam.distance = self.model.stat.extent * 0.5
        pass