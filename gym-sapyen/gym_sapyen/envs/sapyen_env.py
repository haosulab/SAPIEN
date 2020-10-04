import os

from gym import error, spaces
from gym.utils import seeding
import numpy as np
from os import path
import gym
import six
import numpy as np

try:
    import sys
    sys.path.append("/home/yikuan/source/physx_simulation/build")
    import sapyen
except ImportError as e:
    raise error.DependencyNotInstalled("{}. (HINT: you need to install sapyen.)".format(e))

DEFAULT_SIZE = 500

class SapyenEnv(gym.Env):
    """Superclass for all Sapyen environments.
    """

    def __init__(self, frame_skip, timestep):
        self.frame_skip = frame_skip
        self.timestep = timestep
        self.viewer = None
        self._viewers = {}

        self.metadata = {
            'render.modes': ['human'], 
            'video.frames_per_second': int(np.round(1.0 / self.dt))
        }
        self.seed()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # methods to override:
    # ----------------------------

    def reset_model(self):
        """
        Reset the robot degrees of freedom (qpos and qvel).
        Implement this in each subclass.
        """
        raise NotImplementedError

    def viewer_setup(self):
        """
        This method is called when the viewer is initialized.
        Optionally implement this method, if you need to tinker with camera position
        and so forth.
        """
        pass

    # -----------------------------

    def reset(self):
        #self.sim.reset()
        ob = self.reset_model()
        return ob


    def set_state(self, qpos, qvel):
        '''assert qpos.shape == (self.model.nq,) and qvel.shape == (self.model.nv,)
        old_state = self.sim.get_state()
        new_state = mujoco_py.MjSimState(old_state.time, qpos, qvel,
                                         old_state.act, old_state.udd_state)
        self.sim.set_state(new_state)
        self.sim.forward()'''
        pass

    @property
    def dt(self):
        return self.timestep * self.frame_skip

    def do_simulation(self, a, n_frames):
        #self.sim.data.ctrl[:] = ctrl
        for _ in range(n_frames):
            #self.wrapper.set_qf(a)
            self.wrapper.apply_actuator(a)
            self.sim.step()

    def render(self, mode='human', width=DEFAULT_SIZE, height=DEFAULT_SIZE):
        '''if mode == 'rgb_array':
            self._get_viewer(mode).render(width, height)
             window size used for old mujoco-py:
            data = self._get_viewer(mode).read_pixels(width, height, depth=False)
             original image is upside-down, so flip it
            return data[::-1, :, :]
        elif mode == 'depth_array':
            self._get_viewer(mode).render(width, height)
            window size used for old mujoco-py:
            Extract depth part of the read_pixels() tuple
            data = self._get_viewer(mode).read_pixels(width, height, depth=True)[1]
             original image is upside-down, so flip it
            return data[::-1, :]
        elif mode == 'human':
            self._get_viewer(mode).render()'''
        self.sim.update_renderer()
        self.renderer.render()

    def close(self):
        '''if self.viewer is not None:
            # self.viewer.finish()
            self.viewer = None
            self._viewers = {}'''
        pass

    def _get_viewer(self, mode):
        '''self.viewer = self._viewers.get(mode)
        if self.viewer is None:
            if mode == 'human':
                self.viewer = mujoco_py.MjViewer(self.sim)
            elif mode == 'rgb_array' or mode == 'depth_array':
                self.viewer = mujoco_py.MjRenderContextOffscreen(self.sim, -1)
                
            self.viewer_setup()
            self._viewers[mode] = self.viewer
        return self.viewer'''
        pass

    def get_body_com(self, body_name):
    #    return self.data.get_body_xpos(body_name)
        pass

    def state_vector(self):
        return np.concatenate([
            self.body_link.get_global_pose().p,
            self.body_link.get_global_pose().q,
            self.wrapper.get_qpos().flat,
            self.body_link.get_linear_velocity(),
            self.body_link.get_angular_velocity(),
            self.wrapper.get_qvel().flat,
            np.clip(self.wrapper.get_cfrc_ext(), -1, 1).flat
        ])