import sapyen_robot
import sapyen
from robot.python.env.gripper_door_env import DoorEnv
from robot.python.env.single_gripper_env import SingleGripperBaseEnv
from robot.python.env.base_env import SapienSingleObjectEnv
from gym import utils, spaces

from typing import List, Union
import numpy as np
import sys
import os
from robot.python.env.path_utils import get_assets_path

if __name__ == '__main__':
    data_dir = '/home/fangchen/sim/mobility-v0-prealpha3/mobility_verified'
    save_dir = '/home/fangchen/physx_simulation/robot/data'
    partnet_id = '45162'
    #print(sys.argv)

    sapyen_robot.ros.init(sys.argv, "tester")
    initial_state = np.load(save_dir + "/{}_gripper_v1.p".format(partnet_id), allow_pickle=True)['state'][0]
    print(initial_state)
    env = DoorEnv(dataset_dir=data_dir, data_id=partnet_id, on_screening_rendering=True, initial_state=initial_state)
    #env = SapienSingleObjectEnv(dataset_dir=data_dir, data_id=partnet_id, on_screening_rendering=True)
    obs = env.reset()
    done = False
    count_step = 0
    while not done:
        action = env.action_space.sample()
        next_state, reward, done, info = env.step(action)
        count_step += 1
        if done:
            env.reset()
