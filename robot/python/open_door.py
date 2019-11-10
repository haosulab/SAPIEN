from robot.python.env.open_door_env import XArmOpenDoorEnv
import sapyen_robot
import sapyen
import numpy as np
import sys


def main():
    env = XArmOpenDoorEnv(0, True)
    for _ in range(1000):
        env.step()
    env.approach_target_policy(env.gripper_target)
    env.open_door_policy()


if __name__ == '__main__':
    sapyen_robot.ros.init(sys.argv, "open_door")
    main()
