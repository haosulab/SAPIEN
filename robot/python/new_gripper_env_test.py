from robot.python.env.xarm_sapien_env import XArmRecorder

import sapyen_robot
import sys


def main():
    CONVEX_PARTNET_DIR = "/home/sim/project/mobility_convex"
    obj_id = 35059
    env = XArmRecorder(CONVEX_PARTNET_DIR, obj_id, True)
    env.robot.set_root_pose([1, 0, 0], [1, 0, 0, 0])

    for _ in range(1000):
        env.step()

    for _ in range(10000):
        env.step()
    for _ in range(10000):
        env.step()


if __name__ == '__main__':
    sapyen_robot.ros.init(sys.argv, "sapien_robot")
    main()
