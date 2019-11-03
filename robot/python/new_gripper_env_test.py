from robot.python.env.movo_env import MOVOEnv
import sapyen_robot
import sys


def main():
    CONVEX_PARTNET_DIR = "/home/sim/project/mobility_convex"
    obj_id = 35059
    env = MOVOEnv()

    for _ in range(10000):
        env.step()

    env.close_gripper(1)
    for _ in range(10000):
        env.step()
    env.open_gripper(4)
    for _ in range(10000):
        env.step()


if __name__ == '__main__':
    sapyen_robot.ros.init(sys.argv, "sapien_robot")
    main()
