from robot.python.env.single_gripper_task_env import SingleGripperOpenDoorEnv, SingleGripperOpenDrawer, \
    DOOR_WITH_HANDLE_LIST, DRAWER_WITH_HANDLE_LIST
import sapyen_robot
from time import time
import sys
import numpy as np
import pickle


def generate_header(env):
    header = {}
    header.update({"robot_joint_name": env.robot.get_joint_names()})
    header.update({"robot_link_name": env.robot.get_link_names()})
    header.update({"object_joint_name": env.object.get_joint_names()})
    header.update({"object_link_name": env.object.get_link_names()})
    return header


def open_drawer():
    index = 15
    data = {}
    env = SingleGripperOpenDrawer(index, on_screening_rendering=False)
    for _ in range(200):
        env.step()
    env.move_to_target_pose_policy(env.gripper_target)
    data.update({"state": np.array(env.sim.dump())[np.newaxis, :]})
    data.update({"header": generate_header(env)})
    data.update({"target_link_index": env.target_link_index})
    success = env.open_drawer_policy()
    print("Success: {}".format(success))
    for _ in range(300):
        env.step()


invalid_door_index = [3, 7, 9, 11, 13, 14, 15, 17, 20, 22, 26, 27, 36, 42, 45, 47, 48, 50, 51, 52, 55, 59, 68, 74, 80,
                      82, 86, 87, 90, 91, 92, 97, 102, 105]


# 35, 43, 63
def open_door():
    index = 40
    env = SingleGripperOpenDoorEnv(index, on_screening_rendering=False)
    color = [4, 0.773 * 4, 0.561 * 4]
    env.renderer.add_point_light([0, 0, 3.6], color)
    env.renderer.add_point_light([-0.2, -4, 4.2], color)
    env.renderer.add_point_light([0.3, -2, 3.7], color)
    env.move_to_target_pose_policy(env.gripper_target)
    success = env.open_the_door_policy()
    print(f"Success: {success}")
    for _ in range(300):
        env._step()


if __name__ == '__main__':
    sapyen_robot.ros.init(sys.argv, "single_gripper_open_door")
    open_drawer()
