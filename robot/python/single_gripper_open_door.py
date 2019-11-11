from robot.python.env.single_gripper_door_env import SingleGripperOpenDoorEnv
import sapyen_robot
import sys


def main():
    env = SingleGripperOpenDoorEnv(18, on_screening_rendering=True)
    for _ in range(1000):
        env.step()
    env.move_to_target_pose_policy(env.gripper_target)
    for _ in range(1000):
        env.step()


if __name__ == '__main__':
    sapyen_robot.ros.init(sys.argv, "single_gripper_open_door")
    main()
