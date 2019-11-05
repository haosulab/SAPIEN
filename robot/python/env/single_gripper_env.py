import sapyen_robot
import sapyen
from .base_robot_env import BaseRobotEnv
import numpy as np
import os
from .path_utils import get_assets_path


class SingleGripperBaseEnv(BaseRobotEnv):
    def __init__(self):
        """
        Sapien single gripper base class.
        If you want to use it with sapien object environment, do not use __init__ but _init_robot
            e.g. single_hand_recorder
        If you just want to load the robot only, you should consider use __init__ but not _init_robot
        """
        gripper_material = self.sim.create_material(3.0, 2.0, 0.01)
        BaseRobotEnv.__init__(self)
        self._load_robot(os.path.join(get_assets_path(), "robot/single_gripper.urdf"), gripper_material)
        self._load_ros_controller()
        print("Initiate Single Gripper Environment in stand alone version")

    def _init_robot(self) -> None:
        """
        Load the robot and controllers
        """
        gripper_material = self.sim.create_material(3.0, 2.0, 0.01)
        self._load_robot(os.path.join(get_assets_path(), "robot/single_gripper.urdf"), gripper_material)

    def _load_controller_parameters(self):
        # Init robot pose and controller
        self.robot.set_pd(200, 40, 20, [0, 1, 2, 3, 4, 5])
        self.robot.set_pd(1, 0.05, 1, [6, 7, 8])
        self.init_qpos = [0, 0, 1, 0, 0, 0, 0, 0, 0]
        self.robot.set_drive_qpos(self.init_qpos)
        self.robot.set_qpos(self.init_qpos)
        self.sim.step()

        # Change the base link name
        self._base_link_name = "right_ee_link"

        # Load manger
        controllable_wrapper = self.sim.create_controllable_articulation(self.robot)
        self.manger = sapyen_robot.ControllerManger("movo", controllable_wrapper)

    def _load_ros_controller(self) -> None:
        """
        Create controllers, set pd and force limit to each joint with fine tuned value
        """
        self.gripper_joint = ["right_gripper_finger1_joint", "right_gripper_finger2_joint",
                              "right_gripper_finger3_joint"]
        self.translation_joint = ["x_axis_joint", "y_axis_joint",
                                  "z_axis_joint"]
        self.rotation_joint = ["r_rotation_joint", "p_rotation_joint",
                               "y_rotation_joint"]
        self.gripper_controller = self.manger.create_joint_velocity_controller(self.gripper_joint, "gripper")
        self.translation_controller = self.manger.create_joint_velocity_controller(self.translation_joint,
                                                                                   "translation")
        self.rotation_controller = self.manger.create_joint_velocity_controller(self.rotation_joint, "rotation")

        # Cache gripper limit for execute high level action
        joint_limit = self.robot.get_qlimits()
        gripper_index = self.robot_joint_names.index(self.gripper_joint[0])
        self.__gripper_limit = joint_limit[gripper_index, :]

        # Note that you should always start the manger before use any ROS utility
        # Otherwise the spinner will not going to process the thread callback
        self.manger.start()

    def close_gripper(self, velocity: float = 2) -> None:
        """
        Close gripper with given velocity
        :param velocity: Velocity of gripper joint
        """
        time_step = self.__gripper_limit[1] / velocity * self.simulation_hz
        for _ in range(time_step.astype(np.int)):
            self.gripper_controller.move_joint(self.gripper_joint, velocity)

    def open_gripper(self, velocity: float = 2) -> None:
        """
        Open gripper with given velocity
        :param velocity: Velocity of gripper joint
        """
        time_step = self.__gripper_limit[1] / velocity * self.simulation_hz
        for _ in range(time_step.astype(np.int)):
            self.gripper_controller.move_joint(self.gripper_joint, -velocity)
