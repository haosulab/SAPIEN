import sapyen
import sapyen_robot
from .base_robot_env import BaseRobotEnv
from .physx_utils import mat2transform, rpy2transform
import numpy as np
from typing import Union
from .path_utils import get_assets_path
import os


class XArmEnv(BaseRobotEnv):
    def __init__(self):
        """
        Sapien Kinova MOVO base class.
        If you want to use it with sapien object environment, do not use __init__ but _init_robot" (most case)
        If you just want to load the robot only, you should consider use __init__ but not _init_robot
        """
        BaseRobotEnv.__init__(self)
        gripper_material = self.sim.create_material(3.0, 2.0, 0.01)
        self._load_robot(os.path.join(get_assets_path(), "robot/xarm6.urdf"), gripper_material)
        print("Initiate MOVO Environment in stand alone version")

    def _init_robot(self) -> None:
        """
        Load the robot and controllers
        """
        gripper_material = self.sim.create_material(3.0, 2.0, 0.01)
        self._load_robot('../assets/robot/xarm6.urdf', gripper_material)

    def _load_controller_parameters(self):
        self.__ee_link_name = "link6"
        self.root_theta = 0
        self.root_pos = np.array([0, 0], dtype=np.float)
        self.init_qpos = np.array([0.121, -0.889, -0.467, 3.897, 0.216, -3.85, 0, 0, 0, 0, 0, 0])

        # Tune PD controller
        self.robot.set_pd(2000, 300, 300, np.arange(6))
        self.robot.set_pd(5000, 800, 20000, np.arange(6, 12))
        self.robot.set_drive_qpos(self.init_qpos)
        self.robot.set_qpos(self.init_qpos)
        self.sim.step()

        # Create manger
        controllable_wrapper = self.sim.create_controllable_articulation(self.robot)  # Cache robot pose
        self.manger = sapyen_robot.ControllerManger("xarm6", controllable_wrapper)

    def _load_ros_controller(self) -> None:
        """
        Create controllers, set pd and force limit to each joint with fine tuned value
        """
        self._gripper_joint = ["drive_joint",
                               "left_finger_joint",
                               "left_inner_knuckle_joint",
                               "right_outer_knuckle_joint",
                               "right_finger_joint",
                               "right_inner_knuckle_joint"]
        self.gripper_controller = self.manger.create_joint_velocity_controller(self._gripper_joint, "gripper")

        # Add joint state publisher to keep in synchronization with ROS
        # You must use it if you want to do cartesian control
        self.manger.add_joint_state_publisher(60, 400)
        self.manger.add_group_trajectory_controller("xarm6")
        self._arm_velocity_controller = self.manger.create_cartesian_velocity_controller("xarm6")
        self.__arm_planner = self.manger.create_group_planner("xarm6")

        # Cache gripper limit for execute high level action
        joint_limit = self.robot.get_joint_limits()
        gripper_index = self.robot_joint_names.index(self._gripper_joint[0])
        self.__gripper_limit = joint_limit[gripper_index, :]

    def close_gripper(self, velocity: float = 2) -> None:
        """
        Close gripper with given velocity
        :param velocity: Velocity of gripper joint
        """
        time_step = self.__gripper_limit[1] / velocity * self.simulation_hz
        for _ in range(time_step.astype(np.int)):
            self.gripper_controller.move_joint(self._gripper_joint, velocity)

    def open_gripper(self, velocity: float = 2) -> None:
        """
        Open gripper with given velocity
        :param velocity: Velocity of gripper joint
        """
        time_step = self.__gripper_limit[1] / velocity * self.simulation_hz
        for _ in range(time_step.astype(np.int)):
            self.gripper_controller.move_joint(self._gripper_joint, -velocity)

    def move_ee_relative_translation(self, translation: np.ndarray) -> None:
        """
        Move end effector in robot frame with relative translation
        :param translation: 3d vector of xyz in robot frame
        """
        assert translation.shape == (3,), "Translation should be a 3d vector"
        ee_pose = self.get_robot_link_local_pose_by_name(self.__ee_link_name)
        ee_pose.set_p(np.array(ee_pose.p) + translation)
        result = self.__arm_planner.go(ee_pose, "base_link")
        return result

    def move_ee_relative_rotation(self, rpy: np.ndarray) -> None:
        """
        Move end effector in robot frame with relative rotation
        :param rpy: 3d vector of row, yaw, pitch in robot frame
        """
        ee_pose = self.get_robot_link_local_pose_by_name(self.__ee_link_name)
        rotation_pose = rpy2transform(rpy)
        ee_pose = ee_pose.transform(rotation_pose)
        result = self.__arm_planner.go(ee_pose, "base_link")
        return result

    def move_ee_relative_pose(self, pose: Union[sapyen.Pose, np.ndarray]) -> None:
        """
        Move end effector in robot frame with respect to the current one
        :param pose: numpy array or physx natural pose
        """
        if type(pose) == np.ndarray:
            pose = mat2transform(pose)
        ee_pose = self.get_robot_link_local_pose_by_name(self.__ee_link_name)
        new_pose = pose.transform(ee_pose)
        self.__arm_planner.go(new_pose, "base_link")

    def move_ee(self, pose: Union[sapyen.Pose, np.ndarray]) -> None:
        """
        Move end effector in robot frame
        :param pose: numpy array or physx natural pose
        """
        if type(pose) == np.ndarray:
            self.__arm_planner.go(mat2transform(pose), "base_link")
        elif type(pose) == sapyen.Pose:
            self.__arm_planner.go(pose, "base_link")
        else:
            raise RuntimeError("Pose type should be Pose or numpy array")

    def step(self):
        self._step()
