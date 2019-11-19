import sapyen
import sapyen_robot
from .base_robot_env import BaseRobotEnv
from .physx_utils import mat2transform, rpy2transform
import numpy as np
from typing import Union
import os
from .path_utils import get_assets_path


class MOVOEnv(BaseRobotEnv):
    def __init__(self):
        """
        Sapien Kinova MOVO base class.
        If you want to use it with sapien object environment, do not use __init__ but _init_robot" (most case)
        If you just want to load the robot only, you should consider use __init__ but not _init_robot
        """
        BaseRobotEnv.__init__(self)
        gripper_material = self.sim.create_material(3.0, 2.0, 0.01)
        self._load_robot(os.path.join(get_assets_path(), "robot/all_robot.urdf"), gripper_material)
        self._load_ros_controller()
        print("Initiate MOVO Environment in stand alone version")

    def _init_robot(self) -> None:
        """
        Load the robot and controllers
        """
        gripper_material = self.sim.create_material(3.0, 2.0, 0.01)
        self._load_robot(os.path.join(get_assets_path(), "robot/all_robot.urdf"), gripper_material)

    def _load_controller_parameters(self):
        # Cache robot pose
        self.root_theta = 0
        self.root_pos = np.array([0, 0], dtype=np.float)
        self.robot.set_root_pose([-1, 2, 0.06])
        self.init_qpos = [0.25, -1.381, 0, 0.05, 0.9512, 0.387, 0.608, 2.486, 1.05, -1.16, 0, 0, 0]

        # Tune PD controller
        self.robot.set_pd(20000, 3000, 2000, np.arange(1))
        self.robot.set_pd(2000, 300, 300, [1, 3, 5, 6, 7, 8, 9])
        self.robot.set_pd(500, 100, 300, [2, 4])
        self.robot.set_pd(200, 40, 20, np.arange(10, 13))
        self.robot.set_drive_qpos(self.init_qpos)
        self.robot.set_qpos(self.init_qpos)
        self.sim.step()

        # Create manger
        controllable_wrapper = self.sim.create_controllable_articulation(self.robot)
        self.manger = sapyen_robot.ControllerManger("movo", controllable_wrapper)

    def _load_ros_controller(self) -> None:
        """
        Create controllers, set pd and force limit to each joint with fine tuned value
        """
        self._head_joint = ["pan_joint", "tilt_joint"]
        self._gripper_joint = ["right_gripper_finger1_joint", "right_gripper_finger2_joint",
                               "right_gripper_finger3_joint"]
        self._body_joint = ["linear_joint"]

        self.manger.add_joint_state_publisher(60)
        self.head_controller = self.manger.create_joint_velocity_controller(self._head_joint, "head")
        self.gripper_controller = self.manger.create_joint_velocity_controller(self._gripper_joint, "right_gripper")
        self.body_controller = self.manger.create_joint_velocity_controller(self._body_joint, "body")

        # Add joint state publisher to keep in synchronization with ROS
        # You must use it if you want to do cartesian control
        self.__arm_velocity_controller = self.manger.create_cartesian_velocity_controller("right_arm")
        self.manger.add_group_trajectory_controller("right_arm")
        self.__arm_planner = self.manger.create_group_planner("right_arm")

        # Cache gripper limit for execute high level action
        joint_limit = self.robot.get_qlimits()
        gripper_index = self.robot_joint_names.index(self._gripper_joint[0])
        self.__gripper_limit = joint_limit[gripper_index, :]
        self.manger.start()

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
        ee_pose = self.get_robot_link_local_pose_by_name("right_ee_link")
        ee_pose.set_p(np.array(ee_pose.p) + translation)
        result = self.__arm_planner.go(ee_pose, "base_link")
        return result

    def move_ee_relative_rotation(self, rpy: np.ndarray) -> None:
        """
        Move end effector in robot frame with relative rotation
        :param rpy: 3d vector of row, yaw, pitch in robot frame
        """
        ee_pose = self.get_robot_link_local_pose_by_name("right_ee_link")
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
        ee_pose = self.get_robot_link_local_pose_by_name("right_ee_link")
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
