import sapyen_robot
import sapyen
from .base_robot_env import BaseRobotEnv
import numpy as np
import os
from .path_utils import get_assets_path
import transforms3d
import open3d


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
        self.robot.set_pd(200, 60, 20, [0, 1, 2, 3, 4, 5])
        self.robot.set_pd(1, 0.05, 1, [6, 7, 8])
        self.init_qpos = [0, 0, 2, 0, 0, 0, 0, 0, 0]
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
        self._gripper_joint = ["right_gripper_finger1_joint", "right_gripper_finger2_joint",
                               "right_gripper_finger3_joint"]
        self._translation_joint = ["x_axis_joint", "y_axis_joint",
                                   "z_axis_joint"]
        self._rotation_joint = ["r_rotation_joint", "p_rotation_joint",
                                "y_rotation_joint"]
        self.manger.add_joint_state_publisher(100)
        self.gripper_controller = self.manger.create_joint_velocity_controller(self._gripper_joint, "gripper")
        self.translation_controller = self.manger.create_joint_velocity_controller(self._translation_joint,
                                                                                   "translation")
        self.rotation_controller = self.manger.create_joint_velocity_controller(self._rotation_joint, "rotation")
        self._translation_velocity = 0.25
        self._rotation_velocity = 0.7

        # Cache gripper limit for execute high level action
        joint_limit = self.robot.get_qlimits()
        gripper_index = self.robot_joint_names.index(self._gripper_joint[0])
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
            self.gripper_controller.move_joint(self._gripper_joint, velocity)
            self._step()

    def open_gripper(self, velocity: float = 2) -> None:
        """
        Open gripper with given velocity
        :param velocity: Velocity of gripper joint
        """
        time_step = self.__gripper_limit[1] / velocity * self.simulation_hz
        for _ in range(time_step.astype(np.int)):
            self.gripper_controller.move_joint(self._gripper_joint, -velocity)
            self._step()

    def force_gripper(self) -> None:
        """
        Give a force constantly on the gripper to close
        """
        self.gripper_controller.move_joint([2, 2, 2])

    def arrive_pose_with_closed_loop(self, target_pose: sapyen.Pose, loop_step=5, velocity_factor=0.5) -> None:
        step = 0
        target_quat = target_pose.q
        target_rotation = transforms3d.quaternions.quat2mat(target_quat)
        while True:
            if step % loop_step == 0:
                current_pose = self.robot_global_pose
                current_rotation = transforms3d.quaternions.quat2mat(current_pose.q)
                rotation = np.linalg.inv(current_rotation) @ target_rotation
                euler_rotation = transforms3d.euler.mat2euler(rotation, "rxyz")
                rotation_distance = np.linalg.norm(euler_rotation)
                rotation_direction = euler_rotation / rotation_distance
                if abs(transforms3d.axangles.mat2axangle(rotation)[1]) < 0.1 or step > self.simulation_hz * 5:
                    break

            self.rotation_controller.move_joint(
                rotation_direction * self._rotation_velocity / 2)
            self._step()
            step += 1

        target_position = target_pose.p
        step = 0
        while True:
            if step % loop_step == 0:
                current_pose = self.robot_global_pose
                current_position = current_pose.p
                translation = target_position - current_position
                translation_distance = np.linalg.norm(translation)
                translation_direction = translation / translation_distance
                if translation_distance < 0.01:
                    break

            self.translation_controller.move_joint(
                translation_direction * self._translation_velocity * np.maximum(translation_distance, velocity_factor))
            self._step()
            step += 1

    @staticmethod
    def calculate_grasp_pose_from_handle_cloud(point_cloud: np.ndarray) -> sapyen.Pose:
        # Calculate center and generic pose of gripper
        assert point_cloud.shape[1] == 3, "Point Cloud must be in (n, 3) shape"
        pc = open3d.geometry.PointCloud()
        pc.points = open3d.utility.Vector3dVector(point_cloud)
        bounding_box: open3d.geometry.AxisAlignedBoundingBox = pc.get_axis_aligned_bounding_box()
        center = bounding_box.get_center()
        box_min = bounding_box.get_min_bound()
        box_max = bounding_box.get_max_bound()
        scale = box_max - box_min
        gripper_pose = sapyen.Pose(center)
        z_euler = 1.57 * np.sign(center[0]) + 1.57
        if scale[1] > scale[2]:
            print("Horizontal Handle Detected")
            gripper_pose.set_q(transforms3d.euler.euler2quat(1.57, 0, z_euler, "rxyz"))
        else:
            print("Vertical Handle Detected")
            gripper_pose.set_q(transforms3d.euler.euler2quat(0, 0, z_euler, "rxyz"))

        # Add offset for Gripper
        position = gripper_pose.p
        position[0] -= 0.05
        gripper_pose.set_p(position)
        return gripper_pose
