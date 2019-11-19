import sapyen_robot
from .base_robot_env import BaseRobotEnv
from robot.python.env.physx_utils import mat2transform, transform2mat
import transforms3d
import numpy as np
import os
from .path_utils import get_assets_path

RGBD_CAMERA_THRESHOLD = 10
CAMERA_TO_LINK = np.zeros([4, 4])
CAMERA_TO_LINK[[0, 1, 2, 3], [2, 0, 1, 3]] = [1, -1, -1, 1]


class MOVOEnv(BaseRobotEnv):
    def __init__(self):
        """
        Sapien Kinova MOVO base class.
        If you want to use it with sapien object environment, do not use __init__ but _init_robot" (most case)
        If you just want to load the robot only, you should consider use __init__ but not _init_robot
        """
        urdf_path = os.path.join(get_assets_path(), "robot/all_robot.urdf")
        gripper_material = self.sim.create_material(3.0, 2.0, 0.01)
        BaseRobotEnv.__init__(self, urdf_path, gripper_material)
        print("Initiate MOVO Environment in stand alone version")

    def _init_robot(self) -> None:
        """
        Load the robot and controllers
        """
        gripper_material = self.sim.create_material(3.0, 2.0, 0.01)
        self._load_robot('../assets/robot/single_gripper.urdf', gripper_material)

    def _load_controller(self) -> None:
        """
        Create controllers, set pd and force limit to each joint with fine tuned value
        """
        controllable_wrapper = self.sim.create_controllable_articulation(self.robot)
        self._head_joint = ["pan_joint", "tilt_joint"]
        self._gripper_joint = ["right_gripper_finger1_joint", "right_gripper_finger2_joint",
                               "right_gripper_finger3_joint"]
        self._body_joint = ["linear_joint"]
        self.manger = sapyen_robot.ControllerManger("movo", controllable_wrapper)

        self.head_controller = self.manger.create_joint_velocity_controller(self._head_joint, "head")
        self.gripper_controller = self.manger.create_joint_velocity_controller(self._gripper_joint, "gripper")
        self.body_controller = self.manger.create_joint_velocity_controller(self._body_joint, "body")

        # Add joint state publisher to keep in synchronization with ROS
        # You must use it if you want to do cartesian control
        self.manger.add_joint_state_publisher(60)
        self.manger.add_group_trajectory_controller("right_arm")
        self.arm_planner = self.manger.create_group_planner("right_arm")

        # Cache gripper limit for execute high level action
        joint_limit = self.robot.get_qlimits()
        gripper_index = self.robot_joint_names.index(self._gripper_joint[0])
        self.__gripper_limit = joint_limit[gripper_index, :]

        # Cache robot pose
        self.root_theta = 0
        self.root_pos = np.array([0, 0], dtype=np.float)
        self.init_qpos = [0, 0, 0, 0.25, -1.9347, 0, -1.5318, 0, 0.9512, -2.24, 0.34, 0.64, -1.413, 0, 0, 0]

        # Tune PD controller
        self.robot.set_pd(20000, 3000, 2000, np.arange(4))
        self.robot.set_drive_qpos(self.init_qpos)
        self.robot.set_qpos(self.init_qpos)
        self.sim.step()

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

    def move_robot_to_target_place(self, target_pose):
        end_pose = np.eye(4)
        end_pose[0:2, 3] = target_pose[0:2, 3]
        end_pose[0:2, 0:2] = target_pose[0:2, 0:2]
        current_pose = np.eye(4)
        current_pose[0:2, 3] = self.root_pos
        new_theta = transforms3d.euler.mat2euler(end_pose)[2]
        x_axis = end_pose[0:3, 3] - current_pose[0:3, 3]
        x_axis /= np.linalg.norm(x_axis)
        z_axis = np.array([0, 0, 1])
        y_axis = np.cross(z_axis, x_axis)
        forward_pose = np.stack([x_axis, y_axis, z_axis], axis=1)
        relative_pose = np.linalg.inv(current_pose[0:3, 0:3]) @ forward_pose
        move_direction_theta = transforms3d.euler.mat2euler(relative_pose)[2]
        angular_velocity = 0.6
        velocity = 0.4

        sign = np.sign(move_direction_theta - self.root_theta)
        for _ in range(
                np.ceil(np.abs(move_direction_theta - self.root_theta) / angular_velocity * self.simulation_hz).astype(
                    np.int)):
            self.root_theta += angular_velocity / self.simulation_hz * sign
            current_pose[0:2, 0:2] = np.array([[np.cos(self.root_theta), -np.sin(self.root_theta)],
                                               [np.sin(self.root_theta), np.cos(self.root_theta)]])
            self.manger.move_base(mat2transform(current_pose))
            self.step()

        move_direction = end_pose[0:2, 3] - self.root_pos
        move_distance = np.linalg.norm(move_direction)
        move_direction /= move_distance

        for _ in range(np.ceil(move_distance / velocity * self.simulation_hz).astype(np.int)):
            self.root_pos += move_direction * velocity / self.simulation_hz
            current_pose[0:2, 3] = self.root_pos
            self.manger.move_base(mat2transform(current_pose))
            self.step()

        sign = np.sign(new_theta - self.root_theta)
        for _ in range(
                np.ceil(np.abs(new_theta - self.root_theta) / angular_velocity * self.simulation_hz).astype(np.int)):
            self.root_theta += angular_velocity / self.simulation_hz * sign
            current_pose[0:2, 0:2] = np.array([[np.cos(self.root_theta), -np.sin(self.root_theta)],
                                               [np.sin(self.root_theta), np.cos(self.root_theta)]])
            self.manger.move_base(mat2transform(current_pose))
            self.step()

    def translate_end_effector(self, translation: np.ndarray):
        assert translation.shape == (3,), "Translation should be a 3d vector"
        ee_pose = self.get_robot_link_relative_pose("right_ee_link")
        ee_pose.set_p(np.array(ee_pose.p) + translation)
        result = self.arm_planner.go(ee_pose, "base_link")
        return result

    def move_camera_toward_semantic(self, semantic_name):
        links = self.object.get_links()
        semantic_link = self.semantic_mapping['semantic2link']
        name = semantic_link[semantic_name]
        link_names = self.object.get_link_names()
        link_index = link_names.index(name)

        # Move robot to the right place
        link_pose = transform2mat(links[link_index].get_global_mass_center())
        camera_pose = self.get_robot_link_global_pose("kinect2_color_optical_frame")
        robot_target_pose = np.eye(4)
        robot_target_pose[0:3, 3] = link_pose[0:3, 3]
        robot_target_pose[0, 3] -= 1.5
        self.move_robot_to_target_place(robot_target_pose)

        # Get camera pan and tilt target
        relative_pose = link_pose[:3, 3] - camera_pose[:3, 3]
        tilt_angle = np.arctan(relative_pose[2] / np.linalg.norm(relative_pose[0:2]))

        # Move camera to the right place
        for _ in range(self.simulation_hz):
            self.head_controller.move_joint(["tilt_joint"], tilt_angle)
            self.step()

    def close_gripper(self, velocity):
        time_step = self.gripper_limit[1] / velocity * self.simulation_hz
        for _ in range(time_step.astype(np.int)):
            self.gripper_controller.move_joint(self._gripper_joint, velocity)

    def open_gripper(self, velocity):
        time_step = self.gripper_limit[1] / velocity * self.simulation_hz
        for _ in range(time_step.astype(np.int)):
            self.gripper_controller.move_joint(self._gripper_joint, -velocity)
