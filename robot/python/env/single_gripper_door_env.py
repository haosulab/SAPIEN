from .single_gripper_env import SingleGripperBaseEnv
from .base_env import SapienSingleObjectEnv
from typing import List, Union, Optional
from .physx_utils import mat2transform, transform2mat
import numpy as np
import sapyen
import open3d
import transforms3d
from .path_utils import CONVEX_PARTNET_DIR, DOOR_WITH_HANDLE_LIST, VALID_DOOR_INDEX


class SingleGripperOpenDoorEnv(SingleGripperBaseEnv, SapienSingleObjectEnv):
    def __init__(self, valid_id: int, on_screening_rendering: bool):
        SapienSingleObjectEnv.__init__(self, CONVEX_PARTNET_DIR, DOOR_WITH_HANDLE_LIST[VALID_DOOR_INDEX[valid_id]],
                                       on_screening_rendering)
        self._init_robot()
        self._continuous = False
        self.sim.step()

        # Get a magic force opposite to the direction of door opening
        self.target_link_index = self.object_link_semantics.index("rotation_door")
        self.__close_object_qf = -0.03 * np.ones(self.object.dof())

        # Set a camera and rendering a semantics part
        camera_target_pose = self.calculate_pose_in_front_of_object_link(self.target_link_index, category=None,
                                                                         horizontal_offset=1.5, vertical_offset=0)
        self.add_camera("front_view", camera_target_pose, width=640, height=480)
        self._step()
        part_point_cloud = self.get_global_part_point_cloud_with_seg_id(0, "handle", self.target_link_index)
        if part_point_cloud.size == 0:
            raise RuntimeError("No pullable part detected.")

        # Heuristic method to get the gripper target pose
        gripper_target = self.calculate_grasp_pose_from_handle_cloud(part_point_cloud)
        self.gripper_target = gripper_target
        self.object.set_qvel(np.zeros(self.object.dof()))

    def step(self):
        self.object.set_qf(self.__close_object_qf)
        self._step()

    def hold_and_step(self):
        self.force_gripper()
        self.step()

    def calculate_grasp_pose_from_handle_cloud(self, point_cloud: np.ndarray) -> sapyen.Pose:
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
        position[0] -= 0.06
        gripper_pose.set_p(position)
        return gripper_pose

    def move_ee_against_link(self, cam_id: int, seg_id: int):
        point_cloud = self.render_point_cloud(cam_id, xyz=False, rgba=False, segmentation=True, normal=True)
        normal = point_cloud[:, :, 0:3]
        segmentation = point_cloud[:, :, 3]

        valid_bool = seg_id == segmentation
        mean_normal = np.mean(normal[valid_bool], axis=0)
        mean_normal /= np.linalg.norm(mean_normal)
        mean_normal = self.camera_pose[cam_id][0:3, 0:3] @ mean_normal[:, None]
        print(mean_normal)

        current_normal = transform2mat(self.get_robot_link_global_pose_by_name(self._ee_link_name))[0:3, 2]
        cos = np.cross(current_normal, mean_normal[:, 0])[2] * self.simulation_hz
        self._continuous = True
        return cos

    def move_to_target_pose_policy(self, pose: sapyen.Pose):
        target_position = pose.p
        target_quat = pose.q
        target_mat = transforms3d.quaternions.quat2mat(target_quat)

        current_pose = self.robot_global_pose
        current_position = current_pose.p
        current_quat = current_pose.q
        current_mat = transforms3d.quaternions.quat2mat(current_quat)

        rotation = np.linalg.inv(current_mat) @ target_mat
        translation = target_position - current_position
        translation_distance = np.linalg.norm(translation)
        translation_direction = translation / translation_distance
        euler_rotation = transforms3d.euler.mat2euler(rotation, "rxyz")
        rotation_distance = np.linalg.norm(euler_rotation)
        rotation_direction = euler_rotation / rotation_distance

        # First step pose
        target_position[0] += 0.5 * np.sign(target_position[0])
        first_target_pose = sapyen.Pose(target_position, target_quat)
        self.arrive_pose_with_closed_loop(first_target_pose)
        self.arrive_pose_with_closed_loop(pose)

        # self.robot.set_drive_qpos(np.concatenate([target_position, [0,0,0], [0,0,0]]))
        # for i in range(100000):
        #     self.step()

        # while True:
        #     self.rotation_controller.move_joint(rotation_direction * self._rotation_velocity)
        #     self.step()
        #     if np.linalg.norm(self.robot_global_pose.q - target_quat) < 0.01:
        #         break

        # while True:
        #     self.translation_controller.move_joint(translation_direction * self._translation_velocity)
        #     self.step()
        #     if np.linalg.norm(self.robot_global_pose.p - target_position) < 0.01:
        #         print("Get to target position")
        #         break
