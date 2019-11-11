from .xarm_env import XArmEnv
from .base_env import SapienSingleObjectEnv
from typing import List, Union, Optional
from .physx_utils import mat2transform, transform2mat
import numpy as np
import sapyen
import open3d
import transforms3d
from .path_utils import CONVEX_PARTNET_DIR, DOOR_WITH_HANDLE_LIST, VALID_DOOR_INDEX


class XArmOpenDoorEnv(XArmEnv, SapienSingleObjectEnv):
    def __init__(self, valid_id: int, on_screening_rendering: bool):
        SapienSingleObjectEnv.__init__(self, CONVEX_PARTNET_DIR, DOOR_WITH_HANDLE_LIST[VALID_DOOR_INDEX[valid_id]],
                                       on_screening_rendering)
        self._init_robot()
        self._continuous = False
        self.sim.step()

        # Get a magic force opposite to the direction of door opening
        self.target_link_index = self.object_link_semantics.index("rotation_door")
        self.__close_object_qf = -0.01 * np.ones(self.object.dof())

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

    def approach_target_policy(self, gripper_target: sapyen.Pose, time_out: float = 10):
        pre_target = transform2mat(gripper_target)
        approach_direction = pre_target[:3, 2]
        pre_target[:3, 3] = pre_target[:3, 3] - approach_direction * 0.2

        self.move_ee(pre_target)
        self.open_gripper()
        steps = 0
        while True:
            self.step()
            steps += 1
            if steps > time_out * self.simulation_hz:
                print("Time out without finishing move")
                break
            if np.mean(np.abs(self.robot.get_qvel())) < 0.0001 and np.mean(np.abs(self.robot.get_qacc())) < 0.0001:
                print("Pre move action finished")
                break

        self.move_ee(gripper_target)
        steps = 0
        while True:
            self.step()
            steps += 1
            if steps > time_out * self.simulation_hz:
                print("Time out without finishing move")
                break
            if np.mean(np.abs(self.robot.get_qvel())) < 0.0001 and np.mean(np.abs(self.robot.get_qacc())) < 0.0001:
                print("Approach target action finished")
                break

        self.arm_velocity_controller.move_local_translate([0, 0, 0.02], False)
        for i in range(200):
            self.arm_velocity_controller.move_local_translate([0, 0, 0.02], True)
            self.step()
        for _ in range(100):
            self.hold_and_step()

    def open_door_policy(self):
        self.arm_velocity_controller.move_local_translate([0, 0, -0.005], False)

        self.hold_and_step()
        for i in range(5000):
            self.arm_velocity_controller.move_local_translate([0, 0, -0.02], True)
            self.hold_and_step()
            if np.mean(np.abs(self.object.get_qpos())) < 0.01 and i > 1000:
                break
            if i % 50 == 9:
                cos = self.move_ee_against_link(0, self.object_link_segmentation_ids[self.target_link_index])
                self.arm_velocity_controller.move_local_translate([0, -cos * 0.006, 0], True)
                self.arm_velocity_controller.move_local_rotate([-cos * 0.025, 0, 0], True)
                self.hold_and_step()

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
        if scale[1] > scale[2]:
            print("Horizontal Handle Detected")
            gripper_pose.set_q(transforms3d.euler.euler2quat(0, 1.57, 1.57, "rxyz"))
        else:
            print("Vertical Handle Detected")
            gripper_pose.set_q(transforms3d.euler.euler2quat(0, 1.57, 3.14, "rxyz"))

        # Move robot to the right place
        self.object.set_qpos(np.ones(self.object.dof()) * 0.01)
        robot_target_pose = self.calculate_pose_in_front_of_semantics("rotation_door")
        target_position = robot_target_pose.p
        target_position[1:3] = [np.mean([target_position[1], center[1]]), center[2] - 0.5]
        robot_target_pose.set_p(target_position)
        self.set_robot_base_pose(robot_target_pose)
        self.object.set_qpos(np.ones(self.object.dof()) * 0.01)
        self.sim.step()

        # Add offset for XArm
        gripper_target = self.robot_global_pose.inv().transform(gripper_pose)
        position = gripper_target.p
        position[0] -= 0.16
        gripper_target.set_p(position)
        return gripper_target

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
