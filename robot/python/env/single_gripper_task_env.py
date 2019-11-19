from .single_gripper_env import SingleGripperBaseEnv
from .base_env import SapienSingleObjectEnv
from .physx_utils import mat2transform, transform2mat
import numpy as np
import sapyen
import open3d
import transforms3d
from .path_utils import PARTNET_DIR, DOOR_WITH_HANDLE_LIST, DRAWER_WITH_HANDLE_LIST


class SingleGripperOpenDoorEnv(SingleGripperBaseEnv, SapienSingleObjectEnv):
    def __init__(self, valid_id: int, on_screening_rendering: bool):
        SapienSingleObjectEnv.__init__(self, PARTNET_DIR, DOOR_WITH_HANDLE_LIST[valid_id],
                                       on_screening_rendering)
        self._init_robot()
        self.object.set_pd(0, 2)
        self._continuous = False
        self.sim.step()

        # Get a magic force opposite to the direction of door opening
        self.target_link_index = self.object_link_semantics.index("rotation_door")
        self.__close_object_qf = -0.00 * np.ones(self.object.dof())

        self.__init_arena_camera()
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

    def __init_arena_camera(self):
        # Set a camera and rendering a semantics part
        camera_target_pose = self.calculate_pose_in_front_of_object_link(self.target_link_index, category=None,
                                                                         horizontal_offset=2.5, vertical_offset=0)
        self.add_camera("front_view", camera_target_pose, width=1080, height=1080)
        object_pose = self.object_links[self.target_link_index].get_global_mass_center()
        object_pose.set_q([1, 0, 0, 0])
        camera_relative_pose = object_pose.inv().transform(camera_target_pose)

        object_pose.set_q(transforms3d.euler.euler2quat(0, 0, 1.0))
        camera_right_pose = object_pose.transform(camera_relative_pose)
        object_pose.set_q(transforms3d.euler.euler2quat(0, 0, -1.0))
        camera_left_pose = object_pose.transform(camera_relative_pose)
        self.add_camera("right_view", camera_right_pose, width=1080, height=1080)
        self.add_camera("left_view", camera_left_pose, width=1080, height=1080)

    def calculate_edge_grasp_pose(self, return_size_info=False):
        obj_segmentation_list = []
        for cam_id in range(len(self.cam_list)):
            part_cloud = self.get_global_part_point_cloud_with_seg_id(cam_id, "door", self.object_link_segmentation_ids[
                self.target_link_index])
            obj_segmentation_list.append(part_cloud.reshape(-1, 3))
        all_part_cloud = np.concatenate(obj_segmentation_list, axis=0)

        pc = open3d.geometry.PointCloud()
        pc.points = open3d.utility.Vector3dVector(all_part_cloud)
        bounding_box: open3d.geometry.OrientedBoundingBox = pc.get_oriented_bounding_box()
        eight_points = np.asarray(bounding_box.get_box_points())
        front_four_index = np.argsort(np.abs(eight_points[:, 0]))[4:]
        back_four_index = np.argsort(np.abs(eight_points[:, 0]))[:4]

        edge_points = eight_points[front_four_index]
        center = np.mean(edge_points, axis=0)
        direction = center - np.mean(eight_points[back_four_index, :], axis=0)
        door_length = np.linalg.norm(direction)
        direction /= door_length

        gripper_position = center + direction * 0.2
        angle = np.arccos(np.sum(np.array([np.sign(center[0]), 0, 0]) * direction)) * np.sign(
            np.cross(np.array([np.sign(center[0]), 0, 0]), direction)[2])
        gripper_quaternion = transforms3d.euler.euler2quat(0, 0, angle, "rxyz")
        if return_size_info:
            return sapyen.Pose(gripper_position, gripper_quaternion), -direction, door_length, center
        else:
            return sapyen.Pose(gripper_position, gripper_quaternion), -direction

    def move_to_target_pose_policy(self, pose: sapyen.Pose):
        target_position = pose.p
        target_quat = pose.q

        # First step pose
        target_position[0] += 0.5 * np.sign(target_position[0])
        first_target_pose = sapyen.Pose(target_position, target_quat)
        self.arrive_pose_with_closed_loop(first_target_pose)

        # Second step pose
        self.arrive_pose_with_closed_loop(pose)

        # Rest and close the gripper
        for _ in range(200):
            self.hold_and_step()

    def __get_normal_estimation_fn(self):
        camera_pose = self.calculate_pose_in_front_of_object_link(self.target_link_index, category=None,
                                                                  horizontal_offset=1.5, vertical_offset=0)
        self.add_camera("front_view", camera_pose, width=640, height=480)
        self.sim.step()
        mean_normal = self.get_mean_normal_with_seg_id(0, self.object_link_segmentation_ids[
            self.target_link_index]).reshape(3, 1)

        joint_index = self.object.get_link_joint_indices()[self.target_link_index]
        axis_pose = self.object.get_link_joint_pose(self.target_link_index)
        axis_orientation = transforms3d.quaternions.quat2mat(axis_pose.q)
        joint_index = int(np.sum(self.object.get_joint_dofs()[0:joint_index + 1]) - 1)
        init_qpos = self.object.get_qpos()[joint_index]
        local_mean_normal = np.linalg.inv(axis_orientation) @ mean_normal

        def get_target_normal_fn():
            qpos = self.object.get_qpos()[joint_index]
            relative_transform = transforms3d.euler.euler2mat(qpos - init_qpos, 0, 0)
            return axis_orientation @ relative_transform @ local_mean_normal

        return get_target_normal_fn

    def open_the_door_policy(self, feed_back_step=1):
        target_link_joint_index = self.object.get_link_joint_indices()[self.target_link_index]
        target_joint_index = int(np.sum(self.object.get_joint_dofs()[0:target_link_joint_index + 1]) - 1)
        target_joint_limit = self.object.get_qlimits()[target_joint_index, 1]
        init_robot_pose = self.robot_global_pose
        backward = 1 if init_robot_pose.p[0] > 0 else -1
        backward_direction = np.array([backward, 0, 0])

        # Clear the buffer
        for _ in range(self.simulation_hz // 1):
            self.step()

        # First try open with a small angle
        for _ in range(self.simulation_hz):
            self.translation_controller.move_joint(backward_direction * self._translation_velocity / 2)
            self.hold_and_step()
        for _ in range(self.simulation_hz * 2):
            self.translation_controller.move_joint(backward_direction * self._translation_velocity / 2)
            self.gripper_controller.move_joint([-5, -5, -5])
            self.step()
        for _ in range(self.simulation_hz // 2):
            self.translation_controller.move_joint(backward_direction * self._translation_velocity)
            self.step()

        if np.abs(self.object.get_qpos()[target_joint_index]) < 0.1:
            return False
        for _ in range(self.simulation_hz * 2):
            self.step()

        # Grasp the door edge
        new_target_pose, forward_direction, door_length, edge_center = self.calculate_edge_grasp_pose(
            return_size_info=True)
        step = 0
        while np.linalg.norm(self.robot_global_pose.p - new_target_pose.p) > 0.01:
            if step > 20:
                return False
            if np.abs(self.object.get_qpos()[target_joint_index] - target_joint_limit) < 0.1 * target_joint_limit:
                return True
            self.arrive_pose_with_closed_loop(new_target_pose, loop_step=feed_back_step, velocity_factor=0.2)
            new_target_pose, forward_direction = self.calculate_edge_grasp_pose()
            step += 1
        new_target_pose.set_p(new_target_pose.p + forward_direction * 0.24)
        self.arrive_pose_with_closed_loop(new_target_pose, loop_step=feed_back_step)

        # Move the door
        for _ in range(self.simulation_hz // 2):
            self.hold_and_step()
        final_direction = -np.sign(transform2mat(self.object.get_link_joint_pose(self.target_link_index))[2, 0])
        final_rotation_direction = -int(final_direction)

        step = 0
        start_qpos = self.object.get_qpos()[target_joint_index]
        while True:
            current_q = self.object.get_qpos()[target_joint_index]
            current_v = np.array([-np.cos(current_q), final_direction * np.sin(current_q), 0])
            self.translation_controller.move_joint(current_v * self._translation_velocity * 0.5)
            self.rotation_controller.move_joint(
                [0, 0, final_rotation_direction * self._translation_velocity / door_length * 0.5])
            self.hold_and_step()
            step += 1

            if step > 1000 and np.abs(
                    current_q - target_joint_limit) < 0.1 * target_joint_limit:
                return True

            if step > 1000 and np.abs(self.object.get_qvel()[target_joint_index]) < 0:
                return False

            if current_q < start_qpos - 0.1:
                return False


class SingleGripperOpenDrawer(SingleGripperBaseEnv, SapienSingleObjectEnv):
    def __init__(self, valid_id: int, on_screening_rendering: bool):
        SapienSingleObjectEnv.__init__(self, PARTNET_DIR, DRAWER_WITH_HANDLE_LIST[valid_id],
                                       on_screening_rendering)
        self._init_robot()
        self._continuous = False
        self.sim.step()

        # Get a magic force opposite to the direction of door opening
        self.target_link_index = self.object_link_semantics.index("drawer")
        self.__close_object_qf = -0.03 * np.ones(self.object.dof())

        # Set a camera and rendering a semantics part
        self.__init_arena_camera()
        self._step()
        self.sim.update_renderer()
        part_point_cloud = self.get_global_part_point_cloud_with_seg_id(0, "handle", self.target_link_index)
        if part_point_cloud.size == 0:
            raise RuntimeError("No pullable part detected.")

        # Heuristic method to get the gripper target pose
        gripper_target = self.calculate_grasp_pose_from_handle_cloud(part_point_cloud)
        self.gripper_target = gripper_target
        self.object.set_qvel(np.zeros(self.object.dof()))

    def __init_arena_camera(self):
        # Set a camera and rendering a semantics part
        camera_target_pose = self.calculate_pose_in_front_of_object_link(self.target_link_index, category=None,
                                                                         horizontal_offset=2.5, vertical_offset=-0.4)
        self.add_camera("front_view", camera_target_pose, width=1080, height=1080)
        object_pose = self.object_links[self.target_link_index].get_global_mass_center()
        object_pose.set_q([1, 0, 0, 0])
        camera_relative_pose = object_pose.inv().transform(camera_target_pose)

        object_pose.set_q(transforms3d.euler.euler2quat(0, 0, 1.0))
        camera_right_pose = object_pose.transform(camera_relative_pose)
        object_pose.set_q(transforms3d.euler.euler2quat(0, 0, -1.0))
        camera_left_pose = object_pose.transform(camera_relative_pose)
        self.add_camera("right_view", camera_right_pose, width=1080, height=1080)
        self.add_camera("left_view", camera_left_pose, width=1080, height=1080)

    def step(self):
        self.object.set_qf(self.__close_object_qf)
        self._step()

    def hold_and_step(self):
        self.force_gripper()
        self.step()

    def move_to_target_pose_policy(self, pose: sapyen.Pose):
        target_position = pose.p
        target_quat = pose.q

        # First step pose
        target_position[0] += 0.5 * np.sign(target_position[0])
        first_target_pose = sapyen.Pose(target_position, target_quat)
        self.arrive_pose_with_closed_loop(first_target_pose, loop_step=1)

        # Second step pose
        self.arrive_pose_with_closed_loop(pose, loop_step=1)

        # Rest and close the gripper
        for _ in range(100):
            self.hold_and_step()

    def open_drawer_policy(self, feed_back_step=5):
        target_link_joint_index = self.object.get_link_joint_indices()[self.target_link_index]
        target_joint_index = int(np.sum(self.object.get_joint_dofs()[0:target_link_joint_index + 1]) - 1)
        target_joint_limit = self.object.get_qlimits()[target_joint_index, 1]
        target_link_seg_id = self.object_link_segmentation_ids[self.target_link_index]
        print(f"joint limit: {target_joint_limit}")

        init_robot_pose = self.robot_global_pose
        backward = 1 if init_robot_pose.p[0] > 0 else -1
        backward_direction = np.array([backward, 0, 0])

        step = 0
        while True:
            for i in range(feed_back_step):
                self.hold_and_step()
                step += 1
                self.translation_controller.move_joint(backward_direction * self._translation_velocity / 3)

            mean_normal = self.get_mean_normal_with_seg_id(0, target_link_seg_id)
            backward = np.sum(backward_direction * mean_normal)
            backward_direction = np.sign(backward) * mean_normal

            if self.object.get_qvel()[target_joint_index] < -0.02 and \
                    self.object.get_qacc()[target_joint_index] < 0:
                return False

            if np.abs(self.object.get_qpos()[target_joint_index]) < 0.001 and step > 500:
                return False

            if np.linalg.norm(self.object.get_qpos()[target_joint_index] - target_joint_limit) < 0.01:
                return True
                # self.track_object_closed_loop(None)
