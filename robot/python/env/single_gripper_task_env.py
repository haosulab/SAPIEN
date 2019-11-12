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
        self.normal_fn = self.__get_normal_estimation_fn()

    def step(self):
        self.object.set_qf(self.__close_object_qf)
        print(self.normal_fn())
        self._step()

    def hold_and_step(self):
        self.force_gripper()
        self.step()

    def __init_arena_camera(self):
        # Set a camera and rendering a semantics part
        camera_target_pose = self.calculate_pose_in_front_of_object_link(self.target_link_index, category=None,
                                                                         horizontal_offset=1.5, vertical_offset=0)
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

    # def _get_door_bounding_box_cloud(self):
    # pc_list = []
    # for cam_id in range(len(self.cam_list)):
    #     self.render_point_cloud(cam_id=cam_id, xyz=True, rgba=False, segmentation=True, normal=False,
    #                             world_coordinate=True)
    #
    # # Calculate center and generic pose of gripper
    # assert point_cloud.shape[1] == 3, "Point Cloud must be in (n, 3) shape"
    # pc = open3d.geometry.PointCloud()
    # pc.points = open3d.utility.Vector3dVector(point_cloud)
    # bounding_box: open3d.geometry.AxisAlignedBoundingBox = pc.get_axis_aligned_bounding_box()
    # center = bounding_box.get_center()
    # box_min = bounding_box.get_min_bound()
    # box_max = bounding_box.get_max_bound()
    # scale = box_max - box_min
    # gripper_pose = sapyen.Pose(center)
    # z_euler = 1.57 * np.sign(center[0]) + 1.57
    # if scale[1] > scale[2]:
    #     print("Horizontal Handle Detected")
    #     gripper_pose.set_q(transforms3d.euler.euler2quat(1.57, 0, z_euler, "rxyz"))
    # else:
    #     print("Vertical Handle Detected")
    #     gripper_pose.set_q(transforms3d.euler.euler2quat(0, 0, z_euler, "rxyz"))
    #
    # # Add offset for Gripper
    # position = gripper_pose.p
    # position[0] -= 0.06
    # gripper_pose.set_p(position)
    # return gripper_pose

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

    def open_the_door_policy(self, feed_back_step=5):
        target_link_joint_index = self.object.get_link_joint_indices()[self.target_link_index]
        target_joint_index = int(np.sum(self.object.get_joint_dofs()[0:target_link_joint_index + 1]) - 1)
        target_link_seg_id = self.object_link_segmentation_ids[self.target_link_index]

        init_robot_pose = self.robot_global_pose
        init_link_pose = self.object_links[self.target_link_index].get_global_mass_center()

        backward = 1 if init_robot_pose.p[0] > 0 else -1
        backward_direction = np.array([backward, 0, 0])
        theta = 0

        # Clear the buffer
        for _ in range(self.simulation_hz // 3):
            self.step()

        # First try open with a small angle
        for _ in range(self.simulation_hz * 3):
            self.translation_controller.move_joint(backward_direction * self._translation_velocity / 4)
            self.hold_and_step()
        for _ in range(self.simulation_hz * 2):
            self.translation_controller.move_joint(backward_direction * self._translation_velocity / 4)
            self.gripper_controller.move_joint([-2, -2, -2])
            self.step()
        for _ in range(self.simulation_hz):
            self.translation_controller.move_joint(backward_direction * self._translation_velocity)
            self.step()

    # step = 0
    # for _ in range(self.simulation_hz * 2):
    #     for i in range(feed_back_step):
    #         self.hold_and_step()
    #         step += 1
    #         self.translation_controller.move_joint(backward_direction * self._translation_velocity / 3)
    #         self.rotation_controller.move_joint(self._gripper_joint, theta * self.simulation_hz / feed_back_step)
    #
    #     mean_normal = self.get_mean_normal_with_seg_id(0, target_link_seg_id)
    #     backward = np.sum(backward_direction * mean_normal)
    #     theta = -np.arccos(backward) * np.sign(backward)
    #     backward_direction = np.sign(backward) * mean_normal

    # if step % 1000 == 0:
    #     # for _ in range(10):
    #     #     self.gripper_controller.move_joint([-5, -5, -5])
    #     current_object_pose = self.object_links[self.target_link_index].get_global_mass_center()
    #     new_target_pose = (current_object_pose.transform(init_link_pose.inv())).transform(init_robot_pose)
    #     print(new_target_pose)
    #     self.arrive_pose_with_closed_loop(new_target_pose, loop_step=1)
    #     self.close_gripper(5)
    #
    # if self.object.get_qvel()[target_joint_index] < -0.1 and \
    #         self.object.get_qacc()[target_joint_index] < 0:
    #     return
    # self.track_object_closed_loop(None)


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
        for _ in range(1000):
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
