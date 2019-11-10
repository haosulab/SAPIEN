from .xarm_env import XArmEnv
from .base_env import SapienSingleObjectEnv
from typing import List, Union, Optional
from .physx_utils import mat2transform, transform2mat
import numpy as np
import sapyen
import open3d
import transforms3d

CONVEX_PARTNET_DIR = "/home/sim/mobility_dataset/mobility_v1_alpha5"
door_with_handle = ['35059', '40147', '40417', '41003', '41004', '41083', '41085', '41452', '41510',
                    '41529', '44781', '44826', '45001', '45007', '45087', '45091', '45130', '45134', '45146',
                    '45159', '45162', '45164', '45166', '45168', '45173', '45176', '45177', '45189', '45194',
                    '45203', '45212', '45213', '45219', '45235', '45238', '45244', '45247', '45249', '45267',
                    '45271', '45297', '45305', '45332', '45354', '45372', '45378', '45384', '45385', '45387',
                    '45397', '45403', '45415', '45419', '45420', '45423', '45443', '45448', '45463', '45503',
                    '45504', '45505', '45523', '45524', '45526', '45573', '45575', '45594', '45600', '45606',
                    '45612', '45621', '45622', '45623', '45632', '45633', '45636', '45638', '45645', '45661',
                    '45662', '45667', '45670', '45671', '45676', '45687', '45689', '45690', '45693', '45694',
                    '45696', '45699', '45717', '45747', '45749', '45759', '45767', '45776', '45779', '45780',
                    '45783', '45784', '45790', '45850', '45853', '45908', '45915', '45916', '45922', '45936',
                    '45937', '45940', '45948', '45949', '45950', '45961', '45963', '45964', '45984', '46002',
                    '46019', '46029', '46033', '46037', '46044', '46045', '46084', '46092', '46108', '46117',
                    '46120', '46134', '46145', '46166', '46179', '46180', '46197', '46199', '46236', '46277',
                    '46401', '46408', '46417', '46427', '46430', '46452', '46456', '46480', '46490', '46598',
                    '46616', '46700', '46732', '46741', '46744', '46787', '46801', '46825', '46839', '46847',
                    '46856', '46859', '46874', '46889', '46906', '46922', '46944', '46955', '46966', '46981',
                    '47021', '47024', '47088', '47133', '47180', '47182', '47185', '47187', '47227', '47252',
                    '47254', '47278', '47281', '47290', '47315', '47388', '47419', '47514', '47529', '47570',
                    '47577', '47585', '47595', '47601', '47613', '47632', '47648', '47669', '47701', '47729',
                    '47742', '47747', '47808', '47817', '47853', '47926', '47944', '47976', '48018', '48023',
                    '48036', '48063', '48167', '48177', '48243', '48271', '48356', '48379', '48381', '48413',
                    '48452', '48467', '48490', '48513', '48519', '48623', '48700', '48721', '48797', '48859',
                    '48878', '49025', '49038', '49042', '49062', '49132', '49133', '49188']

vertical_door_handle_index = [1, 3, 6, 9, 11, 14, 19, 20, 21, 24, 26, 27, 28, 29, 32, 35, 36, 38, 40, 42, 43,
                              44, 45, 46, 47, 50, 52, 54]


class XArmOpenDoorEnv(XArmEnv, SapienSingleObjectEnv):
    def __init__(self, valid_id: int, on_screening_rendering: bool):
        SapienSingleObjectEnv.__init__(self, CONVEX_PARTNET_DIR, door_with_handle[vertical_door_handle_index[valid_id]],
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

        self._arm_velocity_controller.move_local_translate([0, 0, 0.02], False)
        for i in range(200):
            self._arm_velocity_controller.move_local_translate([0, 0, 0.02], True)
            self.step()
        for _ in range(100):
            self.hold_and_step()

    def open_door_policy(self):
        self._arm_velocity_controller.move_local_translate([0, 0, -0.005], False)

        self.hold_and_step()
        for i in range(5000):
            self._arm_velocity_controller.move_local_translate([0, 0, -0.02], True)
            self.hold_and_step()
            if np.mean(np.abs(self.object.get_qpos())) < 0.01 and i > 1000:
                break
            if i % 50 == 9:
                cos = self.move_ee_against_link(0, self.object_link_segmentation_ids[self.target_link_index])
                self._arm_velocity_controller.move_local_translate([0, -cos*0.006, 0], True)
                self._arm_velocity_controller.move_local_rotate([-cos * 0.025, 0, 0], True)
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
