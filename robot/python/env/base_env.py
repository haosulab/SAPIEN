import re
import sapyen
from typing import List, Union, Optional
from .physx_utils import transform2mat, mat2transform
import transforms3d
import numpy as np
import warnings
import os

RGBD_CAMERA_THRESHOLD = 10
CAMERA_TO_LINK = np.zeros([4, 4])
CAMERA_TO_LINK[[0, 1, 2, 3], [2, 0, 1, 3]] = [1, -1, -1, 1]


def point_cloud_from_depth(depth, color, proj, model):
    W, H = depth.shape

    WS = np.repeat(np.linspace(1 / (2 * W), 1 - 1 / (2 * W), W).reshape([1, -1]), H, axis=0)
    HS = np.repeat(np.linspace(1 / (2 * H), 1 - 1 / (2 * H), H)[::-1].reshape([-1, 1]), W, axis=1)
    points = np.stack([WS, HS, depth, np.ones_like(depth)], 2)

    color = color[depth < 1]
    points = points[depth < 1]
    points = points * 2 - 1
    cam_points = np.linalg.inv(proj) @ points.T
    cam_points /= cam_points[3]

    world_points = model @ cam_points
    world_points = world_points.T
    print(model)

    return world_points[:, :3], color[:, :3]


def get_pc(cam):
    cam.take_picture()
    depth = cam.get_depth()
    color = cam.get_color_rgba()

    xyz, rgb = point_cloud_from_depth(depth, color, cam.get_projection_mat(), cam.get_model_mat())


class BaseEnv:
    def __init__(self, on_screening_rendering: bool):
        """
        Base class of a environment of
        :param on_screening_rendering: Whether to use rendering visualization or not
        """
        # Rendering
        self.renderer = sapyen.OptifuserRenderer()
        self.renderer.set_ambient_light([.4, .4, .4])
        self.renderer.set_shadow_light([1, -1, -1], [.5, .5, .5])
        self.renderer.add_point_light([2, 2, 2], [1, 1, 1])
        self.renderer.add_point_light([2, -2, 2], [1, 1, 1])
        self.renderer.add_point_light([-2, 0, 2], [1, 1, 1])
        self.renderer.cam.set_position([0.5, -2, 2])
        self.renderer.cam.rotate_yaw_pitch(0.5, -0.5)

        # Use rendering step with render if visualization is enable
        if on_screening_rendering:
            self._step = self.__step
            self.renderer.show_window()
        else:
            self._step = lambda: self.sim.step()

        # Simulation
        self.sim = sapyen.Simulation()
        self.sim.set_renderer(self.renderer)
        self.simulation_hz = 200
        self.sim.set_time_step(1 / self.simulation_hz)

        # Articulation loader for both robot and object, articulation builder for simple object without joint
        self.loader = self.sim.create_urdf_loader()
        self.builder = self.sim.create_actor_builder()

        # Camera
        self.camera_frame_id = []
        self.camera_pose = []
        self.camera_name_list = []
        self.cam_list: List[sapyen.mounted_camera] = []
        self.mount_actor_list: List[sapyen.PxRigidActor] = []
        self.mapping_list = []
        self.depth_lambda_list = []
        self.__gl_camera_mapping = []
        self._gl_projection_inv = []

    def __step(self):
        """
        Step function when on screen visualization is enabled
        """
        self.sim.step()
        self.sim.update_renderer()
        self.renderer.render()

    def obj_id2name(self, obj_id: int) -> str:
        return self.sim.get_render_name_dict()[obj_id]

    def _init_camera_cache(self):
        """
        Init camera mapping for camera define in urdf file, like camera on robot's head.
        Camera added by user after the class instantiate will not be handle by this function
        :return:
        """
        num = self.renderer.get_camera_count()
        for i in range(num):
            camera = self.renderer.get_camera(i)
            height, width = camera.get_height(), camera.get_width()
            self.mount_actor_list.append(None)
            self.camera_name_list.append(camera.get_name())
            self.cam_list.append(camera)
            self.__build_camera_mapping(height, width, camera.get_camera_matrix())
            self.depth_lambda_list.append(
                lambda depth: 1 / (depth * (1 / camera.far - 1 / camera.near) + 1 / camera.near))

            # Build OpenGL camera mapping
            width_points = np.repeat(np.linspace(1 / (2 * width), 1 - 1 / (2 * width), width).reshape([1, -1]), height,
                                     axis=0)
            height_points = np.repeat(
                np.linspace(1 / (2 * height), 1 - 1 / (2 * height), height)[::-1].reshape([-1, 1]),
                width, axis=1)

            points = np.stack([width_points, height_points], 2) * 2 - 1
            homo_padding = np.ones_like(width_points) * 2 - 1
            self.__gl_camera_mapping.append((points, homo_padding[:, :, np.newaxis]))
            self._gl_projection_inv.append(np.linalg.inv(camera.get_projection_mat()))

    def add_camera(self, name: str, camera_pose: Union[np.ndarray, sapyen.Pose], width: int, height: int, fov=1.1,
                   near=0.01, far=100) -> None:
        """
        Add custom mounted camera to the scene. These camera have same property as the urdf-defined camera
        :param name: Name of the camera, used for get camera name and may be used for namespace of topic if ROS enabled
        :param camera_pose: (4, 4) transformation matrix to define the pose of camera. Camera point forward along
            positive x-axis, the y-axis and z-axis accounts for the width and height of the image captured by camera
        :param width: The width of the camera, e.g. 1920 in the (1920, 1080) hd image
        :param height: The height of the camera, e.g. 1080 in the (1920, 1080) hd image
        :param fov: Field of view angle in arc. Note the fov is the full angle of view, not half angle. Currently,
            we do not support differernt fov for x and y axis and thus we can only render square pixel
        :param near: Minimum distance camera can observe, it will influence all texture channel
        :param far: Maximum distance camera can observe, it will influence all texture channel
        """
        actor = self.builder.build(False, True, "{}".format(name), True)
        self.mount_actor_list.append(actor)
        self.camera_name_list.append(name)

        if isinstance(camera_pose, np.ndarray):
            assert camera_pose.shape == (4, 4), "Camera pose matrix must be (4, 4)"
            pose = mat2transform(camera_pose)
        elif isinstance(camera_pose, sapyen.Pose):
            pose = camera_pose
            camera_pose = transform2mat(pose)
        else:
            raise RuntimeError("Unknown format of camera pose: {}".format(type(camera_pose)))

        self.sim.add_mounted_camera(name, actor, sapyen.Pose([0, 0, 0], [1, 0, 0, 0]), width, height, fov, fov,
                                    near, far)
        actor.set_global_pose(pose)

        camera = self.renderer.get_camera(len(self.cam_list))
        self.cam_list.append(camera)
        self.__build_camera_mapping(height, width, camera.get_camera_matrix())
        self.depth_lambda_list.append(lambda depth: 1 / (depth * (1 / far - 1 / near) + 1 / near))
        self.camera_frame_id.append("/base_link")
        self.camera_pose.append((camera_pose @ CAMERA_TO_LINK).astype(np.float32))

        # Build OpenGL camera mapping
        width_points = np.repeat(np.linspace(1 / (2 * width), 1 - 1 / (2 * width), width).reshape([1, -1]), height,
                                 axis=0)
        height_points = np.repeat(np.linspace(1 / (2 * height), 1 - 1 / (2 * height), height)[::-1].reshape([-1, 1]),
                                  width, axis=1)

        points = np.stack([width_points, height_points], 2) * 2 - 1
        homo_padding = np.ones_like(width_points) * 2 - 1
        self.__gl_camera_mapping.append((points, homo_padding[:, :, np.newaxis]))
        self._gl_projection_inv.append(np.linalg.inv(camera.get_projection_mat()))

    def __build_camera_mapping(self, height: int, width: int, camera_matrix: np.ndarray):
        """
        Build camera mapping matrix which maps depth to xyz point cloud
        :param height: Height of camera
        :param width: Width of camera
        :param camera_matrix: Camera intrinsic matrix
        :return:
        """
        x = np.linspace(0.5, width - 0.5, width)
        y = np.linspace(0.5, height - 0.5, height)
        x, y = np.meshgrid(x, y)
        cor = np.stack([x.flatten(), y.flatten(), np.ones([x.size])], axis=0)
        mapping = np.linalg.inv(camera_matrix[:3, :3]) @ cor
        self.mapping_list.append(np.reshape(mapping.T, [height, width, 3]).astype(np.float32))

    @property
    def mounted_camera_names(self) -> List[str]:
        """
        Names of all existing camera in order
        """
        return self.camera_name_list.copy()

    def camera_name2id(self, name: str) -> int:
        """
        Get camera id by name
        :param name: camera name
        :return: camera id or -1 if not found
        """
        if name in self.camera_name_list:
            return self.camera_name_list.index(name)
        else:
            warnings.warn("Camera name {} not found, valid camera names: {}".format(name, self.camera_name_list))
            return -1

    def render_point_cloud(self, cam_id: int, xyz: bool = True, rgba: bool = True, normal: bool = True,
                           segmentation: bool = True, world_coordinate: bool = True) -> np.ndarray:
        """
        Render all the thins expect depth map for the channel enabled
        :param cam_id: Camera id
        :param xyz: Whether to render xyz point cloud
        :param rgba: Whether to render rgba
        :param normal: Whether to render normal
        :param segmentation: Whether to render segmentation mask
        :param world_coordinate: Whether to use world coordinate
        :return: array of all the enabled channels with shape (width, height, channels)
        """
        assert (xyz or rgba or normal or segmentation), "You can not call rendering function with nothing to output"
        camera = self.cam_list[cam_id]
        camera.take_picture()
        result = []

        # W, H = depth.shape
        #
        # WS = np.repeat(np.linspace(1 / (2 * W), 1 - 1 / (2 * W), W).reshape([1, -1]), H, axis=0)
        # HS = np.repeat(np.linspace(1 / (2 * H), 1 - 1 / (2 * H), H)[::-1].reshape([-1, 1]), W, axis=1)
        # points = np.stack([WS, HS, depth, np.ones_like(depth)], 2)
        #
        # color = color[depth < 1]
        # points = points[depth < 1]
        # points = points * 2 - 1
        # cam_points = np.linalg.inv(proj) @ points.T
        # cam_points /= cam_points[3]
        #
        # world_points = model @ cam_points
        # world_points = world_points.T
        # print(model)
        #
        # return world_points[:, :3], color[:, :3]

        if xyz:
            depth = camera.get_depth()[:, :, np.newaxis] * 2 - 1
            points = np.concatenate([self.__gl_camera_mapping[cam_id][0], depth, self.__gl_camera_mapping[cam_id][1]],
                                    axis=2).astype(np.float32)
            cam_points = self._gl_projection_inv[cam_id] @ (points.transpose([2, 0, 1]).reshape(4, -1))
            cam_points /= cam_points[3]

            if world_coordinate:
                cam_points = camera.get_model_mat() @ cam_points
            result.append(
                cam_points.reshape([4, camera.get_height(), camera.get_width()]).transpose([1, 2, 0])[:, :, :3])

            # depth = self.depth_lambda_list[cam_id](camera.get_depth())[:, :, np.newaxis].astype(np.float32)
            # result.append(self.mapping_list[cam_id] * depth)

        if rgba:
            color = camera.get_color_rgba()
            result.append(color)

        if normal:
            normal = camera.get_normal_rgba()[:,:,:3]
            if world_coordinate:
                shape = normal.shape
                normal = camera.get_model_mat()[:3, :3] @ normal.reshape(-1, 3).T
                normal = normal.T.reshape(shape[0], shape[1], 3)
            result.append(normal[:, :, :3])

        if segmentation:
            seg = camera.get_segmentation()
            result.append(seg[:, :, np.newaxis])

        return np.concatenate(result, axis=2)


class SapienSingleObjectEnv(BaseEnv):
    def __init__(self, dataset_dir: str, data_id: Union[int, str], on_screening_rendering: bool):
        """
        Sapien environment with single sapien object
        :param dataset_dir: Path of dataset directory
        :param data_id: Data ID of the sapien object
        :param on_screening_rendering: Whether to use rendering visualization or not
        """
        BaseEnv.__init__(self, on_screening_rendering)
        part_dir = os.path.join(dataset_dir, str(data_id))
        urdf = os.path.join(part_dir, "mobility.urdf")

        # By default, objects except robot will not balance passive force automatically, e.g. gravity
        self.loader.fix_loaded_object = True
        self.loader.balance_passive_force = False
        self.object: sapyen.ArticulationWrapper = self.loader.load(urdf)
        # self.object.set_root_pose([3, 0, 0], [1, 0, 0, 0])
        self.__urdf_file = urdf

        # Get the mapping for link_name, link_id, semantic_name
        self.__build_object_semantic_mapping(part_dir=part_dir)

    def __repr__(self):
        return f"Sapien Single Object Environment with object loaded from {self.__urdf_file}"

    @property
    def object_joint_names(self) -> List[str]:
        return self.object.get_joint_names()

    @property
    def object_link_names(self) -> List[str]:
        return self.object.get_link_names()

    @property
    def object_links(self) -> List[sapyen.PxRigidBody]:
        return self.object.get_links()

    @property
    def object_link_segmentation_ids(self) -> List[int]:
        return self.object.get_link_ids()

    @property
    def object_link_motions(self) -> List[str]:
        return self.__object_link_motion

    @property
    def object_link_semantics(self) -> List[str]:
        return self.__object_link_semantics

    def object_name2link(self, name: str) -> sapyen.PxRigidBody:
        return self.__object_name2link[name]

    def object_segmentation_id2semantics(self, segmentation_id: int) -> str:
        return self.__object_segmentation_id2semantics[segmentation_id]

    def object_segmentation_id2link(self, segmentation_id: int) -> sapyen.PxRigidBody:
        return self.__object_segmentation_id2links[segmentation_id]

    def get_object_link_center_global_pose_by_name(self, name: str) -> sapyen.Pose:
        return self.object_name2link(name).get_global_mass_center()

    def get_object_link_center_local_pose_by_name(self, name: str) -> sapyen.Pose:
        return self.object_name2link(name).get_local_mass_center()

    def __build_object_semantic_mapping(self, part_dir: str):
        """
        Build internally stored cache for objects
        :param part_dir:
        :return:
        """
        semantics = os.path.join(part_dir, 'semantics.txt')
        link2semantics = {'base': 'root'}
        link2motion = {'base': 'fixed'}
        id2link_name = dict(zip(self.object.get_link_ids(), self.object.get_link_names()))
        with open(semantics, 'r') as f:
            for line in f:
                if line.strip():
                    link, motion, semantics = line.split()
                    link2semantics[link] = semantics
                    link2motion[link] = motion

        link_names = self.object.get_link_names()
        links = self.object.get_links()
        self.__object_link_semantics = [link2semantics[link] for link in link_names]
        self.__object_link_motion = [link2motion[link] for link in link_names]
        self.__object_name2link = {link_names[i]: links[i] for i in range(len(link_names))}
        self.__object_segmentation_id2semantics = {i: link2semantics[id2link_name[i]] for i in
                                                   self.object_link_segmentation_ids}
        self.__object_segmentation_id2links = {i: self.__object_name2link[id2link_name[i]] for i in
                                               self.object_link_segmentation_ids}

    def apply_general_force_torque(self, link_index: int, force_array: Union[np.ndarray, List]) -> None:
        """
        Apply force and torque to a object link, this function can not be used to actuate robot
        :param link_index: Index of object link
        :param force_array: 6d array for force and torque, xyz convention and defined in global coordinate
        """
        assert len(force_array) == 6
        link = self.object_links[link_index]
        link.add_force(force_array)

    @staticmethod
    def __apply_force_to_link_at_position(link: sapyen.PxRigidBody, position: np.ndarray, direction: np.ndarray):
        """
        Apply a magic force to a global position, where the link should be specified to avoid ambiguity
        :param link: Link handle of the target
        :param position: Position where the force is applied
        :param direction: Direction in global coordinate, where its norm is the magnitude
        """
        link_mass_center = link.get_global_mass_center()
        point_to_center = position - link_mass_center.p
        torque = np.cross(point_to_center, direction)
        link.add_force(np.concatenate([direction, torque]))

    def apply_force_to_object_at_rendering_position(self, cam_id: int, pos: np.ndarray, direction: np.ndarray) -> None:
        """
        Apply a magic force to a position in the camera rendering layer, to avoid ambiguity
        :param cam_id: ID of camera
        :param pos: 2d index in the camera pixel or point cloud index, it should be a 2d int vector
        :param direction: Direction in global coordinate, where its norm is the magnitude
        """
        camera = self.cam_list[cam_id]

        # Get segmentation id
        seg = camera.get_segmentation()
        segmentation_id = seg[pos]
        if segmentation_id not in self.object_link_segmentation_ids:
            warnings.warn("The place you apply force is not a valid object, e.g. robot or ground")
            warnings.warn("No force will be apply")

        depth = self.depth_lambda_list[cam_id](camera.get_depth())[:, :, np.newaxis].astype(np.float32)
        position = depth[pos, :] * self.mapping_list[cam_id][pos, :]
        link = self.object_segmentation_id2link(segmentation_id)
        self.__apply_force_to_link_at_position(link, position, direction)

    def calculate_pose_in_front_of_semantics(self, semantic_name: str, category: str = "StorageFurniture",
                                             horizontal_offset: float = 0.6,
                                             vertical_offset: float = -0.3) -> sapyen.Pose:
        """
        Get a heuristic pose which locate in front of the semantic part
        :param semantic_name: Semantic name
        :param category: Category of the object
        :param horizontal_offset: How far does the target pose be in front of the semantic part
        :param vertical_offset: How far does the target pose be in above the semantic part
        :return: Target pose
        """
        link_index = self.object_link_semantics.index(semantic_name)
        return self.calculate_pose_in_front_of_object_link(link_index, category, horizontal_offset, vertical_offset)

    def calculate_pose_in_front_of_object_link(self, link_index: int, category: Optional[str], horizontal_offset: float,
                                               vertical_offset: float) -> sapyen.Pose:
        """
        Get a heuristic pose which locate in front of a object link
        :param link_index: Index of a link in the object link list
        :param category: Category of the object
        :param horizontal_offset: How far does the target pose be in front of the semantic part
        :param vertical_offset: How far does the target pose be in above the semantic part
        :return: Target pose
        """
        object_pose: sapyen.Pose = self.object_links[0].get_global_pose()
        semantic_local_pose: sapyen.Pose = object_pose.inv().transform(
            self.object_links[link_index].get_global_mass_center())
        if semantic_local_pose.p[0] > 0:
            target_x = semantic_local_pose.p[0] + horizontal_offset
            target_direction_quaternion = [0, 0, 0, 1]
        else:
            target_x = semantic_local_pose.p[0] - horizontal_offset
            target_direction_quaternion = [1, 0, 0, 0]

        target_pos = semantic_local_pose.p
        target_pos[0] = target_x
        target_pos[2] += vertical_offset
        target_pose = sapyen.Pose(target_pos, target_direction_quaternion)
        return object_pose.transform(target_pose)

    def get_global_part_point_cloud_with_seg_id(self, cam: [int, str], part_name: str, seg_id: int) -> np.ndarray:
        if isinstance(cam, str):
            cam = self.camera_name2id(cam)

        pc = self.render_point_cloud(cam, xyz=True, rgba=False, normal=False, segmentation=True, world_coordinate=True)
        segmentation = pc[:, :, 3]
        found_link_bool = segmentation == seg_id
        if np.sum(found_link_bool) == 0:
            warnings.warn(f"Object with segmentation ID:{seg_id} not found.")
            return np.zeros([0, 3])

        obj_segmentation = self.cam_list[cam].get_obj_segmentation()
        obj_ids = obj_segmentation[found_link_bool]
        obj_ids_set = np.unique(obj_ids)
        part_id_set = [obj_id for obj_id in obj_ids_set if re.search(part_name, self.obj_id2name(obj_id))]

        if len(part_id_set) == 0:
            warnings.warn(f"Part with obj name :{part_name} not found")
            return np.zeros([0, 3])

        found_obj_index = []
        for obj_id in part_id_set:
            found_obj_index.append(np.nonzero(obj_id == obj_segmentation[found_link_bool])[0])
        all_obj_index = np.concatenate(found_obj_index)

        part_pc = pc[:, :, :3][found_link_bool][all_obj_index]
        return part_pc

    def get_mean_normal_with_seg_id(self, cam_id: int, seg_id: int):
        point_cloud = self.render_point_cloud(cam_id, xyz=False, rgba=False, segmentation=True, normal=True,
                                              world_coordinate=True)
        normal = point_cloud[:, :, 0:3]
        segmentation = point_cloud[:, :, 3]

        valid_bool = seg_id == segmentation
        mean_normal = np.mean(normal[valid_bool], axis=0)
        mean_normal /= np.linalg.norm(mean_normal)
        return mean_normal

    def _calculate_force_to_joint_given_acceleration(self, acceleration: float,
                                                     joint_index: List[int] = None) -> np.ndarray:
        # TODO: parallel axis theorem
        if not joint_index:
            dof = self.object.dof()
            joint_index = list(range(dof))

        link_joint_index = self.object.get_link_joint_indices()
        force_array = np.zeros(len(joint_index))
        for i in joint_index:
            link_index = link_joint_index.index(i)
            link = self.object_links[link_index]
            link_inertia = link.get_inertia()
            link_mass_pose = transform2mat(link.get_global_mass_center())[0:3, 0:3]
            joint_pose = transform2mat(self.object.get_link_joint_pose(link_index))[0:3, 1:2]
            inertia_frac = np.abs(link_mass_pose @ joint_pose)  # (3,1)
            inertia = np.squeeze(link_inertia[np.newaxis, :] @ inertia_frac)
            force_array[i] = inertia * acceleration
        return -force_array
