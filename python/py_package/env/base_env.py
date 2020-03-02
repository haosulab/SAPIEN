import re
from typing import List, Union, Optional
from .physx_utils import transform2mat, mat2transform, pysapien
import numpy as np
import warnings
import os

RGBD_CAMERA_THRESHOLD = 10
_CAMERA_TO_LINK = np.zeros([4, 4])
_CAMERA_TO_LINK[[0, 1, 2, 3], [2, 0, 1, 3]] = [1, -1, -1, 1]


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


class BaseEnv:
    def __init__(self, sim: pysapien.Engine):
        """
        Base class of a environment of
        """
        # Scene
        self.sim = sim
        self.scene: pysapien.Scene = sim.create_scene()
        self.scene.add_ground(-1)
        self.scene.set_timestep(1 / 240)

        # Articulation loader for both robot and object, articulation builder for simple object without joint
        self.loader: pysapien.URDFLoader = self.scene.create_urdf_loader()
        self.builder = self.scene.create_actor_builder()

        # Camera
        self.camera_frame_id = []
        self.camera_pose = []
        self.camera_name_list = []
        self.cam_list: List[pysapien.OptifuserCamera] = []
        self.mount_actor_list: List[pysapien.PxRigidActor] = []
        self.mapping_list = []
        self.depth_lambda_list = []
        self.__gl_camera_mapping = []
        self._gl_projection_inv = []

    def update_renderer(self):
        self.scene.update_render()

    def _step(self):
        """
        Step function when on screen visualization is enabled
        """
        self.scene.step()
        self.after_step()

    def after_step(self):
        """
        By default, this function is empty, you can overload this function to add custom step, e.g. video recording
        """
        pass

    def obj_id2name(self, obj_id: int) -> str:
        return self.sim.get_render_name_dict()[obj_id]

    def _init_camera_cache(self):
        """
        Init camera mapping for camera define in urdf file, like camera on robot's head.
        Camera added by user after the class instantiate will not be handle by this function
        :return:
        """
        mounted_cameras: List[pysapien.OptifuserCamera] = self.scene.get_mounted_cameras()
        mounted_actors: List[pysapien.ActorBase] = self.scene.get_mounted_actors()
        num = len(mounted_actors)
        for i in range(num):
            camera = mounted_cameras[i]
            height, width = camera.get_height(), camera.get_width()
            self.mount_actor_list.append(mounted_actors[i])
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
            self._gl_projection_inv.append(np.linalg.inv(camera.get_projection_matrix()))

    def add_camera(self, name: str, camera_pose: Union[np.ndarray, pysapien.Pose], width: int, height: int, fov=1.1,
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

        if isinstance(camera_pose, np.ndarray):
            assert camera_pose.shape == (4, 4), "Camera pose matrix must be (4, 4)"
            pose = mat2transform(camera_pose)
        elif isinstance(camera_pose, pysapien.Pose):
            pose = camera_pose
            camera_pose = transform2mat(pose)
        else:
            raise RuntimeError("Unknown format of camera pose: {}".format(type(camera_pose)))

        camera = self.sim.add_mounted_camera(name, actor, pysapien.Pose([0, 0, 0], [1, 0, 0, 0]), width, height, fov,
                                             fov, near, far)
        actor.set_global_pose(pose)

        self.cam_list.append(camera)
        self.mount_actor_list.append(actor)
        self.camera_name_list.append(name)
        self.__build_camera_mapping(height, width, camera.get_camera_matrix())
        self.depth_lambda_list.append(lambda depth: 1 / (depth * (1 / far - 1 / near) + 1 / near))
        self.camera_frame_id.append("/base_link")
        self.camera_pose.append((camera_pose @ _CAMERA_TO_LINK).astype(np.float32))

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
            normal = camera.get_normal_rgba()[:, :, :3]
            if world_coordinate:
                shape = normal.shape
                normal = camera.get_model_mat()[:3, :3] @ normal.reshape(-1, 3).T
                normal = normal.T.reshape(shape[0], shape[1], 3)
            result.append(normal[:, :, :3])

        if segmentation:
            seg = camera.get_segmentation()
            result.append(seg[:, :, np.newaxis])

        return np.concatenate(result, axis=2)
