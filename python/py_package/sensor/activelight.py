from ..core import (
    KuafuRenderer,
    ActorBase,
    ActorStatic,
    Pose,
    Scene,
    ArticulationBase,
    CameraEntity,
)

from .depth_processor import calc_main_depth_from_left_right_ir
from .sensor_base import SensorEntity

from typing import Optional, Tuple
from copy import deepcopy as copy
import os

import numpy as np

import transforms3d as t3d


class ActiveLightSensor(SensorEntity):
    def __init__(self,
                 sensor_name: str,
                 renderer: KuafuRenderer,
                 scene: Scene,
                 sensor_type: Optional[str] = 'fakesense_j415',
                 rgb_resolution: Tuple[int, int] = None,
                 ir_resolution: Tuple[int, int] = None,
                 rgb_intrinsic: Optional[np.ndarray] = None,
                 ir_intrinsic: Optional[np.ndarray] = None,
                 trans_pose_l: Optional[Pose] = None,
                 trans_pose_r: Optional[Pose] = None,
                 light_pattern: Optional[str] = None,
                 max_depth: float = 8.0,
                 min_depth: float = 0.3,
                 ir_ambient_strength: float = 0.002):
        """

        :param sensor_name: Name of the sensor
        :param renderer:
        :param scene:

        :param sensor_type: If this is set, all the parameters below will be omitted.
                            Supported sensor types: ['fakesense_j415']
        :param rgb_resolution:
        :param ir_resolution:
        :param rgb_intrinsic:
        :param ir_intrinsic:
        :param trans_pose_l:
        :param trans_pose_r:
        :param light_pattern: Path to active light pattern file.
                              Use rgb modality if set to None.
        """

        assert isinstance(renderer, KuafuRenderer), \
            "KuafuRenderer is required for active light sensor simulation"
        super().__init__()

        from warnings import warn
        warn('Current implementation of ActiveLightSensor is incompatible with emissive objects.')
        warn('Current implementation of ActiveLightSensor is incompatible with environment maps.')

        self.name = sensor_name
        # super().set_name(sensor_name)

        self.renderer = renderer
        self.scene = scene
        self.mount = self.scene.create_actor_builder().build_kinematic(name=f'{sensor_name}_mount')

        if sensor_type:
            self._set_sensor_parameters(sensor_type)
        else:
            self.rgb_w, self.rgb_h = rgb_resolution
            self.ir_w, self.ir_h = ir_resolution
            self.rgb_intrinsic = rgb_intrinsic
            self.ir_intrinsic = ir_intrinsic
            self.trans_pose_l = trans_pose_l
            self.trans_pose_r = trans_pose_r
            self.light_pattern = light_pattern
            self.max_depth = max_depth
            self.min_depth = min_depth

        self.pose = Pose()

        self._create_cameras()
        self.alight = self.scene.add_active_light(
            pose=Pose([0, 0, 0]),
            color=[0, 0, 0],
            fov=1.57,                      # TODO: parameterize this
            tex_path=self.light_pattern,
        )
        self.ir_ambient_strength = ir_ambient_strength

        self.set_pose(Pose())

        self._rgb = None
        self._ir_l = None
        self._ir_r = None
        self._depth = None
        self._pc = None

    def clear_cache(self):
        self._rgb = None
        self._ir_l = None
        self._ir_r = None
        self._depth = None
        self._pc = None

    def set_pose(self, pose: Pose):
        # super().set_pose(pose)
        self.pose = pose
        self.mount.set_pose(pose)
        apos = t3d.quaternions.mat2quat(self.mount.get_pose().to_transformation_matrix()[:3, :3]
                                        @ t3d.quaternions.quat2mat((-0.5, 0.5, 0.5, -0.5)))
        self.alight.set_pose(Pose(self.mount.get_pose().p, apos))
        self.clear_cache()

    def get_pose(self):
        return copy(self.pose)

    def take_picture(self):
        """
        Note: we expect one scene.update_render call before take_picture
        """
        self.clear_cache()

        self.cam_rgb.take_picture()

        self._ir_mode()
        self.scene.update_render()
        self.cam_ir_l.take_picture()
        self.cam_ir_r.take_picture()

        self._normal_mode()
        self.scene.update_render()

    def get_rgb(self):
        self._fetch('rgb')
        return copy(self._rgb)

    def get_ir(self):
        self._fetch('ir_l')
        self._fetch('ir_r')
        return [copy(self._ir_l), copy(self._ir_r)]

    def get_depth(self):
        if self._depth is None:
            self._fetch('ir_l')
            self._fetch('ir_r')

            ex_l = self._pose2cv2ex(self.trans_pose_l)
            ex_r = self._pose2cv2ex(self.trans_pose_r)
            ex_main = self._pose2cv2ex(Pose())

            depth = calc_main_depth_from_left_right_ir(
                self._float2uint8(self._ir_l),
                self._float2uint8(self._ir_r),
                ex_l, ex_r, ex_main,
                self.ir_intrinsic, self.ir_intrinsic, self.rgb_intrinsic,
                lr_consistency=False, main_cam_size=(self.rgb_w, self.rgb_h),
                ndisp=128, use_census=True, register_depth=True, census_wsize=7,
                use_noise=False)
            depth[depth > self.max_depth] = 0
            depth[depth < self.min_depth] = 0
            self._depth = depth

        return copy(self._depth)

    def get_pointcloud(self, frame='camera', with_rgb=False):
        assert frame in ['camera', 'world']
        depth = self.get_depth()
        if frame == 'camera':
            xyz = self._depth2pts_np(depth, self.rgb_intrinsic)
        else:
            xyz = self._depth2pts_np(depth, self.rgb_intrinsic, self._pose2cv2ex(self.pose))

        if with_rgb:
            self._fetch('rgb')
            xyz = np.concatenate([xyz, self._rgb.reshape(-1, 3)], axis=1)

        return xyz

    def _set_sensor_parameters(self, sensor_type):
        if sensor_type == 'fakesense_j415':
            self.rgb_w, self.rgb_h = (1920, 1080)
            self.ir_w, self.ir_h = (1280, 720)
            self.rgb_intrinsic = np.array([
                [1380.,    0., 960.],
                [0.,    1380., 540.],
                [0.,       0.,   1.]
            ])
            self.ir_intrinsic = np.array([
                [920.,   0., 640.],
                [0.,   920., 360.],
                [0.,     0.,   1.]
            ])
            self.trans_pose_l = Pose([0, -0.0175, 0])
            self.trans_pose_r = Pose([0, -0.0720, 0])
            self.light_pattern = os.path.join(os.path.dirname(__file__), 'assets/patterns/fakesense_j415.png')
            self.max_depth = 10.0
            self.min_depth = 0.2
        else:
            assert False, f"Unsupported sensor type: {sensor_type}"

    def _create_cameras(self):
        self.scene.update_render()

        self.cam_rgb = self.scene.add_mounted_camera(
            f"{self.name}", self.mount, Pose([0, 0, 0]), self.rgb_w, self.rgb_h, 0.78, 0.001, 100)
        self.cam_rgb.set_perspective_parameters(
            0.1, 100.0,
            self.rgb_intrinsic[0, 0], self.rgb_intrinsic[1, 1],
            self.rgb_intrinsic[0, 2], self.rgb_intrinsic[1, 2],
            self.rgb_intrinsic[0, 1]
        )

        self.cam_ir_l = self.scene.add_mounted_camera(
            f"{self.name}_left", self.mount, self.trans_pose_l, self.ir_w, self.ir_h, 0.78, 0.001, 100)
        self.cam_ir_l.set_perspective_parameters(
            0.1, 100.0,
            self.ir_intrinsic[0, 0], self.ir_intrinsic[1, 1],
            self.ir_intrinsic[0, 2], self.ir_intrinsic[1, 2],
            self.ir_intrinsic[0, 1]
        )

        self.cam_ir_r = self.scene.add_mounted_camera(
            f"{self.name}_right", self.mount, self.trans_pose_r, self.ir_w, self.ir_h, 0.78, 0.001, 100)
        self.cam_ir_r.set_perspective_parameters(
            0.1, 100.0,
            self.ir_intrinsic[0, 0], self.ir_intrinsic[1, 1],
            self.ir_intrinsic[0, 2], self.ir_intrinsic[1, 2],
            self.ir_intrinsic[0, 1]
        )

    def _ir_mode(self):
        if self.light_pattern is None:
            return
        else:
            self._light_d = {}
            for l in self.scene.get_all_lights():
                self._light_d[l] = l.color
                l.set_color([0, 0, 0])

            self._light_a = self.scene.ambient_light
            self.scene.set_ambient_light([self.ir_ambient_strength, 0, 0])
            self.alight.set_color([1, 0, 0])

    def _normal_mode(self):
        if self.light_pattern is None:
            return
        else:
            for l in self.scene.get_all_lights():
                l.set_color(self._light_d[l])
            self.scene.set_ambient_light(self._light_a)
            self.alight.set_color([0, 0, 0])

    @staticmethod
    def _float2uint8(x):
        return (x * 255).clip(0, 255).astype(np.uint8)

    def _fetch(self, mod):
        if mod == 'rgb':
            if self._rgb is None:
                self._rgb = self.cam_rgb.get_color_rgba()[..., :3].clip(0, 1)
        elif mod == 'ir_l':
            if self._ir_l is None:
                self._ir_l = self.cam_ir_l.get_color_rgba()[..., 0].clip(0, 1)
        elif mod == 'ir_r':
            if self._ir_r is None:
                self._ir_r = self.cam_ir_r.get_color_rgba()[..., 0].clip(0, 1)

    @staticmethod
    def _pose2cv2ex(pose):
        ros2opencv = np.array([[0., -1., 0., 0.],
                               [0., 0., -1., 0.],
                               [1., 0., 0., 0.],
                               [0., 0., 0., 1.]], dtype=np.float32)
        return ros2opencv @ np.linalg.inv(pose.to_transformation_matrix())

    @staticmethod
    def _get_pixel_grids_np(height, width):
        x_linspace = np.linspace(0.5, width - 0.5, width)
        y_linspace = np.linspace(0.5, height - 0.5, height)
        x_coordinates, y_coordinates = np.meshgrid(x_linspace, y_linspace)
        x_coordinates = np.reshape(x_coordinates, (1, -1))
        y_coordinates = np.reshape(y_coordinates, (1, -1))
        ones = np.ones_like(x_coordinates).astype(np.float)
        grid = np.concatenate([x_coordinates, y_coordinates, ones], axis=0)
        return grid

    @staticmethod
    def _depth2pts_np(depth_map, cam_intrinsic, cam_extrinsic=np.eye(4)):
        feature_grid = ActiveLightSensor._get_pixel_grids_np(
            depth_map.shape[0], depth_map.shape[1])

        uv = np.matmul(np.linalg.inv(cam_intrinsic), feature_grid)
        cam_points = uv * np.reshape(depth_map, (1, -1))

        r = cam_extrinsic[:3, :3]
        t = cam_extrinsic[:3, 3:4]
        r_inv = np.linalg.inv(r)

        world_points = np.matmul(r_inv, cam_points - t).transpose()
        return world_points
