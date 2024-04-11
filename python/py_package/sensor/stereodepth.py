from ..pysapien import Pose, Entity
from ..pysapien.render import (
    RenderCameraComponent,
    RenderTexturedLightComponent,
    RenderTexture2D,
)
from typing import Optional
from copy import deepcopy as copy

import os
import numpy as np


class StereoDepthSensorConfig:
    """
    An instance of this class is required to initialize StereoDepthSensor.
    """

    def __init__(self):
        self.light_pattern = os.path.join(
            os.path.dirname(__file__), "assets/patterns/d415.png"
        )
        """Path to active light pattern file. Use RGB modality if set to None."""

        self.ir_camera_exposure = 0.01
        # """Camera exposure for infrared cameras."""

        self.rgb_resolution = (1920, 1080)
        """Resolution of the RGB camera (width x height)."""

        self.ir_resolution = (1280, 720)
        """Resolution of the infrared cameras (width x height)."""

        self.rgb_intrinsic = np.array(
            [[1380.0, 0.0, 960.0], [0.0, 1380.0, 540.0], [0.0, 0.0, 1.0]]
        )
        """Intrinsic matrix of the RGB camera."""

        self.ir_intrinsic = np.array(
            [[920.0, 0.0, 640.0], [0.0, 920.0, 360.0], [0.0, 0.0, 1.0]]
        )
        """Intrinsic matrix of the infrared cameras."""

        self.trans_pose_l = Pose([0, -0.0175, 0])
        """Relative pose of the left infrared camera to the RGB camera."""

        self.trans_pose_r = Pose([0, -0.0720, 0])
        """Relative pose of the right infrared camera to the RGB camera."""

        self.min_depth = 0.2
        """Minimum valid depth in meters."""

        self.max_depth = 10.0
        """Maximum valid depth (non-inclusive) in meters."""

        self.ir_noise_seed = 0
        """Random seed for simulating infrared noise."""

        self.ir_speckle_noise = 1.0
        """Scale for simulating infrared speckle noise. Set to 0 will disable noise simulation."""

        self.ir_thermal_noise = 1.0
        """Scale for simulating infrared thermal noise. Only effective when speckle noise is on."""

        self.rectified = True
        """Set to true if left and right infrared cameras have a common image plane,
        otherwise false."""

        self.census_width = 7
        """Width of the center-symmetric census transform window for steoreo matching.
        This must be an odd number."""

        self.census_height = 7
        """Height of the center-symmetric census transform window for stereo matching.
        This must be an odd number."""

        self.max_disp = 128
        """Maximum disparity (non-inclusive) for stereo matching."""

        self.block_width = 7
        """Width of the matched block for stereo matching. This must be an odd number."""

        self.block_height = 7
        """Height of the matched block for stereo matching. This must be an odd number."""

        self.p1_penalty = 8
        """P1 penalty for semi-global matching algorithm."""

        self.p2_penalty = 32
        """P2 penalty for semi-global matching algorithm."""

        self.uniqueness_ratio = 15
        """How much (in percentage) the minimum cost computed in stereo matching must exceed
        the second best cost (ignoring best match's adjacent pixels) to be considered valid."""

        self.lr_max_diff = 1
        """Maximum allowed difference of the left-right consistency check in stereo matching.
        Set it to 255 will disable the check."""

        self.median_filter_size = 3
        """Size of the median filter for disparity post-processing. Choices are 1, 3, 5, 7.
        Set to 1 will disable median filter."""

        self.depth_dilation = True
        """Final depth will be transformed into the same size of rgb_resolution, and depth dilation
        can dilate the final depth map to avoid holes. Recommended to set as true if rgb_resolution
        is greater than ir_resolution."""


class StereoDepthSensor:
    """
    This class simulates an active stereo depth sensor. It has one RGB camera and two infrared camera.
    Depth is computed via semi-global block matching. Refer to StereoDepthSensorConfig for configurable
    parameters. The computed depth map will be presented in RGB camera frame.
    """

    def __init__(
        self,
        config: StereoDepthSensorConfig,
        mount_entity: Entity,
        pose: Optional[Pose] = None,
    ):
        """
        :param config: configuration of the sensor.
        :param mount_entity: entity that the sensor is mounted to.
        :param pose: local pose relative to the mounted entity. If not given, the relative trasformation will be identity transformation.
        """

        super().__init__()

        from .simsense_component import SimSenseComponent

        # Basic configuration
        self._config = config

        self._mount = mount_entity
        if pose is None:
            self._pose = Pose()
        else:
            self._pose = pose

        # Cameras
        self._cam_rgb = None
        self._cam_ir_l = None
        self._cam_ir_r = None
        self._create_cameras()

        # Active light
        self._alight = None
        self._create_light()

        # Simsense component
        self._ss = SimSenseComponent(
            config.rgb_resolution,
            config.ir_resolution,
            config.rgb_intrinsic,
            config.ir_intrinsic,
            config.trans_pose_l,
            config.trans_pose_r,
            config.min_depth,
            config.max_depth,
            config.ir_noise_seed,
            config.ir_speckle_noise,
            config.ir_thermal_noise,
            config.rectified,
            config.census_width,
            config.census_height,
            config.max_disp,
            config.block_width,
            config.block_height,
            config.p1_penalty,
            config.p2_penalty,
            config.uniqueness_ratio,
            config.lr_max_diff,
            config.median_filter_size,
            config.depth_dilation,
        )
        self._mount.add_component(self._ss)

    def take_picture(self, infrared_only: bool = False):
        """
        Note: We expect one scene.update_render() call before calling take_picture().

        :param infrared_only: If true, only take infrared pictures without taking RGB picture.
        """
        scene = self._mount.get_scene()
        if scene is None:
            raise RuntimeError("Cannot take picture: sensor is not added to scene")

        if not infrared_only:
            self._cam_rgb.take_picture()
        self._ir_mode()
        scene.update_render()
        self._cam_ir_l.take_picture()
        self._cam_ir_r.take_picture()
        self._normal_mode()
        scene.update_render()

    def compute_depth(self, bbox_start: tuple = None, bbox_size: tuple = None):
        left_cuda = self._cam_ir_l.get_picture_cuda("Color")
        right_cuda = self._cam_ir_r.get_picture_cuda("Color")
        self._ss.compute(left_cuda, right_cuda, bbox_start, bbox_size)

    def set_local_pose(self, pose: Pose):
        """
        Set local pose of the sensor relative to mounted actor.
        """
        self._pose = pose
        self._alight.local_pose = pose
        self._cam_rgb.local_pose = pose
        self._cam_ir_l.local_pose = pose * self._config.trans_pose_l
        self._cam_ir_r.local_pose = pose * self._config.trans_pose_r

    def set_ir_noise(self, ir_speckle_noise: float, ir_thermal_noise: float):
        """
        :param ir_speckle_noise: Scale for simulating infrared speckle noise. Set to 0 will disable noise simulation.
        :param ir_thermal_noise: Scale for simulating infrared thermal noise. Only effective when speckle noise is on.
        """
        if ir_speckle_noise > 0 and ir_thermal_noise <= 0:
            raise TypeError(
                "ir_speckle_noise > 0, Infrared noise simulation is on. ir_thermal_noise must also be positive"
            )
        if ir_speckle_noise == 0:
            speckle_shape = 0
        else:
            speckle_shape = self._default_speckle_shape / ir_speckle_noise
        speckle_scale = ir_speckle_noise / self._default_speckle_shape
        gaussian_mu = self._default_gaussian_mu
        gaussian_sigma = self._default_gaussian_sigma * ir_thermal_noise

        self._config.ir_speckle_noise = ir_speckle_noise
        self._config.ir_thermal_noise = ir_thermal_noise
        self._ss._engine.set_ir_noise_parameters(
            speckle_shape, speckle_scale, gaussian_mu, gaussian_sigma
        )

    def set_census_window_size(self, census_width: int, census_height: int):
        """
        :param census_width: Width of the center-symmetric census transform window. This must be an odd number.
        :param census_height: Height of the center-symmetric census transform window. This must be an odd number.
        """
        if (
            not isinstance(census_width, int)
            or not isinstance(census_height, int)
            or census_width <= 0
            or census_height <= 0
            or census_width % 2 == 0
            or census_height % 2 == 0
            or census_width * census_height > 65
        ):
            raise TypeError(
                "census_width and census_height must be positive odd integers and their product should be no larger than 65"
            )
        self._config.census_width = census_width
        self._config.census_height = census_height
        self._ss._engine.set_census_window_size(census_width, census_height)

    def set_matching_block_size(self, block_width: int, block_height: int):
        """
        :param block_width: Width of the matched block. This must be an odd number.
        :param block_height: Height of the matched block. This must be an odd number.
        """
        if (
            not isinstance(block_width, int)
            or not isinstance(block_height, int)
            or block_width <= 0
            or block_height <= 0
            or block_width % 2 == 0
            or block_height % 2 == 0
            or block_width * block_height > 256
        ):
            raise TypeError(
                "block_width and block_height must be positive odd integers and their product should be no larger than 256"
            )
        self._config.block_width = block_width
        self._config.block_height = block_height
        self._ss._engine.set_matching_block_size(block_width, block_height)

    def set_penalties(self, p1_penalty: int, p2_penalty: int):
        """
        :param p1_penalty: P1 penalty for semi-global matching algorithm.
        :param p2_penalty: P2 penalty for semi-global matching algorithm.
        """
        if (
            not isinstance(p1_penalty, int)
            or not isinstance(p2_penalty, int)
            or p1_penalty <= 0
            or p2_penalty <= 0
            or p1_penalty >= p2_penalty
            or p2_penalty >= 224
        ):
            raise TypeError(
                "p1 must be positive integer less than p2 and p2 be positive integer less than 224"
            )
        self._config.p1_penalty = p1_penalty
        self._config.p2_penalty = p2_penalty
        self._ss._engine.set_penalties(p1_penalty, p2_penalty)

    def set_uniqueness_ratio(self, uniqueness_ratio: int):
        """
        :param uniqueness_ratio: Margin in percentage by which the minimum computed cost should win the second best (not considering
                                 best match's adjacent pixels) cost to consider the found match valid.
        """
        if (
            not isinstance(uniqueness_ratio, int)
            or uniqueness_ratio < 0
            or uniqueness_ratio > 255
        ):
            raise TypeError(
                "uniqueness_ratio must be positive integer no larger than 255"
            )
        self._config.uniqueness_ratio = uniqueness_ratio
        self._ss._engine.set_uniqueness_ratio(uniqueness_ratio)

    def set_lr_max_diff(self, lr_max_diff: int):
        """
        :param lr_max_diff: Maximum allowed difference in the left-right consistency check. Set it to 255 to disable the check.
        """
        if not isinstance(lr_max_diff, int) or lr_max_diff < -1 or lr_max_diff > 255:
            raise TypeError("lr_max_diff must be integer within the range [0, 255]")
        self._config.lr_max_diff = lr_max_diff
        self._ss._engine.set_lr_max_diff(lr_max_diff)

    def get_config(self):
        return copy(self._config)

    def get_pose(self):
        return copy(self._mount.get_pose() * self._pose)

    def get_rgb(self):
        rgb = self._cam_rgb.get_picture("Color")[..., :3].clip(0, 1)
        return copy(rgb)

    def get_rgba_cuda(self):
        return self._cam_rgb.get_picture_cuda("Color")

    def get_ir(self):
        """
        Note: Noise simulation won't be reflected here.
        """
        ir_l = self._cam_ir_l.get_picture("Color")[..., 0].clip(0, 1)
        ir_r = self._cam_ir_r.get_picture("Color")[..., 0].clip(0, 1)
        return [copy(ir_l), copy(ir_r)]

    def get_depth(self):
        """
        Note: Returned depth map will be of the same resolution and frame of RGB camera.
        """
        depth = self._ss.get_ndarray()
        return copy(depth)

    def get_depth_cuda(self):
        """
        Note: Returned depth map will be of the same resolution and frame of RGB camera.
        """
        return self._ss.get_cuda()

    def get_pointcloud(self, with_rgb: bool = False):
        """
        Note: Returned point cloud is from RGB camera's with x rightward, y downward, z forward.
        """
        if with_rgb:
            rgba_cuda = self.get_rgba_cuda()
            pc = self._ss.get_rgb_point_cloud_ndarray(rgba_cuda)
        else:
            pc = self._ss.get_point_cloud_ndarray()

        return pc

    def get_pointcloud_cuda(self, with_rgb: bool = False):
        """
        Note: Returned point cloud is from RGB camera's with x rightward, y downward, z forward.
        """
        if with_rgb:
            rgba_cuda = self.get_rgba_cuda()
            pc_cuda = self._ss.get_rgb_point_cloud_cuda(rgba_cuda)
        else:
            pc_cuda = self._ss.get_point_cloud_cuda()

        return pc_cuda

    def _create_cameras(self):
        self._cam_rgb = RenderCameraComponent(*self._config.rgb_resolution)
        self._cam_rgb.local_pose = self._pose
        self._cam_rgb.name = "cam_rgb"
        self._cam_rgb.set_perspective_parameters(
            0.1,
            100.0,
            self._config.rgb_intrinsic[0, 0],
            self._config.rgb_intrinsic[1, 1],
            self._config.rgb_intrinsic[0, 2],
            self._config.rgb_intrinsic[1, 2],
            self._config.rgb_intrinsic[0, 1],
        )

        self._cam_ir_l = RenderCameraComponent(*self._config.ir_resolution)
        self._cam_ir_l.local_pose = self._pose * self._config.trans_pose_l
        self._cam_ir_l.name = "cam_ir_l"
        self._cam_ir_l.set_perspective_parameters(
            0.1,
            100.0,
            self._config.ir_intrinsic[0, 0],
            self._config.ir_intrinsic[1, 1],
            self._config.ir_intrinsic[0, 2],
            self._config.ir_intrinsic[1, 2],
            self._config.ir_intrinsic[0, 1],
        )

        self._cam_ir_r = RenderCameraComponent(*self._config.ir_resolution)
        self._cam_ir_r.local_pose = self._pose * self._config.trans_pose_r
        self._cam_ir_r.name = "cam_ir_r"
        self._cam_ir_r.set_perspective_parameters(
            0.1,
            100.0,
            self._config.ir_intrinsic[0, 0],
            self._config.ir_intrinsic[1, 1],
            self._config.ir_intrinsic[0, 2],
            self._config.ir_intrinsic[1, 2],
            self._config.ir_intrinsic[0, 1],
        )

        self._mount.add_component(self._cam_rgb)
        self._mount.add_component(self._cam_ir_l)
        self._mount.add_component(self._cam_ir_r)

        self._cam_ir_l.set_property("exposure", float(self._config.ir_camera_exposure))
        self._cam_ir_r.set_property("exposure", float(self._config.ir_camera_exposure))

    def _create_light(self):
        # Active Light
        self._alight = RenderTexturedLightComponent()
        self._alight.color = [0, 0, 0]
        self._alight.inner_fov = 1.57
        self._alight.outer_fov = 1.57
        self._alight.texture = RenderTexture2D(self._config.light_pattern)
        self._alight.local_pose = self._pose
        self._alight.name = "active_light"
        self._mount.add_component(self._alight)

    def _ir_mode(self):
        if self._config.light_pattern is None:
            return
        self._alight.color = [100, 0, 0]

    def _normal_mode(self):
        if self._config.light_pattern is None:
            return
        self._alight.color = [0, 0, 0]
