from ..pysapien import Component, Pose, CudaArray
from ..pysapien.simsense import DepthSensorEngine

import numpy as np

from typing import Union


class SimSenseComponent(Component):
    def __init__(
        self,
        rgb_resolution: tuple,
        ir_resolution: tuple,
        rgb_intrinsic: np.ndarray,
        ir_intrinsic: np.ndarray,
        trans_pose_l: Pose,
        trans_pose_r: Pose,
        min_depth: float,
        max_depth: float,
        ir_noise_seed: int,
        ir_speckle_noise: float,
        ir_thermal_noise: float,
        rectified: bool,
        census_width: int,
        census_height: int,
        max_disp: int,
        block_width: int,
        block_height: int,
        p1_penalty: int,
        p2_penalty: int,
        uniqueness_ratio: int,
        lr_max_diff: int,
        median_filter_size: int,
        depth_dilation: bool,
    ):
        super().__init__()

        # Instance check
        if not isinstance(rgb_resolution[0], int) or not isinstance(
            rgb_resolution[1], int
        ):
            raise TypeError("RGB resolution (width and height) must be integer")

        if (
            not isinstance(ir_resolution[0], int)
            or not isinstance(ir_resolution[1], int)
            or ir_resolution[0] < 32
            or ir_resolution[1] < 32
        ):
            raise TypeError(
                "Infrared resolution (width and height) must be integer and no less than 32"
            )

        if ir_speckle_noise > 0 and ir_thermal_noise <= 0:
            raise TypeError(
                "ir_speckle_noise > 0, Infrared noise simulation is on. ir_thermal_noise must also be positive"
            )

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

        if not isinstance(max_disp, int) or max_disp < 32 or max_disp > 1024:
            raise TypeError("max_disp must be integer and within range [32, 1024]")

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

        if (
            not isinstance(p1_penalty, int)
            or not isinstance(p2_penalty, int)
            or p1_penalty <= 0
            or p2_penalty <= 0
            or p1_penalty >= p2_penalty
            or p2_penalty >= 224
        ):
            raise TypeError(
                "p1_penalty must be positive integer less than p2_penalty and p2_penalty be positive integer less than 224"
            )

        if (
            not isinstance(uniqueness_ratio, int)
            or uniqueness_ratio < 0
            or uniqueness_ratio > 255
        ):
            raise TypeError(
                "uniqueness_ratio must be positive integer and no larger than 255"
            )

        if not isinstance(lr_max_diff, int) or lr_max_diff < -1 or lr_max_diff > 255:
            raise TypeError("lr_max_diff must be integer and within the range [0, 255]")

        if (
            median_filter_size != 1
            and median_filter_size != 3
            and median_filter_size != 5
            and median_filter_size != 7
        ):
            raise TypeError("Median filter size choices are 1, 3, 5, 7")

        self.rgb_resolution = rgb_resolution
        self.ir_resolution = ir_resolution
        self.rgb_intrinsic = rgb_intrinsic
        self.ir_intrinsic = ir_intrinsic
        self.trans_pose_l = trans_pose_l
        self.trans_pose_r = trans_pose_r
        self.min_depth = min_depth
        self.max_depth = max_depth
        self.ir_noise_seed = ir_noise_seed
        self.ir_speckle_noise = ir_speckle_noise
        self.ir_thermal_noise = ir_thermal_noise
        self.rectified = rectified
        self.census_width = census_width
        self.census_height = census_height
        self.max_disp = max_disp
        self.block_width = block_width
        self.block_height = block_height
        self.p1_penalty = p1_penalty
        self.p2_penalty = p2_penalty
        self.uniqueness_ratio = uniqueness_ratio
        self.lr_max_diff = lr_max_diff
        self.median_filter_size = median_filter_size
        self.depth_dilation = depth_dilation

        self._default_speckle_shape = 1333.33
        self._default_gaussian_sigma = 0.25
        self._default_gaussian_mu = 0

        self._engine = None

    def on_add_to_scene(self, scene):
        import cv2

        if self.ir_speckle_noise == 0:
            speckle_shape = 0
        else:
            speckle_shape = self._default_speckle_shape / self.ir_speckle_noise
        speckle_scale = self.ir_speckle_noise / self._default_speckle_shape
        gaussian_mu = self._default_gaussian_mu
        gaussian_sigma = self._default_gaussian_sigma * self.ir_thermal_noise

        # Depth computing engine
        ir_size, rgb_size = self.ir_resolution, self.rgb_resolution
        ir_intrinsic = self.ir_intrinsic.astype(float)
        rgb_intrinsic = self.rgb_intrinsic.astype(float)
        rgb_pose = Pose()  # Dummy pose for deriving extrinsic matrices
        rgb_extrinsic = self._pose2cv2ex(rgb_pose)
        l_extrinsic = self._pose2cv2ex(rgb_pose * self.trans_pose_l)
        r_extrinsic = self._pose2cv2ex(rgb_pose * self.trans_pose_r)
        l2r = r_extrinsic @ np.linalg.inv(l_extrinsic).astype(float)
        l2rgb = rgb_extrinsic @ np.linalg.inv(l_extrinsic).astype(float)
        r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(
            cameraMatrix1=ir_intrinsic,
            distCoeffs1=None,
            cameraMatrix2=ir_intrinsic,
            distCoeffs2=None,
            imageSize=ir_size,
            R=l2r[:3, :3],
            T=l2r[:3, 3:],
            alpha=1.0,
            newImageSize=ir_size,
        )
        f_len = q[2][3]  # focal length of the left camera in meters
        b_len = 1.0 / q[3][2]  # baseline length in meters
        map_l = cv2.initUndistortRectifyMap(
            ir_intrinsic, None, r1, p1, ir_size, cv2.CV_32F
        )
        map_r = cv2.initUndistortRectifyMap(
            ir_intrinsic, None, r2, p2, ir_size, cv2.CV_32F
        )
        map_lx, map_ly = map_l
        map_rx, map_ry = map_r
        a1, a2, a3, b = self._get_registration_mat(
            ir_size, ir_intrinsic, rgb_intrinsic, l2rgb
        )
        rgb_fx = rgb_intrinsic[0][0]
        rgb_fy = rgb_intrinsic[1][1]
        rgb_skew = rgb_intrinsic[0][1]
        rgb_cx = rgb_intrinsic[0][2]
        rgb_cy = rgb_intrinsic[1][2]

        self._engine = DepthSensorEngine(
            ir_size[1],
            ir_size[0],
            rgb_size[1],
            rgb_size[0],
            f_len,
            b_len,
            self.min_depth,
            self.max_depth,
            self.ir_noise_seed,
            speckle_shape,
            speckle_scale,
            gaussian_mu,
            gaussian_sigma,
            self.rectified,
            self.census_width,
            self.census_height,
            self.max_disp,
            self.block_width,
            self.block_height,
            self.p1_penalty,
            self.p2_penalty,
            self.uniqueness_ratio,
            self.lr_max_diff,
            self.median_filter_size,
            map_lx,
            map_ly,
            map_rx,
            map_ry,
            a1,
            a2,
            a3,
            b[0],
            b[1],
            b[2],
            self.depth_dilation,
            rgb_fx,
            rgb_fy,
            rgb_skew,
            rgb_cx,
            rgb_cy,
        )

    def on_remove_from_scene(self, scene):
        self._engine = None

    def compute(
        self,
        left: Union[np.ndarray, CudaArray],
        right: Union[np.ndarray, CudaArray],
        bbox_start: tuple = None,
        bbox_size: tuple = None,
    ) -> None:
        if self._engine is None:
            raise RuntimeError("simsense component is not added to scene")
        
        if bbox_start is not None and bbox_size is not None:
            self._engine.compute(left, right, True, *bbox_start, *bbox_size)
        else:
            self._engine.compute(left, right, False, 0, 0, 0, 0)

    def get_ndarray(self) -> np.ndarray:
        if self._engine is None:
            raise RuntimeError("simsense component is not added to scene")
        return self._engine.get_ndarray()

    def get_cuda(self) -> CudaArray:
        if self._engine is None:
            raise RuntimeError("simsense component is not added to scene")
        return self._engine.get_cuda()

    def get_point_cloud_ndarray(self) -> np.ndarray:
        if self._engine is None:
            raise RuntimeError("simsense component is not added to scene")
        return self._engine.get_point_cloud_ndarray()

    def get_point_cloud_cuda(self) -> CudaArray:
        if self._engine is None:
            raise RuntimeError("simsense component is not added to scene")
        return self._engine.get_point_cloud_cuda()

    def get_rgb_point_cloud_ndarray(self, rgba_cuda: CudaArray) -> np.ndarray:
        if self._engine is None:
            raise RuntimeError("simsense component is not added to scene")
        return self._engine.get_rgb_point_cloud_ndarray(rgba_cuda)

    def get_rgb_point_cloud_cuda(self, rgba_cuda: CudaArray) -> CudaArray:
        if self._engine is None:
            raise RuntimeError("simsense component is not added to scene")
        return self._engine.get_rgb_point_cloud_cuda(rgba_cuda)

    @staticmethod
    def _get_registration_mat(ir_size, ir_intrinsic, rgb_intrinsic, ir2rgb):
        R = ir2rgb[:3, :3]
        t = ir2rgb[:3, 3:]

        w, h = ir_size
        x = np.arange(w)
        y = np.arange(h)
        u, v = np.meshgrid(x, y)
        w = np.ones_like(u)
        pixel_coords = np.stack([u, v, w], axis=-1)  # pixel_coords[y, x] is (x, y, 1)

        A = np.einsum(
            "ij,hwj->hwi", rgb_intrinsic @ R @ np.linalg.inv(ir_intrinsic), pixel_coords
        )
        B = rgb_intrinsic @ t

        return A[..., 0], A[..., 1], A[..., 2], B

    @staticmethod
    def _pose2cv2ex(pose):
        ros2opencv = np.array(
            [
                [0.0, -1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        return ros2opencv @ np.linalg.inv(pose.to_transformation_matrix())
