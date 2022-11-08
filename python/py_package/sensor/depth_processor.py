# by Xiaoshuai Jet Zhang <i@jetd.me>, Jan 2021
# Changed by Ang Li, Sep 2022
# v2022.09.27
#
# Depth simulation toolkit.
#
# Please run:
#  1. `pip3 install --upgrade pip`
#  2. `pip3 install opencv-contrib-python scipy open3d`
# before using this package.
#
# TODO:
#  1. IR intensity instead of pixel values in real pipeline.
#

from ..core.pysapien.simsense import DepthSensorEngine

from typing import Optional, Tuple

import numpy as np
try:
    import scipy.signal
except ImportError:
    print('Please install scipy with `pip3 install scipy`')
    raise
try:
    import cv2
except ImportError:
    print('opencv-contrib not installed, '
          'some features will be disabled.')
    print('Please install with `pip3 install opencv-contrib-python`')


def pad_lr(img: np.ndarray, ndisp: int) -> np.ndarray:
    padding = np.zeros((img.shape[0], ndisp), dtype=np.uint8)
    return np.concatenate([padding, img, padding], axis=1)


def unpad_lr(img: np.ndarray, ndisp: int) -> np.ndarray:
    return img[:, ndisp: -ndisp]


def sim_ir_noise(
        img: np.ndarray,
        scale: float = 0.0, blur_ksize: int = 0, blur_ksigma: float = 0.03,
        speckle_shape: float = 398.12, speckle_scale: float = 2.54e-3,
        gaussian_mu: float = -0.231, gaussian_sigma: float = 0.83,
        seed: int = 0
) -> np.ndarray:
    """
    TODO: IR density model

    Simulate IR camera noise.

    Noise model from Landau et al.
    Simulating Kinect Infrared and Depth Images

    :param img: Input IR image
    :param scale: Scale for downsampling & applying gaussian blur
    :param blur_ksize: Kernel size for gaussian blur
    :param blur_ksigma: Kernel sigma for gaussian blur
    :param speckle_shape: Shape parameter for speckle noise (Gamma distribution)
    :param speckle_scale: Scale parameter for speckle noise (Gamma distribution)
    :param gaussian_mu: mu for additive gaussian noise
    :param gaussian_sigma: sigma for additive gaussian noise
    :param seed: random seed used for numpy
    :return: Processed IR image
    """
    h, w = img.shape

    if scale > 0:
        # downsampling (to emulate soft intensity)
        inter = cv2.INTER_BICUBIC if scale > 1 else cv2.INTER_LANCZOS4
        img = cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=inter)
        if blur_ksize > 0:
            img = cv2.GaussianBlur(img, (blur_ksize, blur_ksize), blur_ksigma)
        img = cv2.resize(img, (w, h), interpolation=cv2.INTER_CUBIC)

    rng = np.random.default_rng(seed)
    img = img.astype(np.float)

    # speckle noise
    img = img * rng.gamma(shape=speckle_shape, scale=speckle_scale, size=img.shape)

    # gaussian noise
    img = img + gaussian_mu + gaussian_sigma * rng.standard_normal(img.shape)

    # renormalize
    img[img < 0] = 0
    img[img > 255] = 255
    img = img.astype(np.uint8)

    return img


def depth_post_processing(depth: np.ndarray, ksize: int = 5) -> np.ndarray:
    # depth = scipy.signal.medfilt2d(depth, kernel_size=ksize)
    return depth


def get_census(img: np.ndarray, wsize:int = 7) -> np.ndarray:
    h, w = img.shape
    assert wsize % 2 == 1

    whalf = wsize // 2
    center = img[whalf: h - whalf, whalf: w - whalf]

    census = np.zeros((h - 2 * whalf, w - 2 * whalf), dtype=np.uint8)
    offsets = [(u, v) for v in range(wsize) \
               for u in range(wsize) if not u == v == whalf]

    for u, v in offsets:
        census = (census << 1) \
                 | (img[v: v + h - 2 * whalf, u: u + w - 2 * whalf] >= center)

    ret = np.zeros((h, w), dtype=np.uint8)
    ret[whalf: -whalf, whalf: -whalf] = census

    return ret


def calc_disparity(
        imgl: np.ndarray, imgr: np.ndarray, method: str, *,
        ndisp: int = 96, min_disp: int = 0,
        use_census: bool = True, census_wsize: int = 7
) -> np.ndarray:
    """
    Calculate disparity given a rectified image pair.

    :param imgl: Left image
    :param imgr: Right image
    :param method: SGBM or BM
    :param ndisp: max disparity
    :param min_disp: min disparity
    :return: disparity
    """
    if use_census:
        imgl = get_census(imgl, census_wsize)
        imgr = get_census(imgr, census_wsize)

    imgl = pad_lr(imgl, ndisp)
    imgr = pad_lr(imgr, ndisp)

    if method == 'SGBM':
        window_size = 7
        matcherl = cv2.StereoSGBM_create(
            minDisparity=min_disp,
            numDisparities=ndisp,
            blockSize=window_size,
            P1=8 * 1 * window_size ** 2,
            P2=32 * 1 * window_size ** 2,
            disp12MaxDiff=1, # Left-Right Consistency Check
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=1,
            mode=cv2.STEREO_SGBM_MODE_HH
        )
    elif method == 'BM':
        matcherl = cv2.StereoBM_create(
            numDisparities=ndisp,
            blockSize=7
        )
    else:
        raise NotImplementedError(f'Not implemented: {method}')

    displ = matcherl.compute(imgl, imgr)

    displ = unpad_lr(displ, ndisp).astype(np.float32) / 16.0

    return displ


def init_rectify_stereo(
        w: int, h: int, kl: np.ndarray, kr: np.ndarray, rt: np.ndarray,
        distortl: Optional[np.ndarray] = None, distortr: Optional[np.ndarray] = None
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Initiate parameters needed for rectification with given camera parameters.

    :param w: width of the image
    :param h: height of the image
    :param kl: Left intrinsic matrix
    :param kr: Right intrinsic matrix
    :param rt: Extrinsic matrix (left to right)
    :param distortl: Left distortion coefficients
    :param distortr: Right distortion coefficients
    :return map1: Map for left camera
    :return map2: Map for right camera
    :return q: Perspective transformation matrix (for cv2.reprojectImageTo3D)
    """
    r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(
        R=rt[:3, :3], T=rt[:3, 3:],
        cameraMatrix1=kl, cameraMatrix2=kr,
        alpha=1.0, imageSize=(w, h), newImageSize=(w, h),
        distCoeffs1=distortl, distCoeffs2=distortr
    )

    map1 = cv2.initUndistortRectifyMap(kl, distortl, r1, p1, (w, h), cv2.CV_32F)
    map2 = cv2.initUndistortRectifyMap(kr, distortr, r2, p2, (w, h), cv2.CV_32F)

    return map1, map2, q


def calc_rectified_stereo_pair(
        imgl: np.ndarray, imgr: np.ndarray, map1: np.ndarray, map2:np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Rectify an image pair with given camera parameters.

    :param imgl: Left image
    :param imgr: Right image
    :param map1: Map for left camera
    :param map2: Map for right camera
    :return imgl_rect: Rectified left image
    :return imgr_rect: Rectified right image
    """
    assert imgl.shape == imgr.shape

    imgl_rect = cv2.remap(imgl, *map1, cv2.INTER_LINEAR)
    imgr_rect = cv2.remap(imgr, *map2, cv2.INTER_LINEAR)

    return imgl_rect, imgr_rect


def calc_depth_and_pointcloud(
        disparity: np.ndarray, mask: np.ndarray, q: np.ndarray,
        no_pointcloud: bool = False
) -> Tuple[np.ndarray, object]:  # object is o3d.geometry.PointCloud
    """
    Calculate depth and pointcloud.

    :param disparity: Disparity
    :param mask: Valid mask
    :param q: Perspective transformation matrix
    :param no_pointcloud:
    :return depth: Depth
    :return pointcloud: Pointcloud
    """
    try:
        import open3d as o3d
    except ImportError:
        print('Please install open3d with `pip3 install open3d`')
        raise

    _3d_image = cv2.reprojectImageTo3D(disparity, q)
    depth = _3d_image[..., 2]
    depth[~mask] = 0
    depth[np.isinf(depth)] = 0
    depth[np.isnan(depth)] = 0

    depth = depth_post_processing(depth) * mask
    _3d_image[..., 2] = depth

    if no_pointcloud:
        pointcloud = None
    else:
        points = _3d_image.reshape(-1, 3)
        valid_flag = mask.reshape(-1)
        valid_points = []
        for i, point in enumerate(points):
            if valid_flag[i]:
                valid_points.append(point)
        valid_points = o3d.utility.Vector3dVector(np.array(valid_points))
        pointcloud = o3d.geometry.PointCloud(points=valid_points)

    return depth, pointcloud


def calc_main_depth_from_left_right_ir(
        ir_l: np.ndarray, ir_r: np.ndarray,
        l2r:np.ndarray, l2rgb: np.ndarray,
        k_l: np.ndarray, k_r: np.ndarray, k_main: np.ndarray,
        map1: np.ndarray, map2:np.ndarray, q: np.ndarray,
        method: str = 'SGBM',
        ndisp: int = 96,
        use_noise: bool = True,
        use_census: bool = True,
        register_depth: bool = True,
        register_blur_ksize: int = 5,
        main_cam_size=(1920, 1080),
        census_wsize=7,
        **kwargs
) -> np.ndarray:
    """
    Calculate depth for rgb camera from left right ir images.

    :param ir_l: left ir image
    :param ir_r: right ir image
    :param l2r: Change-of-coordinate matrix from left camera's frame to right camera's frame (in OpenCV coordinate system)
    :param l2rgb: Change-of-coordinate matrix from left camera's frame to RGB camera's frame (in OpenCV coordinate system)
    :param k_l: left intrinsic matrix
    :param k_r: right intrinsic matrix
    :param k_main: rgb intrinsic matrix
    :param map1: Left map for rectification
    :param map2: Right map for rectification
    :param q: Perspective transformation matrix (for cv2.reprojectImageTo3D)
    :param method: method for depth calculation (SGBM or BM)
    :param use_noise: whether to simulate ir noise before processing
    :return depth: calculated depth
    """
    assert ir_l.shape == ir_r.shape

    # assert np.allclose(k_l, k_r)
    # w, h = k_main[:2, 2]
    # w, h = int(w * 2), int(h * 2)
    w, h = main_cam_size

    if not np.allclose(l2r[:3, :3], np.eye(3), atol=1e-6):
        raise RuntimeError("extrinsics contain rotation")
    if not (np.sum(l2r[1:3, 3] ** 2) < 2e-4):
        raise RuntimeError(f"extrinsics contain translation {l2r[:3, 3]}")

    if use_noise:
        ir_l, ir_r = sim_ir_noise(ir_l, **kwargs), sim_ir_noise(ir_r, **kwargs)

    # ir_l, ir_r = calc_rectified_stereo_pair(
    #     ir_l, ir_r, map1, map2
    # )

    disp = calc_disparity(
        ir_l, ir_r, method, ndisp=ndisp,
        use_census=use_census, census_wsize=census_wsize
    )

    valid_mask = disp >= 1
    depth, _ = calc_depth_and_pointcloud(disp, valid_mask, q, no_pointcloud=True)

    depth[np.isnan(depth)] = 0
    depth[np.isinf(depth)] = 0
    depth[depth < 0] = 0

    if register_depth:
        depth = cv2.rgbd.registerDepth(
            k_l.astype(np.float), k_main.astype(np.float),
            None, l2rgb.astype(np.float), depth, (w, h), depthDilation=True)
        depth[np.isnan(depth)] = 0
        depth[np.isinf(depth)] = 0
        depth[depth < 0] = 0
        if register_blur_ksize > 0:
            depth = cv2.medianBlur(depth, register_blur_ksize)

    return depth

# GPU Depth Sensor
class DepthSensorCUDA:
    def __init__(self, lr_size, k_l, k_r, l2r, rgb_size=None, k_rgb=None, l2rgb=None, min_depth=0.0, max_depth=10.0, rectified=False,
                    census_width=9, census_height=7, max_disp=128, block_width=1, block_height=1, p1_penalty=7, p2_penalty=86,
                    uniqueness_ratio=0, lr_max_diff=1, median_filter_size=3, depth_dilation=False):
        """
        Initiate the DepthSensor class. The camera frame follows the OpenCV coordinate system, which is x right, y down, z forward.
        In this sense, the origin of the right camera's frame should have positive x value when viewed in left camera's frame. Left,
        right and RGB image are assumed to be undistorted. By default, the final depth map will be presented in left camera's frame
        with lr_size. Specifying rgb_size, k_rgb and l2rgb will turn on the registration mode, which will tranform the final depth
        map from left camera's frame to specified RGB camera's frame with rgb_size.

        :param lr_size: (width, height) of the left and right image.
        :param k_l: Intrinsic matrix of the left camera (in OpenCV coordinate system).
        :param k_r: Intrinsic matrix of the right camera (in OpenCV coordinate system).
        :param l2r: Change-of-coordinate matrix from left camera's frame to right camera's frame (in OpenCV coordinate system).
        :param rgb_size: (width, height) of the RGB image.
        :param k_rgb: Intrinsic matrix of the RGB camera (in OpenCV coordinate system).
        :param l2rgb: Change-of-coordinate matrix from left camera's frame to RGB camera's frame (in OpenCV coordinate system).
        :param min_depth: Minimum valid depth in meters.
        :param max_depth: Maximum valid depth (non-inclusive) in meters.
        :param rectified: Whether the input has already been rectified. Set to true if no rectification is needed.
        :param census_width: Width of the center-symmetric census transform window. This must be an odd number.
        :param census_height: Height of the center-symmetric census transform window. This must be an odd number.
        :param max_disp: Maximum disparity search space (non-inclusive) for stereo matching.
        :param block_width: Width of the matched block. This must be an odd number.
        :param block_height: Height of the matched block. This must be an odd number.
        :param p1_penalty: P1 penalty for semi-global matching algorithm. It is the penalty on the disparity change by plus or minus
                           1 between neighboring pixels.
        :param p2_penalty: P2 penalty for semi-global matching algorithm. It is the penalty on the disparity change by more than 1
                           between neighboring pixels.
        :param uniqueness_ratio: Margin in percentage by which the minimum computed cost should win the second best (not considering
                                 best match's adjacent pixels) cost to consider the found match valid.
        :param lr_max_diff: Maximum allowed difference in the left-right consistency check. Set it to 255 will disable the check.
        :param median_filter_size: Size of the median filter. Choices are 1, 3, 5, 7. When set to 1 the median filter is turned off.
        :param depth_dilation: Whether to dilate the final depth map to avoid holes. This is only effective when registration mode is on.
                               Depth dilation is recommended when rgb_size is greater than lr_size.
        """
        img_w, img_h = lr_size
        registration = False
        if rgb_size != None or k_rgb != None or l2rgb != None:
            registration = True

        # Instance check
        if not isinstance(img_h, int) or not isinstance(img_w, int) or img_h < 32 or img_w < 32:
            raise TypeError("Image height and width must be integer no less than 32")
        
        if registration == True and (rgb_size is None or k_rgb is None or l2rgb is None):
            raise TypeError("Registration mode is on but missing one or two RGB camera's parameters")

        if not isinstance(census_width, int) or not isinstance(census_height, int) or census_width <= 0 or census_height <= 0 or \
                census_width % 2 == 0 or census_height % 2 == 0 or census_width*census_height > 65:
            raise TypeError("census_width and census_height must be positive odd integers and their product should be no larger than 65")
        
        if not isinstance(max_disp, int) or max_disp < 32 or max_disp > 1024:
            raise TypeError("max_disp must be integer within range [32, 1024]")
        
        if not isinstance(block_width, int) or not isinstance(block_height, int) or block_width <= 0 or block_height <= 0 or \
                block_width % 2 == 0 or block_height % 2 == 0 or block_width*block_height > 256:
            raise TypeError("block_width and block_height must be positive odd integers and their product should be no larger than 256")
 
        if not isinstance(p1_penalty, int) or not isinstance(p2_penalty, int) or p1_penalty <= 0 or p2_penalty <= 0 or \
                p1_penalty >= p2_penalty or p2_penalty >= 224:
            raise TypeError("p1 must be positive integer less than p2 and p2 be positive integer less than 224")

        if not isinstance(uniqueness_ratio, int) or uniqueness_ratio < 0 or uniqueness_ratio > 255:
            raise TypeError("uniqueness_ratio must be positive integer no larger than 255")

        if not isinstance(lr_max_diff, int) or lr_max_diff < -1 or lr_max_diff > 255:
            raise TypeError("lr_max_diff must be integer within the range [0, 255]")

        if median_filter_size != 1 and median_filter_size != 3 and median_filter_size != 5 and median_filter_size != 7:
            raise TypeError("Median filter size choices are 1, 3, 5, 7")

        # Get rectification map
        r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(
            cameraMatrix1=k_l, distCoeffs1=None, cameraMatrix2=k_r, distCoeffs2=None,
            imageSize=lr_size, R=l2r[:3, :3], T=l2r[:3, 3:], alpha=1.0, newImageSize=lr_size
        )
        f_len = q[2][3] # focal length of the left camera in meters
        b_len = 1.0 / q[3][2] # baseline length in meters
        map_l = cv2.initUndistortRectifyMap(k_l, None, r1, p1, lr_size, cv2.CV_32F)
        map_r = cv2.initUndistortRectifyMap(k_r, None, r2, p2, lr_size, cv2.CV_32F)
        map_lx, map_ly = map_l
        map_rx, map_ry = map_r

        if registration:
            # Get registration matrix
            a1, a2, a3, b = self._get_registration_mat(lr_size, k_l, k_rgb, l2rgb)
            self.engine = DepthSensorEngine(img_h, img_w, rgb_size[1], rgb_size[0], f_len, b_len, min_depth, max_depth, rectified,
                                            census_width, census_height, max_disp, block_width, block_height,
                                            p1_penalty, p2_penalty, uniqueness_ratio, lr_max_diff, median_filter_size,
                                            map_lx, map_ly, map_rx, map_ry, a1, a2, a3, b[0], b[1], b[2], depth_dilation)
        else:
            self.engine = DepthSensorEngine(img_h, img_w, f_len, b_len, min_depth, max_depth, rectified,
                                            census_width, census_height, max_disp, block_width, block_height,
                                            p1_penalty, p2_penalty, uniqueness_ratio, lr_max_diff, median_filter_size,
                                            map_lx, map_ly, map_rx, map_ry)

    def compute(self, img_l, img_r):
        """
        Take two images captured by a pair of nearby parallel cameras, and output the computed depth map in meters.

        :param img_l: Grayscale/infrared image (uint8) captured by left camera.
        :param img_r: Grayscale/infrared image (uint8) captured by right camera.
        :return: Computed depth map (in meters) from left camera's view or rgb camera's view.
        """
        result = self.engine.compute(img_l, img_r)

        return result
    
    def set_penalties(self, p1_penalty, p2_penalty):
        if not isinstance(p1_penalty, int) or not isinstance(p2_penalty, int) or p1_penalty <= 0 or p2_penalty <= 0 or \
                p1_penalty >= p2_penalty or p2_penalty >= 224:
            raise TypeError("p1 must be positive integer less than p2 and p2 be positive integer less than 224")
        self.engine._set_penalties(p1_penalty, p2_penalty)
    
    def set_census_window_size(self, census_width, census_height):
        if not isinstance(census_width, int) or not isinstance(census_height, int) or census_width <= 0 or census_height <= 0 or \
                census_width % 2 == 0 or census_height % 2 == 0 or census_width*census_height > 65:
            raise TypeError("census_width and census_height must be positive odd integers and their product should be no larger than 65")
        self.engine._set_census_window_size(census_width, census_height)
    
    def set_matching_block_size(self, block_width, block_height):
        if not isinstance(block_width, int) or not isinstance(block_height, int) or block_width <= 0 or block_height <= 0 or \
                block_width % 2 == 0 or block_height % 2 == 0 or block_width*block_height > 256:
            raise TypeError("block_width and block_height must be positive odd integers and their product should be no larger than 256")
        self.engine._set_matching_block_size(block_width, block_height)
    
    def set_uniqueness_ratio(self, uniqueness_ratio):
        if not isinstance(uniqueness_ratio, int) or uniqueness_ratio < 0 or uniqueness_ratio > 255:
            raise TypeError("uniqueness_ratio must be positive integer no larger than 255")
        self.engine._set_uniqueness_ratio(uniqueness_ratio)

    def set_lr_max_diff(self, lr_max_diff):
        if not isinstance(lr_max_diff, int) or lr_max_diff < -1 or lr_max_diff > 255:
            raise TypeError("lr_max_diff must be integer within the range [0, 255]")
        self.engine._set_lr_max_diff(lr_max_diff)

    def _get_registration_mat(self, ir_size, k_ir, k_rgb, ir2rgb):
        R = ir2rgb[:3, :3]
        t = ir2rgb[:3, 3:]
        
        w, h = ir_size
        x = np.arange(w)
        y = np.arange(h)
        u, v = np.meshgrid(x, y)
        w = np.ones_like(u)
        pixel_coords = np.stack([u, v, w], axis=-1) # pixel_coords[y, x] is (x, y, 1)

        A = np.einsum("ij,hwj->hwi", k_rgb @ R @ np.linalg.inv(k_ir), pixel_coords)
        B = k_rgb @ t

        return A[..., 0], A[..., 1], A[..., 2], B
