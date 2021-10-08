# by Xiaoshuai Jet Zhang <i@jetd.me>, Jan 2021
# v2021.10.08
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

from typing import Optional, Tuple

import numpy as np
try:
    import open3d as o3d
except ImportError:
    print('Please install open3d with `pip3 install open3d`')
    raise
try:
    import scipy.signal
except ImportError:
    print('Please install scipy with `pip3 install scipy`')
    raise
try:
    import cv2
    from cv2 import ximgproc
except ImportError:
    print('opencv-contrib not installed, '
          'some features will be disabled.')
    print('Please install with `pip3 install opencv-contrib-python`')
    ximgproc = None


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


def get_census(img, wsize=7):
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
        ndisp: int = 96, min_disp: int = 0, lr_consistency: bool = True,
        use_census: bool = True, census_wsize: int = 7
) -> np.ndarray:
    """
    Calculate disparity given a rectified image pair.

    :param imgl: Left image
    :param imgr: Right image
    :param method: SGBM or BM
    :param ndisp: max disparity
    :param min_disp: min disparity
    :param lr_consistency: Use Left-Right Consistency (LRC) check
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
            disp12MaxDiff=1,
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

    if lr_consistency:
        if ximgproc is not None:
            matcherr = ximgproc.createRightMatcher(matcherl)
            dispr = matcherr.compute(imgr, imgl)

            wls_filter = ximgproc.createDisparityWLSFilter(matcherl)
            wls_filter.setLRCthresh(16)
            wls_filter.setLambda(0.02)
            wls_filter.setSigmaColor(1.0)
            displ = wls_filter.filter(
                disparity_map_left=displ,
                disparity_map_right=dispr,
                left_view=imgl,
                right_view=imgr
            )
            # confidence = wls_filter.getConfidenceMap()
        else:
            print('opencv-contrib not installed, post-processing disabled.')

    displ = unpad_lr(displ, ndisp).astype(np.float32) / 16.0

    return displ


def calc_rectified_stereo_pair(
        imgl: np.ndarray, imgr: np.ndarray, kl: np.ndarray, kr: np.ndarray, rt: np.ndarray,
        distortl: Optional[np.ndarray] = None, distortr: Optional[np.ndarray] = None
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Rectify an image pair with given camera parameters.

    :param imgl: Left image
    :param imgr: Right image
    :param kl: Left intrinsic matrix
    :param kr: Right intrinsic matrix
    :param rt: Extrinsic matrix (left to right)
    :param distortr: Left distortion coefficients
    :param distortl: Right distortion coefficients
    :return imgl_rect: Rectified left image
    :return imgr_rect: Rectified right image
    :return q: Perspective transformation matrix (for cv2.reprojectImageTo3D)
    """
    assert imgl.shape == imgr.shape
    h, w = imgl.shape

    rt = np.linalg.inv(rt)

    r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(
        R=rt[:3, :3], T=rt[:3, 3:],
        cameraMatrix1=kl, cameraMatrix2=kr,
        alpha=1.0, imageSize=(w, h), newImageSize=(w, h),
        distCoeffs1=distortl, distCoeffs2=distortr
    )

    map1 = cv2.initUndistortRectifyMap(kl, distortr, r1, p1, (w, h), cv2.CV_32F)
    map2 = cv2.initUndistortRectifyMap(kr, distortl, r2, p2, (w, h), cv2.CV_32F)
    imgl_rect = cv2.remap(imgl, *map1, cv2.INTER_LINEAR)
    imgr_rect = cv2.remap(imgr, *map2, cv2.INTER_LINEAR)

    return imgl_rect, imgr_rect, q


def calc_depth_and_pointcloud(
        disparity: np.ndarray, mask: np.ndarray, q: np.ndarray,
        no_pointcloud: bool = False
) -> Tuple[np.ndarray, o3d.geometry.PointCloud]:
    """
    Calculate depth and pointcloud.

    :param disparity: Disparity
    :param mask: Valid mask
    :param q: Perspective transformation matrix
    :param no_pointcloud:
    :return depth: Depth
    :return pointcloud: Pointcloud
    """
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
        rt_l: np.ndarray, rt_r: np.ndarray, rt_main: np.ndarray,
        k_l: np.ndarray, k_r: np.ndarray, k_main: np.ndarray,
        method: str = 'SGBM',
        ndisp: int = 96,
        use_noise: bool = True,
        use_census: bool = True,
        lr_consistency: bool = False,
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
    :param rt_l: left extrinsic matrix
    :param rt_r: right extrinsic matrix
    :param rt_main: rgb extrinsic matrix
    :param k_l: left intrinsic matrix
    :param k_r: right intrinsic matrix
    :param k_main: rgb intrinsic matrix
    :param method: method for depth calculation (SGBM or BM)
    :param use_noise: whether to simulate ir noise before processing
    :param lr_consistency: whether to use left-right consistency check
    :return depth: calculated depth
    """
    assert ir_l.shape == ir_r.shape

    # assert np.allclose(k_l, k_r)
    # w, h = k_main[:2, 2]
    # w, h = int(w * 2), int(h * 2)
    w, h = main_cam_size
    rt_lr = rt_l @ np.linalg.inv(rt_r)
    rt_mainl = rt_main @ np.linalg.inv(rt_l)

    if not np.allclose(rt_lr[:3, :3], np.eye(3), atol=1e-6):
        raise RuntimeError("extrinsics contain rotation")
    if not (np.sum(rt_lr[1:3, 3] ** 2) < 2e-4):
        raise RuntimeError(f"extrinsics contain translation {rt_lr[:3, 3]}")

    if use_noise:
        ir_l, ir_r = sim_ir_noise(ir_l, **kwargs), sim_ir_noise(ir_r, **kwargs)

    _, _, q = calc_rectified_stereo_pair(
        ir_l, ir_r,
        k_l.astype(np.float), k_r.astype(np.float), rt_lr.astype(np.float)
    )

    disp = calc_disparity(
        ir_l, ir_r, method,
        lr_consistency=lr_consistency, ndisp=ndisp,
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
            None, rt_mainl.astype(np.float), depth, (w, h), depthDilation=True)
        depth[np.isnan(depth)] = 0
        depth[np.isinf(depth)] = 0
        depth[depth < 0] = 0
        if register_blur_ksize > 0:
            depth = cv2.medianBlur(depth, register_blur_ksize)

    return depth
