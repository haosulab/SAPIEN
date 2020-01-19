import numpy as np
import transforms3d

try:
    import pysapien
except ModuleNotFoundError:
    import sapien.core as pysapien


def transform2mat(trans: pysapien.Pose) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, :3] = transforms3d.quaternions.quat2mat(trans.q)
    mat[:3, 3] = trans.p
    return mat


def mat2transform(mat: np.ndarray) -> pysapien.Pose:
    assert mat.shape == (4, 4)
    pose = pysapien.Pose(mat[: 3, 3], transforms3d.quaternions.mat2quat(mat[:3, :3]))
    return pose


def rot2transform(rot: np.ndarray) -> pysapien.Pose:
    assert rot.shape == (3, 3)
    pose = pysapien.Pose(np.zeros(3), transforms3d.quaternions.mat2quat(rot))
    return pose


def rpy2transform(rpy: np.ndarray) -> pysapien.Pose:
    assert rpy.shape == (3,)
    pose = pysapien.Pose(np.zeros(3), transforms3d.euler.euler2quat(rpy))
    return pose
