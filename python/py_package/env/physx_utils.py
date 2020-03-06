import numpy as np
import transforms3d
import sapien.core as sapien


def transform2mat(trans: sapien.Pose) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, :3] = transforms3d.quaternions.quat2mat(trans.q)
    mat[:3, 3] = trans.p
    return mat


def mat2transform(mat: np.ndarray) -> sapien.Pose:
    assert mat.shape == (4, 4)
    pose = sapien.Pose(mat[: 3, 3], transforms3d.quaternions.mat2quat(mat[:3, :3]))
    return pose


def rot2transform(rot: np.ndarray) -> sapien.Pose:
    assert rot.shape == (3, 3)
    pose = sapien.Pose(np.zeros(3), transforms3d.quaternions.mat2quat(rot))
    return pose


def rpy2transform(rpy: np.ndarray) -> sapien.Pose:
    assert rpy.shape == (3,)
    pose = sapien.Pose(np.zeros(3), transforms3d.euler.euler2quat(rpy))
    return pose
