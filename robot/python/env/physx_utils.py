import sapyen
import numpy as np
import transforms3d


def transform2mat(trans: sapyen.Pose):
    mat = np.eye(4)
    mat[:3, :3] = transforms3d.quaternions.quat2mat(trans.q)
    mat[:3, 3] = trans.p
    return mat


def mat2transform(mat: np.ndarray):
    pose = sapyen.Pose(mat[: 3, 3], transforms3d.quaternions.mat2quat(mat[:3, :3]))
    return pose
