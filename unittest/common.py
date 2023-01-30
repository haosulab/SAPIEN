import sapien.core as sapien
import numpy as np


def rand_p():
    return np.random.randn(3)


def rand_q():
    q = np.random.randn(4)
    q /= np.linalg.norm(q)
    return q

def rand_size():
    return np.random.rand(3) * 10

def rand_pose():
    return sapien.Pose(rand_p(), rand_q())
