import sapien.core as sapien
import numpy as np


def repeat(n=10, seed=0):
    def wrapper(func):
        def f(*args, **kwargs):
            np.random.seed(seed)
            for _ in range(n):
                func(*args, **kwargs)

        return f

    return wrapper


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


def pose_equal(pose0, pose1, rtol=1.0e-5, atol=1.0e-8):
    return np.allclose(pose0.p, pose1.p, rtol=rtol, atol=atol) and (
        np.allclose(pose0.q, pose1.q, rtol=rtol, atol=atol)
        or np.allclose(pose0.q, -pose1.q, rtol=rtol, atol=atol)
    )
