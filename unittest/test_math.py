import unittest
import sapien
import numpy as np


class TestPose(unittest.TestCase):
    def test_construction(self):
        # normal construction
        p = np.random.uniform(-1, 1, 3).astype(np.float32)
        q = np.random.uniform(-1, 1, 4).astype(np.float32)
        q /= np.linalg.norm(q)
        pose = sapien.Pose(p, q)
        self.assertTrue(np.allclose(pose.p, p))
        self.assertTrue(np.allclose(pose.q, q))

        # type conversion
        p = np.random.uniform(-1, 1, 3).astype(np.float64)
        q = np.random.uniform(-1, 1, 4)
        pose = sapien.Pose(p, q)
        self.assertTrue(np.allclose(pose.p, p))
        self.assertTrue(np.allclose(pose.q, q))

    def test_matrix(self):
        p = np.random.uniform(-1, 1, 3).astype(np.float32)
        q = np.array([-0.3232385, 0.5748429, -0.6660884, 0.3484233], dtype=np.float32)
        R = np.array(
            [
                [-0.1301451, -0.5405447, 0.8311881],
                [-0.9910400, 0.0963137, -0.0925388],
                [-0.0300335, -0.8357841, -0.5482362],
            ],
            dtype=np.float32,
        )

        pose = sapien.Pose(p, q)
        matrix = pose.to_transformation_matrix()
        pose2 = sapien.Pose(matrix)

        self.assertTrue(np.allclose(matrix[:3, 3], p))
        self.assertTrue(np.allclose(matrix[:3, :3], R))
        self.assertTrue(np.allclose(pose.p, pose2.p))
        self.assertTrue(np.allclose(pose.q, pose2.q) or np.allclose(pose.q, -pose2.q))
