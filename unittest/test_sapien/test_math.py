import pickle
import unittest
from pathlib import Path

import numpy as np
import sapien
from common import rand_p, rand_q, rand_pose, pose_equal, repeat


class TestPose(unittest.TestCase):
    def test_construct(self):
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

    def test_from_matrix(self):
        R = np.array(
            [
                [-0.1301451, -0.5405447, 0.8311881, 0.1],
                [-0.9910400, 0.0963137, -0.0925388, 0.2],
                [-0.0300335, -0.8357841, -0.5482362, 0.3],
                [0, 0, 0, 1],
            ],
            dtype=np.float32,
        )
        pose = sapien.Pose(R)
        self.assertTrue(np.allclose(pose.p, [0.1, 0.2, 0.3]))
        self.assertTrue(
            np.allclose(pose.q, [-0.3232385, 0.5748429, -0.6660884, 0.3484233])
            or np.allclose(pose.q, [0.3232385, -0.5748429, 0.6660884, -0.3484233])
        )

    def test_get_set(self):
        pose = sapien.Pose()
        pose.p = [0.1, 0.2, 0.3]
        self.assertTrue(np.allclose(pose.p, [0.1, 0.2, 0.3]))
        self.assertTrue(np.allclose(pose.get_p(), [0.1, 0.2, 0.3]))
        pose.set_p([0.2, 0.3, 0.4])
        self.assertTrue(np.allclose(pose.p, [0.2, 0.3, 0.4]))

        pose.q = [0, -1, 0, 0]
        self.assertTrue(np.allclose(pose.q, [0, -1, 0, 0]))
        self.assertTrue(np.allclose(pose.get_q(), [0, -1, 0, 0]))
        pose.set_q([0, 0, 1, 0])
        self.assertTrue(np.allclose(pose.q, [0, 0, 1, 0]))

        pose.rpy = [0.3, 0.4, 0.5]
        self.assertTrue(
            np.allclose(pose.q, [0.94628083, 0.09330659, 0.22656631, 0.21098383])
            or np.allclose(pose.q, [-0.94628083, -0.09330659, -0.22656631, -0.21098383])
        )
        rpy = pose.get_rpy()
        pose.rpy = [0, 0, 0]
        pose.set_rpy(rpy)
        self.assertTrue(
            np.allclose(pose.q, [0.94628083, 0.09330659, 0.22656631, 0.21098383])
            or np.allclose(pose.q, [-0.94628083, -0.09330659, -0.22656631, -0.21098383])
        )

    def test_pickle(self):
        p = np.random.uniform(-1, 1, 3).astype(np.float32)
        q = np.array([-0.3232385, 0.5748429, -0.6660884, 0.3484233], dtype=np.float32)
        pose = sapien.Pose(p, q)
        pose1 = pickle.loads(pickle.dumps(pose))
        self.assertTrue(np.allclose(pose.p, pose1.p))
        self.assertTrue(np.allclose(pose.q, pose1.q))

    def test_to_matrix(self):
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

    def test_mult(self):
        pose0 = sapien.Pose(rand_p(), rand_q())
        pose1 = sapien.Pose(rand_p(), rand_q())

        self.assertTrue(
            np.allclose(
                pose0.to_transformation_matrix() @ pose1.to_transformation_matrix(),
                (pose0 * pose1).to_transformation_matrix(),
                rtol=1e-5,
                atol=1e-5,
            )
        )

    def test_inv(self):
        p = rand_p()
        q = rand_q()

        pose = sapien.Pose(p, q)

        i0 = pose * pose.inv()
        i1 = pose.inv() * pose

        self.assertTrue(np.allclose(i0.p, [0, 0, 0], atol=1e-5))
        self.assertTrue(np.allclose(i1.p, [0, 0, 0], atol=1e-5))
        self.assertTrue(np.allclose(np.abs(i0.q[0]), 1))
        self.assertTrue(np.allclose(np.abs(i1.q[0]), 1))


class TestFunctions(unittest.TestCase):
    def test_construct(self):
        q = sapien.math.shortest_rotation([0.1, 0.2, 0.3], [0.8, 0.7, 0.6])
        self.assertTrue(
            np.allclose(q, [0.9684513, -0.1017367, 0.2034734, -0.1017367])
            or np.allclose(-q, [0.9684513, -0.1017367, 0.2034734, -0.1017367])
        )


class TestMatrix(unittest.TestCase):
    def test_matrix(self):
        self.assertTrue(np.allclose(sapien.math.pose_gl_to_ros.p, [0, 0, 0]))
        self.assertTrue(
            np.allclose(sapien.math.pose_gl_to_ros.q, [-0.5, -0.5, 0.5, 0.5])
        )

        self.assertTrue(np.allclose(sapien.math.pose_ros_to_gl.p, [0, 0, 0]))
        self.assertTrue(
            np.allclose(sapien.math.pose_ros_to_gl.q, [-0.5, 0.5, -0.5, -0.5])
        )
        self.assertTrue(
            np.allclose(
                (
                    sapien.math.pose_ros_to_gl * sapien.math.pose_gl_to_ros
                ).to_transformation_matrix(),
                np.eye(4),
            )
        )


class TestMassProperties(unittest.TestCase):
    def test_mesh(self):
        mesh = Path(".") / "assets" / "brass_goblets_1k.glb"
        import trimesh

        mesh = trimesh.load(str(mesh), force="mesh")
        res = sapien.math.compute_mesh_mass_properties(mesh.vertices, mesh.faces)
        props = trimesh.triangles.mass_properties(mesh.vertices[mesh.faces])

        self.assertTrue(np.allclose(res.mass, props.mass, atol=1e-7))
        self.assertTrue(np.allclose(res.cm, props.center_mass, atol=1e-7))
        self.assertTrue(np.allclose(res.cm_inertia, props.inertia, atol=1e-7))

        pose = rand_pose()

        ti = trimesh.inertia.transform_inertia(
            pose.to_transformation_matrix(), res.cm_inertia
        )
        si = res.transform(pose).cm_inertia
        self.assertTrue(np.allclose(ti, si, atol=1e-7))

    @repeat(seed=20)
    def test_inertia(self):
        mass = np.random.rand()
        pose = rand_pose()
        R = pose.to_transformation_matrix()[:3, :3]
        inertia = np.random.rand(3)

        # cosntruct from mass pose inertia_vector
        mp1 = sapien.math.MassProperties(mass, pose, inertia)
        mass1, pose1, inertia1 = mp1.decompose()
        R1 = pose1.to_transformation_matrix()[:3, :3]
        self.assertAlmostEqual(mass, mass1)
        self.assertTrue(
            np.allclose(
                R @ np.diag(inertia) @ R.T,
                R1 @ np.diag(inertia1) @ R1.T,
                atol=1e-5,
                rtol=1e-5,
            )
        )
        self.assertTrue(
            np.allclose(
                R @ np.diag(inertia) @ R.T,
                mp1.cm_inertia,
                atol=1e-5,
                rtol=1e-5,
            )
        )

        # construct from mass cm inertia_matrix
        mp2 = sapien.math.MassProperties(mp1.mass, mp1.cm, mp1.cm_inertia)
        self.assertTrue(np.allclose(mp1.mass, mp2.mass))
        self.assertTrue(np.allclose(mp1.cm, mp2.cm))
        self.assertTrue(np.allclose(mp1.cm_inertia, mp2.cm_inertia))
        self.assertTrue(np.allclose(mp1.origin_inertia, mp2.origin_inertia))

        # get/set origin inertia
        mass = np.random.rand()
        pose = rand_pose()
        R = pose.to_transformation_matrix()[:3, :3]
        inertia = np.random.rand(3)
        mp3 = sapien.math.MassProperties(mass, pose, inertia)
        mp1.origin_inertia = mp3.origin_inertia
        self.assertTrue(np.allclose(mp1.origin_inertia, mp3.origin_inertia))
        mp1.mass = mp3.mass
        mp1.cm = mp3.cm
        self.assertTrue(np.allclose(mp1.cm_inertia, mp3.cm_inertia))

        # get/set cm inertia
        mass = np.random.rand()
        pose = rand_pose()
        R = pose.to_transformation_matrix()[:3, :3]
        inertia = np.random.rand(3)
        mp4 = sapien.math.MassProperties(mass, pose, inertia)
        mp1.cm_inertia = mp4.cm_inertia
        self.assertTrue(np.allclose(mp1.cm_inertia, mp4.cm_inertia))
