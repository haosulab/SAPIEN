import unittest
import numpy as np
from common import pose_equal, rand_pose
from pathlib import Path

import sapien


class TestShape(unittest.TestCase):
    def _test_common(
        self,
        shape: sapien.physx.PhysxCollisionShapePlane,
        mat: sapien.physx.PhysxMaterial,
    ):
        self.assertTrue(shape.physical_material, mat)
        self.assertTrue(shape.get_physical_material(), mat)
        self.assertAlmostEqual(shape.contact_offset, 0.015)
        self.assertAlmostEqual(shape.rest_offset, 0.001)

        mat = sapien.physx.PhysxMaterial(0.3, 0.2, 0.1)
        shape.set_physical_material(mat)
        self.assertTrue(shape.physical_material, mat)
        self.assertTrue(shape.get_physical_material(), mat)

        p = rand_pose()
        shape.set_local_pose(p)
        self.assertTrue(pose_equal(shape.get_local_pose(), p))
        p = rand_pose()
        shape.local_pose = p
        self.assertTrue(pose_equal(shape.local_pose, p))

        shape.set_collision_groups([1, 2, 3, 4])
        self.assertEqual(shape.get_collision_groups(), [1, 2, 3, 4])
        self.assertEqual(shape.collision_groups, [1, 2, 3, 4])

        shape.set_contact_offset(0.1)
        self.assertAlmostEqual(shape.get_contact_offset(), 0.1)
        shape.contact_offset = 0.2
        self.assertAlmostEqual(shape.contact_offset, 0.2)

        shape.set_rest_offset(0.01)
        self.assertAlmostEqual(shape.get_rest_offset(), 0.01)
        shape.rest_offset = 0.02
        self.assertAlmostEqual(shape.rest_offset, 0.02)

        shape.set_density(500)
        self.assertAlmostEqual(shape.get_density(), 500)
        shape.density = 800
        self.assertAlmostEqual(shape.density, 800)

        shape.set_patch_radius(0.03)
        self.assertAlmostEqual(shape.get_patch_radius(), 0.03)
        shape.patch_radius = 0.04
        self.assertAlmostEqual(shape.patch_radius, 0.04)

        shape.set_min_patch_radius(0.05)
        self.assertAlmostEqual(shape.get_min_patch_radius(), 0.05)
        shape.min_patch_radius = 0.06
        self.assertAlmostEqual(shape.min_patch_radius, 0.06)

    def test_plane(self):
        sapien.physx.set_shape_config(contact_offset=0.015, rest_offset=0.001)
        mat = sapien.physx.PhysxMaterial(0.2, 0.1, 0.05)
        shape = sapien.physx.PhysxCollisionShapePlane(mat)
        self._test_common(shape, mat)

    def test_sphere(self):
        sapien.physx.set_shape_config(contact_offset=0.015, rest_offset=0.001)
        mat = sapien.physx.PhysxMaterial(0.2, 0.1, 0.05)
        shape = sapien.physx.PhysxCollisionShapeSphere(0.1, mat)
        self._test_common(shape, mat)

        self.assertTrue(np.allclose(shape.radius, 0.1))

    def test_box(self):
        sapien.physx.set_shape_config(contact_offset=0.015, rest_offset=0.001)
        mat = sapien.physx.PhysxMaterial(0.2, 0.1, 0.05)
        shape = sapien.physx.PhysxCollisionShapeBox([1, 2, 3], mat)
        self._test_common(shape, mat)

        self.assertTrue(np.allclose(shape.half_size, [1, 2, 3]))

    def test_capsule(self):
        sapien.physx.set_shape_config(contact_offset=0.015, rest_offset=0.001)
        mat = sapien.physx.PhysxMaterial(0.2, 0.1, 0.05)
        shape = sapien.physx.PhysxCollisionShapeCapsule(0.2, 0.3, mat)
        self._test_common(shape, mat)

        self.assertTrue(np.allclose(shape.radius, 0.2))
        self.assertTrue(np.allclose(shape.half_length, 0.3))

    def test_cylinder(self):
        sapien.physx.set_shape_config(contact_offset=0.015, rest_offset=0.001)
        mat = sapien.physx.PhysxMaterial(0.2, 0.1, 0.05)
        shape = sapien.physx.PhysxCollisionShapeCylinder(0.2, 0.3, mat)
        self._test_common(shape, mat)

        self.assertTrue(np.allclose(shape.radius, 0.2))
        self.assertTrue(np.allclose(shape.half_length, 0.3))

    def test_convex(self):
        sapien.physx.set_shape_config(contact_offset=0.015, rest_offset=0.001)
        mat = sapien.physx.PhysxMaterial(0.2, 0.1, 0.05)
        shape = sapien.physx.PhysxCollisionShapeConvexMesh(
            str(Path(".") / "assets" / "cone.stl"), [0.1, 0.2, 0.3], mat
        )
        self._test_common(shape, mat)
        self.assertTrue(np.allclose(shape.scale, [0.1, 0.2, 0.3]))

    def test_nonconvex(self):
        sapien.physx.set_shape_config(contact_offset=0.015, rest_offset=0.001)
        mat = sapien.physx.PhysxMaterial(0.2, 0.1, 0.05)
        shape = sapien.physx.PhysxCollisionShapeTriangleMesh(
            str(Path(".") / "assets" / "torus.stl"), [0.2, 0.3, 0.4], mat
        )
        self._test_common(shape, mat)
        self.assertTrue(np.allclose(shape.scale, [0.2, 0.3, 0.4]))
