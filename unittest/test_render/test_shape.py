import unittest
from pathlib import Path
import numpy as np
from common import rand_pose, pose_equal

import sapien


class TestShape(unittest.TestCase):
    def _test_common(
        self, shape: sapien.render.RenderShape, mat: sapien.render.RenderMaterial
    ):
        shape.front_face = "clockwise"
        self.assertEqual(shape.front_face, "clockwise")
        shape.set_front_face("counterclockwise")
        self.assertEqual(shape.get_front_face(), "counterclockwise")

        p = rand_pose()
        shape.local_pose = p
        self.assertTrue(pose_equal(shape.local_pose, p))
        p = rand_pose()
        shape.set_local_pose(p)
        self.assertTrue(pose_equal(shape.get_local_pose(), p))

        shape.name = "s0"
        self.assertEqual(shape.name, "s0")
        shape.set_name("s1")
        self.assertEqual(shape.get_name(), "s1")

        if mat is not None:
            self.assertEqual(shape.material, mat)
            self.assertEqual(shape.get_material(), mat)

        self.assertTrue(len(shape.parts) > 0)
        self.assertEqual(shape.get_parts(), shape.parts)

        # TODO: per scene id

    def test_plane(self):
        mat = sapien.render.RenderMaterial()
        shape = sapien.render.RenderShapePlane([0.1, 0.2, 0.3], mat)
        shape = shape.clone()
        self._test_common(shape, mat)
        self.assertTrue(np.allclose(shape.scale, [0.1, 0.2, 0.3]))

    def test_sphere(self):
        mat = sapien.render.RenderMaterial()
        shape = sapien.render.RenderShapeSphere(0.15, mat)
        shape = shape.clone()
        self._test_common(shape, mat)
        self.assertTrue(np.allclose(shape.radius, 0.15))

    def test_box(self):
        mat = sapien.render.RenderMaterial()
        shape = sapien.render.RenderShapeBox([0.2, 0.3, 0.4], mat)
        shape = shape.clone()
        self._test_common(shape, mat)
        self.assertTrue(np.allclose(shape.half_size, [0.2, 0.3, 0.4]))

    def test_capsule(self):
        mat = sapien.render.RenderMaterial()
        shape = sapien.render.RenderShapeCapsule(0.35, 0.2, mat)
        shape = shape.clone()
        self._test_common(shape, mat)
        self.assertTrue(np.allclose(shape.radius, 0.35))
        self.assertTrue(np.allclose(shape.half_length, 0.2))

    def test_cylinder(self):
        mat = sapien.render.RenderMaterial()
        shape = sapien.render.RenderShapeCapsule(0.25, 0.3, mat)
        shape = shape.clone()
        self._test_common(shape, mat)
        self.assertTrue(np.allclose(shape.radius, 0.25))
        self.assertTrue(np.allclose(shape.half_length, 0.3))

    def test_mesh_file(self):
        filename = str(Path(".") / "assets" / "brass_goblets_1k.glb")
        scale = [0.1, 0.1, 0.2]
        shape = sapien.render.RenderShapeTriangleMesh(filename, scale)
        shape = shape.clone()
        self._test_common(shape, None)
        self.assertEqual(len(shape.parts), 3)
        self.assertNotEqual(shape.parts[0].material, shape.parts[1].material)
        self.assertNotEqual(shape.parts[0].material, shape.parts[2].material)
        self.assertNotEqual(shape.parts[1].material, shape.parts[2].material)

        self.assertIsNotNone(shape.parts[0].material.base_color_texture)
        self.assertIsNotNone(shape.parts[0].material.metallic_texture)
        self.assertIsNotNone(shape.parts[0].material.normal_texture)

        self.assertEqual(shape.filename, filename)
        self.assertEqual(shape.get_filename(), filename)

        self.assertTrue(np.allclose(shape.scale, scale))
        self.assertTrue(np.allclose(shape.get_scale(), scale))

        scale = [0.2, 0.3, 0.3]
        shape.set_scale(scale)
        self.assertTrue(np.allclose(shape.scale, scale))

        # force material
        mat = sapien.render.RenderMaterial()
        shape = sapien.render.RenderShapeTriangleMesh(filename, scale, mat)
        shape = shape.clone()
        self.assertEqual(shape.material, mat)
        for p in shape.parts:
            self.assertEqual(p.material, mat)

    def test_mesh_array(self):
        vertices = np.random.rand(10, 3).astype(np.float32)
        triangles = np.array(
            [np.random.choice(np.arange(10), 3, replace=False) for _ in range(20)]
        ).astype(np.uint32)
        normals = np.random.rand(10, 3).astype(np.float32)
        uvs = np.random.rand(10, 2).astype(np.float32)
        mat = sapien.render.RenderMaterial()
        shape = sapien.render.RenderShapeTriangleMesh(
            vertices, triangles, normals, uvs, mat
        )
        shape = shape.clone()
        self._test_common(shape, mat)

        self.assertTrue(np.allclose(shape.parts[0].vertices, vertices))
        self.assertTrue(np.allclose(shape.parts[0].triangles, triangles))
        self.assertTrue(np.allclose(shape.parts[0].get_vertices(), vertices))
        self.assertTrue(np.allclose(shape.parts[0].get_triangles(), triangles))
        self.assertTrue(np.allclose(shape.parts[0].get_vertex_normal(), normals))
        self.assertTrue(np.allclose(shape.parts[0].get_vertex_uv(), uvs))
        self.assertEqual(shape.parts[0].material, mat)
        self.assertEqual(shape.parts[0].get_material(), mat)

    def test_primitive(self):
        mat = sapien.render.RenderMaterial()
        shape = sapien.render.RenderShapeBox([0.2, 0.3, 0.4], mat)

        self.assertTrue(
            np.allclose(shape.get_vertices(), shape.parts[0].get_vertices())
        )
        self.assertTrue(
            np.allclose(shape.get_triangles(), shape.parts[0].get_triangles())
        )
        self.assertTrue(
            np.allclose(shape.get_vertex_normal(), shape.parts[0].get_vertex_normal())
        )
        self.assertTrue(
            np.allclose(shape.get_vertex_uv(), shape.parts[0].get_vertex_uv())
        )
