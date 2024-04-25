import unittest
from pathlib import Path
import numpy as np
from common import rand_pose, pose_equal

import sapien


class TestBody(unittest.TestCase):
    # TODO: test aabb

    def test_body(self):
        body = sapien.render.RenderBodyComponent()
        body.visibility = 0.5
        body.shading_mode = 2
        body = body.clone()
        self.assertAlmostEqual(body.visibility, 0.5)
        self.assertEqual(body.shading_mode, 2)

    def test_attach(self):
        filename = str(Path(".") / "assets" / "brass_goblets_1k.glb")
        shape = sapien.render.RenderShapeTriangleMesh(filename)
        entity = sapien.Entity()
        body = sapien.render.RenderBodyComponent()
        body.attach(shape)
        entity.add_component(body)
        self.assertEqual(body.render_shapes, [shape])

    def test_disable_id(self):
        filename = str(Path(".") / "assets" / "brass_goblets_1k.glb")
        shape = sapien.render.RenderShapeTriangleMesh(filename)
        scene = sapien.Scene()
        entity = sapien.Entity()
        body = sapien.render.RenderBodyComponent()
        body.disable_render_id()
        body.attach(shape)
        entity.add_component(body)
        scene.add_entity(entity)
        self.assertEqual(shape.per_scene_id, 0)
        entity.remove_from_scene()
        self.assertTrue(body.is_render_id_disabled)
        body.enable_render_id()
        entity.add_to_scene(scene)
        self.assertEqual(shape.per_scene_id, 1)
        self.assertFalse(body.is_render_id_disabled)

    def test_custom_property(self):
        body = sapien.render.RenderBodyComponent()
        scene = sapien.Scene()
        entity = sapien.Entity()
        entity.add_component(body)
        scene.add_entity(entity)

        body.set_property("p_int", 1)
        body.set_property("p_float", 1.0)
        body.set_property("p_vec3", [1.0, 2.0, 3.0])

        array = np.random.rand(5, 10, 20).astype(np.float32)
        tex = sapien.render.RenderTexture(
            array, 3, "R32Sfloat", 2, "nearest", "edge", False
        )
        body.set_texture("tex", tex)
        body.set_texture_array("tex_array", [tex])

        # TODO: properties are not copied, need to define this behavior
        # TODO: implement get property
