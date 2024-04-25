import unittest
from pathlib import Path
import numpy as np
from common import rand_pose, pose_equal

import sapien


class TestLight(unittest.TestCase):
    def test_common(self):
        for cls in [
            sapien.render.RenderPointLightComponent,
            sapien.render.RenderDirectionalLightComponent,
            sapien.render.RenderSpotLightComponent,
            sapien.render.RenderTexturedLightComponent,
            sapien.render.RenderParallelogramLightComponent,
        ]:
            p0 = rand_pose()
            p1 = rand_pose()
            p2 = rand_pose()
            entity = sapien.Entity()
            entity.set_pose(p0)

            l = cls()
            l.color = [0.9, 0.8, 0]
            l.local_pose = p1
            l.shadow = True
            l.shadow_near = 0.3
            l.shadow_far = 40
            l.shadow_map_size = 100

            entity.add_component(l)

            self.assertTrue(np.allclose(l.color, [0.9, 0.8, 0]))
            self.assertTrue(pose_equal(l.local_pose, p1))
            self.assertTrue(pose_equal(l.global_pose, p0 * p1))
            self.assertEqual(l.shadow, True)
            self.assertAlmostEqual(l.shadow_near, 0.3)
            self.assertAlmostEqual(l.shadow_far, 40)
            self.assertEqual(l.shadow_map_size, 100)

            l.disable_shadow()
            self.assertEqual(l.shadow, False)
            l.enable_shadow()
            self.assertEqual(l.shadow, True)

            l.set_color([0.9, 0.8, 0.7])
            l.set_local_pose(p1)
            l.set_shadow_near(0.5)
            l.set_shadow_far(20)
            l.set_shadow_map_size(120)

            self.assertTrue(np.allclose(l.get_color(), [0.9, 0.8, 0.7]))
            self.assertTrue(pose_equal(l.get_local_pose(), p1))
            self.assertTrue(pose_equal(l.get_global_pose(), p0 * p1))
            self.assertAlmostEqual(l.get_shadow_near(), 0.5)
            self.assertAlmostEqual(l.get_shadow_far(), 20)
            self.assertEqual(l.get_shadow_map_size(), 120)

    def test_directional(self):
        l = sapien.render.RenderDirectionalLightComponent()
        l.shadow_half_size = 11
        self.assertAlmostEqual(l.shadow_half_size, 11)

        l.set_shadow_half_size(9)
        self.assertAlmostEqual(l.get_shadow_half_size(), 9)

    def test_spot(self):
        l = sapien.render.RenderSpotLightComponent()
        l.inner_fov = 0.1
        l.outer_fov = 0.3
        self.assertAlmostEqual(l.inner_fov, 0.1)
        self.assertAlmostEqual(l.outer_fov, 0.3)

        l.set_inner_fov(0.2)
        l.set_outer_fov(0.4)
        self.assertAlmostEqual(l.get_inner_fov(), 0.2)
        self.assertAlmostEqual(l.get_outer_fov(), 0.4)

    def test_parallelogram(self):
        l = sapien.render.RenderParallelogramLightComponent()
        l.set_shape(0.1, 0.3, np.pi / 3)
        self.assertAlmostEqual(l.half_width, 0.1)
        self.assertAlmostEqual(l.half_height, 0.3)
        self.assertAlmostEqual(l.angle, np.pi / 3)

        self.assertAlmostEqual(l.get_half_width(), 0.1)
        self.assertAlmostEqual(l.get_half_height(), 0.3)
        self.assertAlmostEqual(l.get_angle(), np.pi / 3)

    def test_textured(self):
        l = sapien.render.RenderTexturedLightComponent()
        tex = sapien.render.RenderTexture2D(
            np.array([[1]], dtype=np.float32), "R32Sfloat"
        )
        l.texture = tex

        self.assertEqual(l.texture, tex)
        self.assertEqual(l.get_texture(), tex)

        l.set_texture(None)
        self.assertEqual(l.texture, None)
