import unittest
from pathlib import Path

import sapien


class TestGlobals(unittest.TestCase):
    def test_globals(self):
        sapien.render.get_device_summary()

        sapien.render.set_camera_shader_dir("rt")
        self.assertTrue(sapien.render.get_camera_shader_dir().endswith("rt"))
        sapien.render.set_viewer_shader_dir("rt")
        self.assertTrue(sapien.render.get_viewer_shader_dir().endswith("rt"))

        self.assertEqual(
            sapien.render.get_imgui_ini_filename(),
            str(Path.home() / ".sapien" / "imgui.ini"),
        )

        sapien.render.set_msaa(4)
        self.assertEqual(sapien.render.get_msaa(), 4)

        sapien.render.set_ray_tracing_denoiser("oidn")
        self.assertEqual(sapien.render.get_ray_tracing_denoiser(), "oidn")

        sapien.render.set_ray_tracing_dof_aperture(0.01)
        self.assertAlmostEqual(sapien.render.get_ray_tracing_dof_aperture(), 0.01)

        sapien.render.set_ray_tracing_dof_plane(0.1)
        self.assertAlmostEqual(sapien.render.get_ray_tracing_dof_plane(), 0.1)

        sapien.render.set_ray_tracing_path_depth(7)
        self.assertEqual(sapien.render.get_ray_tracing_path_depth(), 7)

        sapien.render.set_ray_tracing_samples_per_pixel(5)
        self.assertEqual(sapien.render.get_ray_tracing_samples_per_pixel(), 5)

        sapien.render.set_picture_format("Color", "r16g16b16a16Sfloat")

        # reset settings
        sapien.render.set_msaa(1)
        sapien.render.set_camera_shader_dir("default")
        sapien.render.set_viewer_shader_dir("default")
        sapien.render.set_picture_format("Color", "r32g32b32a32Sfloat")

        # TODO: test global config
        # TODO: test these configs actually take effects

    # TODO: test load scene
    # TODO: test clear cache
