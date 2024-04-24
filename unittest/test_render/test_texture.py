import unittest
from pathlib import Path
import numpy as np

import sapien


class TestTexture(unittest.TestCase):
    def test_creation(self):
        array = np.random.rand(5, 10, 20).astype(np.float32)
        tex = sapien.render.RenderTexture(
            array, 3, "R32Sfloat", 2, "nearest", "edge", False
        )
        self.assertEqual(tex.filter_mode, "nearest")
        self.assertEqual(tex.address_mode, "edge")
        self.assertEqual(tex.width, 20)
        self.assertEqual(tex.height, 10)
        self.assertEqual(tex.depth, 5)
        self.assertEqual(tex.channels, 1)
        self.assertEqual(tex.mipmap_levels, 2)
        self.assertEqual(tex.format, "R32Sfloat")
        self.assertEqual(tex.is_srgb, False)

        tex = sapien.render.RenderTexture(
            array, 3, "R32Sfloat", 2, "linear", "repeat", False
        )
        self.assertEqual(tex.filter_mode, "linear")
        self.assertEqual(tex.address_mode, "repeat")

        tex = sapien.render.RenderTexture(
            array, 3, "R32Sfloat", 2, "linear", "mirror", False
        )
        self.assertEqual(tex.filter_mode, "linear")
        self.assertEqual(tex.address_mode, "mirror")

        tex = sapien.render.RenderTexture(
            array, 3, "R32Sfloat", 2, "linear", "border", False
        )
        self.assertEqual(tex.filter_mode, "linear")
        self.assertEqual(tex.address_mode, "border")

        self.assertTrue(np.allclose(array, tex.download().squeeze()))

        array *= 0.5
        tex.upload(array[..., None])
        self.assertTrue(np.allclose(array, tex.download().squeeze()))


class TestTexture2D(unittest.TestCase):
    def test_file(self):
        tex = sapien.render.RenderTexture2D(
            str(Path(".") / "assets" / "texture" / "uv.png"),
            2,
            "linear",
            "mirror",
            False,
        )
        self.assertEqual(tex.filter_mode, "linear")
        self.assertEqual(tex.address_mode, "mirror")
        self.assertEqual(tex.width, 1024)
        self.assertEqual(tex.height, 1024)
        self.assertEqual(tex.channels, 4)
        self.assertEqual(tex.mipmap_levels, 2)
        self.assertEqual(tex.format, "R8G8B8A8Unorm")
        self.assertEqual(tex.is_srgb, False)

        tex = sapien.render.RenderTexture2D(
            str(Path(".") / "assets" / "texture" / "uv.png"),
            1,
            "nearest",
            "repeat",
            True,
        )
        self.assertEqual(tex.filter_mode, "nearest")
        self.assertEqual(tex.address_mode, "repeat")
        self.assertEqual(tex.width, 1024)
        self.assertEqual(tex.height, 1024)
        self.assertEqual(tex.channels, 4)
        self.assertEqual(tex.mipmap_levels, 1)
        self.assertEqual(tex.format, "R8G8B8A8Unorm")
        self.assertEqual(tex.is_srgb, True)

    def test_creation(self):
        array = np.random.rand(5, 10, 4).astype(np.float32)
        tex = sapien.render.RenderTexture2D(
            array, "R32G32B32A32Sfloat", 3, "nearest", "edge", False
        )
        self.assertEqual(tex.filter_mode, "nearest")
        self.assertEqual(tex.address_mode, "edge")
        self.assertEqual(tex.width, 10)
        self.assertEqual(tex.height, 5)
        self.assertEqual(tex.channels, 4)
        self.assertEqual(tex.mipmap_levels, 3)
        self.assertEqual(tex.format, "R32G32B32A32Sfloat")
        self.assertEqual(tex.is_srgb, False)

        self.assertTrue(np.allclose(array, tex.download().squeeze()))

        array *= 0.5
        tex.upload(array)
        self.assertTrue(np.allclose(array, tex.download().squeeze()))


class TestCubemap(unittest.TestCase):
    def test_load_export(self):
        cubemap = sapien.render.RenderCubemap(
            str(Path(".") / "assets" / "texture" / "CubeLayoutLatLong.png")
        )
        import tempfile
        import os

        tempdir = tempfile.gettempdir()
        outfile = str(Path(tempdir) / "test_cubemap.ktx")
        cubemap.export(outfile)
        tex = sapien.render.RenderCubemap(outfile)
        os.unlink(outfile)

        system = sapien.render.RenderSystem()
        system.set_cubemap(tex)
        self.assertEqual(system.get_cubemap(), tex)

        tex = sapien.asset.create_dome_envmap()
        system.set_cubemap(tex)
        self.assertEqual(system.get_cubemap(), tex)
