import unittest
from pathlib import Path
import numpy as np

import sapien


class TestMaterial(unittest.TestCase):
    def test_creation(self):
        emission = np.random.rand(4) + 1.0
        base_color = np.random.rand(4) + [1, 1, 1, 0]
        specular = 0.4
        roughness = 0.9
        metallic = 0.5
        transmission = 0.3
        ior = 1.3
        transmission_roughness = 0.1
        mat = sapien.render.RenderMaterial(
            emission,
            base_color,
            specular,
            roughness,
            metallic,
            transmission,
            ior,
            transmission_roughness,
        )

        self.assertTrue(np.allclose(emission, mat.emission))
        self.assertTrue(np.allclose(base_color, mat.base_color))
        self.assertTrue(np.allclose(specular, mat.specular))
        self.assertTrue(np.allclose(roughness, mat.roughness))
        self.assertTrue(np.allclose(metallic, mat.metallic))
        self.assertTrue(np.allclose(transmission, mat.transmission))
        self.assertTrue(np.allclose(ior, mat.ior))
        self.assertTrue(np.allclose(transmission_roughness, mat.transmission_roughness))

        emission = np.random.rand(4) + 1.0
        base_color = np.random.rand(4) + [1, 1, 1, 0]
        specular = 0.6
        roughness = 0.8
        metallic = 0.3
        transmission = 0.2
        ior = 1.5
        transmission_roughness = 0.2
        mat.set_emission(emission)
        mat.set_base_color(base_color)
        mat.set_specular(specular)
        mat.set_roughness(roughness)
        mat.set_metallic(metallic)
        mat.set_transmission(transmission)
        mat.set_ior(ior)
        mat.set_transmission_roughness(transmission_roughness)

        self.assertTrue(np.allclose(emission, mat.get_emission()))
        self.assertTrue(np.allclose(base_color, mat.get_base_color()))
        self.assertTrue(np.allclose(specular, mat.get_specular()))
        self.assertTrue(np.allclose(roughness, mat.get_roughness()))
        self.assertTrue(np.allclose(metallic, mat.get_metallic()))
        self.assertTrue(np.allclose(transmission, mat.get_transmission()))
        self.assertTrue(np.allclose(ior, mat.get_ior()))
        self.assertTrue(
            np.allclose(transmission_roughness, mat.get_transmission_roughness())
        )

    def test_texture(self):
        mat = sapien.render.RenderMaterial()

        color_tex = sapien.render.RenderTexture2D(
            str(
                Path(".") / "assets" / "texture" / "Metal" / "Metal038_1K-JPG_Color.jpg"
            )
        )
        normal_tex = sapien.render.RenderTexture2D(
            str(
                Path(".")
                / "assets"
                / "texture"
                / "Metal"
                / "Metal038_1K-JPG_NormalGL.jpg"
            ),
            srgb=False,
        )
        roughness_tex = sapien.render.RenderTexture2D(
            str(
                Path(".")
                / "assets"
                / "texture"
                / "Metal"
                / "Metal038_1K-JPG_Roughness.jpg"
            ),
            srgb=False,
        )
        metallic_tex = sapien.render.RenderTexture2D(
            str(
                Path(".")
                / "assets"
                / "texture"
                / "Metal"
                / "Metal038_1K-JPG_Metalness.jpg"
            ),
            srgb=False,
        )

        emission_tex = sapien.render.RenderTexture2D(
            np.array([[1]], dtype=np.float32), "R32Sfloat"
        )
        transmission_tex = sapien.render.RenderTexture2D(
            np.array([[0]], dtype=np.float32), "R32Sfloat"
        )

        # property
        mat.base_color_texture = color_tex
        mat.normal_texture = normal_tex
        mat.metallic_texture = metallic_tex
        mat.roughness_texture = roughness_tex
        mat.emission_texture = emission_tex
        mat.transmission_texture = transmission_tex
        self.assertEqual(mat.base_color_texture, color_tex)
        self.assertEqual(mat.normal_texture, normal_tex)
        self.assertEqual(mat.metallic_texture, metallic_tex)
        self.assertEqual(mat.roughness_texture, roughness_tex)
        self.assertEqual(mat.emission_texture, emission_tex)
        self.assertEqual(mat.transmission_texture, transmission_tex)

        # deprecated diffuse
        mat.set_diffuse_texture(None)
        self.assertIsNone(mat.get_base_color_texture())
        mat.diffuse_texture = color_tex
        self.assertEqual(mat.base_color_texture, color_tex)

        # get set
        mat.set_base_color_texture(None)
        self.assertIsNone(mat.get_base_color_texture())
        mat.set_normal_texture(None)
        self.assertIsNone(mat.get_normal_texture())
        mat.set_metallic_texture(None)
        self.assertIsNone(mat.get_metallic_texture())
        mat.set_roughness_texture(None)
        self.assertIsNone(mat.get_roughness_texture())
        mat.set_emission_texture(None)
        self.assertIsNone(mat.get_emission_texture())
        mat.set_transmission_texture(None)
        self.assertIsNone(mat.get_transmission_texture())
