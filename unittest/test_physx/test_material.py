import unittest
import numpy as np

import sapien


class TestMaterial(unittest.TestCase):
    def test_creation(self):
        mat = sapien.physx.PhysxMaterial(0.2, 0.1, 0.05)
        self.assertAlmostEqual(mat.static_friction, 0.2)
        self.assertAlmostEqual(mat.dynamic_friction, 0.1)
        self.assertAlmostEqual(mat.restitution, 0.05)
        mat.static_friction = 0.5
        mat.dynamic_friction = 0.4
        mat.restitution = 0.9
        self.assertAlmostEqual(mat.get_static_friction(), 0.5)
        self.assertAlmostEqual(mat.get_dynamic_friction(), 0.4)
        self.assertAlmostEqual(mat.get_restitution(), 0.9)
        mat.set_static_friction(0.2)
        mat.set_dynamic_friction(0.1)
        mat.set_restitution(0.05)
        self.assertAlmostEqual(mat.static_friction, 0.2)
        self.assertAlmostEqual(mat.dynamic_friction, 0.1)
        self.assertAlmostEqual(mat.restitution, 0.05)

    def test_default(self):
        sapien.physx.set_default_material(1.1, 1.05, 0.8)
        mat = sapien.physx.get_default_material()
        self.assertAlmostEqual(mat.static_friction, 1.1)
        self.assertAlmostEqual(mat.dynamic_friction, 1.05)
        self.assertAlmostEqual(mat.restitution, 0.8)
        # TODO: test it is actually effective
