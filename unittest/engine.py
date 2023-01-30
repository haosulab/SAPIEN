import unittest
import sapien.core as sapien


class TestEngine(unittest.TestCase):
    def test_create_engine(self):
        engine = sapien.Engine()

    def test_duplicate(self):
        engine = sapien.Engine()
        engine1 = sapien.Engine()
        engine = sapien.Engine()
        engine = sapien.Engine()
        engine = None
        engine = sapien.Engine()

    def test_create_physical_material(self):
        engine = sapien.Engine()
        mat = engine.create_physical_material(0.15, 0.14, 0.45)
        self.assertAlmostEqual(mat.static_friction, 0.15)
        self.assertAlmostEqual(mat.dynamic_friction, 0.14)
        self.assertAlmostEqual(mat.restitution, 0.45)
        # TODO: invalid value validation?
