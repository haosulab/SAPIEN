import unittest
import sapien.core as sapien


class TestCreation(unittest.TestCase):
    def test_create_scene(self):
        engine = sapien.Engine()
        scene = engine.create_scene()
