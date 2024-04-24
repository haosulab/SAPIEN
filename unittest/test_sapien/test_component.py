import unittest
import sapien
from common import rand_pose, pose_equal


class CustomComponent(sapien.Component):
    def __init__(self):
        self.add_called = 0
        self.remove_called = 0
        self.set_pose_called = 0
        super().__init__()

    def on_add_to_scene(self, scene):
        self.add_called += 1

    def on_remove_from_scene(self, scene):
        self.remove_called += 1

    def on_set_pose(self, pose):
        self.set_pose_called += 1


class TestComponent(unittest.TestCase):
    def test_pose(self):
        pose = rand_pose()
        entity = sapien.Entity()
        entity.set_pose(pose)
        comp = CustomComponent()
        entity.add_component(comp)
        self.assertTrue(pose_equal(comp.pose, pose))
        self.assertTrue(pose_equal(comp.get_pose(), pose))
        self.assertTrue(pose_equal(comp.entity_pose, pose))
        self.assertTrue(pose_equal(comp.get_entity_pose(), pose))

        pose = rand_pose()
        comp.set_pose(pose)
        self.assertTrue(pose_equal(entity.pose, pose))

        pose = rand_pose()
        comp.set_entity_pose(pose)
        self.assertTrue(pose_equal(entity.pose, pose))

        self.assertTrue(entity, comp.entity)

    def test_name(self):
        comp = CustomComponent()
        comp.name = "c0"
        self.assertEqual(comp.name, "c0")

        comp.set_name("c1")
        self.assertEqual(comp.get_name(), "c1")

    def test_lifecycle(self):
        scene = sapien.Scene()
        pose = rand_pose()
        entity = sapien.Entity()
        comp = CustomComponent()

        self.assertEqual(comp.set_pose_called, 0)
        entity.add_component(comp)
        self.assertEqual(comp.set_pose_called, 1)
        entity.set_pose(pose)
        self.assertEqual(comp.set_pose_called, 2)

        self.assertEqual(comp.add_called, 0)
        entity.add_to_scene(scene)
        self.assertEqual(comp.add_called, 1)

        self.assertEqual(comp.remove_called, 0)
        entity.remove_from_scene()
        self.assertEqual(comp.remove_called, 1)

        comp.disable()
        self.assertEqual(comp.add_called, 1)
        entity.add_to_scene(scene)
        self.assertEqual(comp.add_called, 1)
        comp.enable()
        self.assertEqual(comp.add_called, 2)
        comp.disable()
        self.assertEqual(comp.add_called, 2)
        self.assertEqual(comp.remove_called, 2)
        entity.remove_from_scene()
        self.assertEqual(comp.remove_called, 2)

        comp.disable()
        entity.add_to_scene(scene)
        entity.remove_component(comp)
        self.assertEqual(comp.add_called, 2)
        self.assertEqual(comp.remove_called, 2)

        comp.enable()
        entity.add_component(comp)
        self.assertEqual(comp.add_called, 3)

        entity.remove_component(comp)
        self.assertEqual(comp.remove_called, 3)
