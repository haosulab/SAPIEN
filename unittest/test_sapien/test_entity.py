import unittest
import sapien
from common import rand_pose, pose_equal


class TestEntity(unittest.TestCase):
    def test_scene(self):
        entity = sapien.Entity()
        scene = sapien.Scene()
        self.assertIsNone(entity.scene)
        entity.add_to_scene(scene)
        self.assertEqual(entity.scene, scene)
        entity.remove_from_scene()
        self.assertIsNone(entity.scene)

    def test_id(self):
        scene0 = sapien.Scene()
        scene1 = sapien.Scene()
        entity0 = sapien.Entity()
        entity1 = sapien.Entity()
        self.assertEqual(entity0.global_id, entity0.get_global_id())
        self.assertNotEqual(entity0.global_id, entity1.global_id)
        scene0.add_entity(entity0)
        scene1.add_entity(entity1)
        self.assertEqual(entity0.get_per_scene_id(), entity0.per_scene_id)
        self.assertEqual(entity0.per_scene_id, entity1.per_scene_id)

    def test_component(self):
        entity = sapien.Entity()
        c0 = sapien.physx.PhysxRigidDynamicComponent()
        c1 = sapien.render.RenderBodyComponent()
        entity.add_component(c0).add_component(c1)

        self.assertEqual(
            entity.find_component_by_type(sapien.physx.PhysxRigidBaseComponent), c0
        )
        self.assertIsNone(
            entity.find_component_by_type(sapien.physx.PhysxArticulationLinkComponent)
        )

        self.assertEqual(entity.components, entity.get_components())
        self.assertEqual(entity.components, [c0, c1])

        entity.remove_component(c1)
        self.assertEqual(entity.components, [c0])

    def test_name(self):
        entity = sapien.Entity()
        entity.name = "e0"
        self.assertEqual(entity.name, entity.get_name())
        self.assertEqual(entity.name, "e0")

    def test_pose(self):
        entity = sapien.Entity()
        pose = rand_pose()
        entity.pose = pose
        self.assertTrue(pose_equal(entity.pose, entity.get_pose()))
        self.assertTrue(pose_equal(entity.pose, pose))

        pose = rand_pose()
        entity.set_pose(pose)
        self.assertTrue(pose_equal(entity.pose, pose))
