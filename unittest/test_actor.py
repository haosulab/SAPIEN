import unittest
import sapien.core as sapien
from common import *
import numpy as np


class TestActor(unittest.TestCase):
    def test_build_box(self):
        engine = sapien.Engine()
        scene = engine.create_scene()

        builder = scene.create_actor_builder()
        pose = rand_pose()
        half_size = np.random.rand(3)
        mat = np.random.rand(3)
        density = np.random.rand() * 1000 + 100
        min_pr = np.random.rand()
        pr = min_pr + np.random.rand()

        builder.add_box_collision(
            pose,
            half_size,
            engine.create_physical_material(*mat),
            density,
            pr,
            min_pr,
            False,
        )
        actor = builder.build()

        shapes = actor.get_collision_shapes()
        self.assertEqual(len(shapes), 1)
        shape = shapes[0]

        self.assertEqual(shape.actor, actor)

        # physical material
        self.assertAlmostEqual(
            shape.get_physical_material().static_friction, mat[0], places=5
        )
        self.assertAlmostEqual(
            shape.get_physical_material().dynamic_friction, mat[1], places=5
        )
        self.assertAlmostEqual(
            shape.get_physical_material().restitution, mat[2], places=5
        )

        self.assertTrue(pose_equal(shape.get_local_pose(), pose))
        self.assertAlmostEqual(shape.patch_radius, pr, places=5)
        self.assertAlmostEqual(shape.min_patch_radius, min_pr, places=5)
        self.assertEqual(shape.type, "box")
        self.assertTrue(np.allclose(shape.geometry.half_size, half_size))

        shape.set_collision_groups(3, 4, 5, 6)
        self.assertEqual(shape.get_collision_groups(), [3, 4, 5, 6])

    def test_build_sphere(self):
        engine = sapien.Engine()
        scene = engine.create_scene()

        builder = scene.create_actor_builder()
        pose = rand_pose()
        radius = np.random.rand() + 0.1
        mat = np.random.rand(3)
        density = np.random.rand() * 1000 + 100
        min_pr = np.random.rand()
        pr = min_pr + np.random.rand()

        builder.add_sphere_collision(
            pose,
            radius,
            engine.create_physical_material(*mat),
            density,
            pr,
            min_pr,
            False,
        )
        actor = builder.build()

        shapes = actor.get_collision_shapes()
        self.assertEqual(len(shapes), 1)
        shape = shapes[0]

        self.assertEqual(shape.actor, actor)

        # physical material
        self.assertAlmostEqual(
            shape.get_physical_material().static_friction, mat[0], places=5
        )
        self.assertAlmostEqual(
            shape.get_physical_material().dynamic_friction, mat[1], places=5
        )
        self.assertAlmostEqual(
            shape.get_physical_material().restitution, mat[2], places=5
        )

        self.assertTrue(pose_equal(shape.get_local_pose(), pose))
        self.assertAlmostEqual(shape.patch_radius, pr, places=5)
        self.assertAlmostEqual(shape.min_patch_radius, min_pr, places=5)
        self.assertAlmostEqual(shape.type, "sphere", places=5)
        self.assertAlmostEqual(shape.geometry.radius, radius, places=5)

        shape.set_collision_groups(3, 4, 5, 6)
        self.assertEqual(shape.get_collision_groups(), [3, 4, 5, 6])

    def test_build_sphere(self):
        engine = sapien.Engine()
        scene = engine.create_scene()

        builder = scene.create_actor_builder()
        pose = rand_pose()
        radius = np.random.rand() + 0.1
        half_length = np.random.rand() + 0.1
        mat = np.random.rand(3)
        density = np.random.rand() * 1000 + 100
        min_pr = np.random.rand()
        pr = min_pr + np.random.rand()

        builder.add_capsule_collision(
            pose,
            radius,
            half_length,
            engine.create_physical_material(*mat),
            density,
            pr,
            min_pr,
            False,
        )
        actor = builder.build()

        shapes = actor.get_collision_shapes()
        self.assertEqual(len(shapes), 1)
        shape = shapes[0]

        self.assertEqual(shape.actor, actor)

        # physical material
        self.assertAlmostEqual(
            shape.get_physical_material().static_friction, mat[0], places=5
        )
        self.assertAlmostEqual(
            shape.get_physical_material().dynamic_friction, mat[1], places=5
        )
        self.assertAlmostEqual(
            shape.get_physical_material().restitution, mat[2], places=5
        )

        self.assertTrue(pose_equal(shape.get_local_pose(), pose))
        self.assertAlmostEqual(shape.patch_radius, pr, places=5)
        self.assertAlmostEqual(shape.min_patch_radius, min_pr, places=5)
        self.assertEqual(shape.type, "capsule")
        self.assertAlmostEqual(shape.geometry.radius, radius, places=5)
        self.assertAlmostEqual(shape.geometry.half_length, half_length, places=5)

        shape.set_collision_groups(3, 4, 5, 6)
        self.assertEqual(shape.get_collision_groups(), [3, 4, 5, 6])

    def test_build_multiple(self):
        engine = sapien.Engine()
        scene = engine.create_scene()

        builder = scene.create_actor_builder()

        pose = rand_pose()
        half_size = np.random.rand(3)
        half_length = np.random.rand() + 0.1
        mat = np.random.rand(3)
        density = np.random.rand() * 1000 + 100
        min_pr = np.random.rand()
        pr = min_pr + np.random.rand()

        builder.add_box_collision(
            pose,
            half_size,
            engine.create_physical_material(*mat),
            density,
            pr,
            min_pr,
            False,
        )

        pose = rand_pose()
        radius = np.random.rand() + 0.1
        mat = np.random.rand(3)
        density = np.random.rand() * 1000 + 100
        min_pr = np.random.rand()
        pr = min_pr + np.random.rand()

        builder.add_sphere_collision(
            pose,
            radius,
            engine.create_physical_material(*mat),
            density,
            pr,
            min_pr,
            False,
        )

        pose = rand_pose()
        radius = np.random.rand() + 0.1
        half_length = np.random.rand() + 0.1
        mat = np.random.rand(3)
        density = np.random.rand() * 1000 + 100
        min_pr = np.random.rand()
        pr = min_pr + np.random.rand()

        builder.add_capsule_collision(
            pose,
            radius,
            half_length,
            engine.create_physical_material(*mat),
            density,
            pr,
            min_pr,
            False,
        )

        actor = builder.build()

        shapes = actor.get_collision_shapes()
        self.assertEqual(len(shapes), 3)
