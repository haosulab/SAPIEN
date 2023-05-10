import unittest
import sapien.core as sapien
from common import *
import numpy as np


class TestScene(unittest.TestCase):
    def test_create_scene(self):
        engine = sapien.Engine()
        scene = engine.create_scene()

    def test_config(self):
        engine = sapien.Engine()
        config = sapien.SceneConfig()
        config.gravity = [0, 0, -1]
        config.default_static_friction = 0.2
        config.default_dynamic_friction = 0.19
        config.default_restitution = 0.05
        config.bounce_threshold = 1.0
        config.sleep_threshold = 0.001
        config.contact_offset = 0.02
        config.solver_iterations = 4
        config.solver_velocity_iterations = 2
        config.enable_pcm = True
        config.enable_tgs = True
        config.enable_ccd = True
        config.enable_enhanced_determinism = True
        config.enable_friction_every_iteration = False
        config.disable_collision_visual = True
        scene = engine.create_scene(config)
        config1 = scene.get_config()

        self.assertAlmostEqual(tuple(config.gravity), (0, 0, -1))
        self.assertAlmostEqual(config.default_static_friction, 0.2)
        self.assertAlmostEqual(config.default_dynamic_friction, 0.19)
        self.assertAlmostEqual(config.default_restitution, 0.05)
        self.assertAlmostEqual(config.bounce_threshold, 1.0)
        self.assertAlmostEqual(config.sleep_threshold, 0.001)
        self.assertAlmostEqual(config.contact_offset, 0.02)
        self.assertEqual(config.solver_iterations, 4)
        self.assertEqual(config.solver_velocity_iterations, 2)
        self.assertEqual(config.enable_pcm, True)
        self.assertEqual(config.enable_tgs, True)
        self.assertEqual(config.enable_ccd, True)
        self.assertEqual(config.enable_enhanced_determinism, True)
        self.assertEqual(config.enable_friction_every_iteration, False)
        self.assertEqual(config.disable_collision_visual, True)

    def test_actor_builder(self):
        engine = sapien.Engine()
        scene = engine.create_scene()
        builder = scene.create_actor_builder()

        poses = [rand_pose() for _ in range(10)]
        sizes = [rand_size() for _ in range(10)]
        mats = [engine.create_physical_material(*np.random.rand(3)) for _ in range(10)]
        densities = [np.random.rand() * 1000 for _ in range(10)]
        patch_radii = [np.random.rand() for _ in range(10)]
        min_patch_radii = [np.random.rand() for _ in range(10)]

        idx = 0
        builder.add_box_collision(
            pose=poses[idx],
            half_size=sizes[idx],
            material=mats[idx],
            density=densities[idx],
            patch_radius=patch_radii[idx],
            min_patch_radius=min_patch_radii[idx],
            is_trigger=False,
        )
        idx += 1

        builder.add_sphere_collision(
            pose=poses[idx],
            radius=sizes[idx][0],
            material=mats[idx],
            density=densities[idx],
            patch_radius=patch_radii[idx],
            min_patch_radius=min_patch_radii[idx],
            is_trigger=False,
        )
        idx += 1

        builder.add_capsule_collision(
            pose=poses[idx],
            radius=sizes[idx][1],
            half_length=sizes[idx][0],
            material=mats[idx],
            density=densities[idx],
            patch_radius=patch_radii[idx],
            min_patch_radius=min_patch_radii[idx],
            is_trigger=False,
        )
        idx += 1

        builder.add_collision_from_file(
            filename="assets/cone.stl",
            pose=poses[idx],
            scale=sizes[idx],
            material=mats[idx],
            density=densities[idx],
            patch_radius=patch_radii[idx],
            min_patch_radius=min_patch_radii[idx],
            is_trigger=False,
        )
        idx += 1

        builder.add_nonconvex_collision_from_file(
            filename="assets/torus.stl",
            pose=poses[idx],
            scale=sizes[idx],
            material=mats[idx],
            patch_radius=patch_radii[idx],
            min_patch_radius=min_patch_radii[idx],
            is_trigger=False,
        )

        collisions = builder.get_collisions()
        self.assertEqual(len(collisions), 5)
        for i, c in enumerate(collisions):
            self.assertTrue(np.allclose(c.pose.p, poses[i].p))
            self.assertTrue(np.allclose(c.pose.q, poses[i].q))
            self.assertEqual(c.material, mats[i])
            if c.type != "Nonconvex":
                self.assertTrue(np.allclose(c.density, densities[i]))

        self.assertEqual(collisions[0].type, "Box")
        self.assertEqual(collisions[1].type, "Sphere")
        self.assertEqual(collisions[2].type, "Capsule")
        self.assertEqual(collisions[3].type, "Mesh")
        self.assertEqual(collisions[4].type, "Nonconvex")

        self.assertTrue(np.allclose(collisions[0].scale, sizes[0]))
        self.assertTrue(np.allclose(collisions[1].radius, sizes[1][0]))
        self.assertTrue(np.allclose(collisions[2].radius, sizes[2][1]))
        self.assertTrue(np.allclose(collisions[2].length, sizes[2][0]))
        self.assertTrue(np.allclose(collisions[3].scale, sizes[3]))
        self.assertTrue(np.allclose(collisions[4].scale, sizes[4]))

        actor = builder.build_kinematic()
        self.assertEqual(actor.type, "kinematic")
        self.assertEqual(
            tuple(c.type for c in actor.get_collision_shapes()),
            ("box", "sphere", "capsule", "convex_mesh", "nonconvex_mesh"),
        )

        actor = builder.build_static()
        self.assertEqual(actor.type, "static")
        self.assertEqual(
            tuple(c.type for c in actor.get_collision_shapes()),
            ("box", "sphere", "capsule", "convex_mesh", "nonconvex_mesh"),
        )

        builder.remove_collision_at(4)
        actor = builder.build()
        self.assertEqual(actor.type, "dynamic")
        self.assertEqual(
            tuple(c.type for c in actor.get_collision_shapes()),
            ("box", "sphere", "capsule", "convex_mesh"),
        )

        # TODO: check details of the built shapes
