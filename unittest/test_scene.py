import unittest
import sapien
from common import rand_pose, rand_size
import numpy as np


class TestScene(unittest.TestCase):
    def test_create_scene(self):
        scene = sapien.Scene()
        self.assertTrue(isinstance(scene.physx_system, sapien.physx.PhysxCpuSystem))
        self.assertTrue(isinstance(scene.render_system, sapien.render.RenderSystem))
        scene = sapien.Scene([])
        with self.assertRaises(RuntimeError):
            scene.physx_system
        with self.assertRaises(RuntimeError):
            scene.render_system

    def test_config(self):
        config = sapien.physx.PhysxSceneConfig()
        config.gravity = [0, 0, -1]
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

        sapien.physx.set_scene_config(config)
        scene = sapien.Scene()
        config = scene.physx_system.get_config()

        self.assertAlmostEqual(tuple(config.gravity), (0, 0, -1))
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

        # self.assertAlmostEqual(config.default_static_friction, 0.2)
        # self.assertAlmostEqual(config.default_dynamic_friction, 0.19)
        # self.assertAlmostEqual(config.default_restitution, 0.05)

    def test_actor_builder(self):
        scene = sapien.Scene()
        builder = scene.create_actor_builder()

        poses = [rand_pose() for _ in range(10)]
        sizes = [rand_size() for _ in range(10)]
        mats = [sapien.physx.PhysxMaterial(*np.random.rand(3)) for _ in range(10)]
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

        builder.add_convex_collision_from_file(
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

        collisions = builder.collision_records
        self.assertEqual(len(collisions), 5)
        for i, c in enumerate(collisions):
            self.assertTrue(np.allclose(c.pose.p, poses[i].p))
            self.assertTrue(np.allclose(c.pose.q, poses[i].q))
            self.assertEqual(c.material, mats[i])
            if c.type != "nonconvex_mesh":
                self.assertTrue(np.allclose(c.density, densities[i]))

        self.assertEqual(collisions[0].type, "box")
        self.assertEqual(collisions[1].type, "sphere")
        self.assertEqual(collisions[2].type, "capsule")
        self.assertEqual(collisions[3].type, "convex_mesh")
        self.assertEqual(collisions[4].type, "nonconvex_mesh")

        self.assertTrue(np.allclose(collisions[0].scale, sizes[0]))
        self.assertTrue(np.allclose(collisions[1].radius, sizes[1][0]))
        self.assertTrue(np.allclose(collisions[2].radius, sizes[2][1]))
        self.assertTrue(np.allclose(collisions[2].length, sizes[2][0]))
        self.assertTrue(np.allclose(collisions[3].scale, sizes[3]))
        self.assertTrue(np.allclose(collisions[4].scale, sizes[4]))

        body = builder.build_kinematic().find_component_by_type(
            sapien.physx.PhysxRigidBaseComponent
        )
        self.assertTrue(body.kinematic)
        self.assertEqual(
            tuple(c.__class__ for c in body.get_collision_shapes()),
            (
                sapien.physx.PhysxCollisionShapeBox,
                sapien.physx.PhysxCollisionShapeSphere,
                sapien.physx.PhysxCollisionShapeCapsule,
                sapien.physx.PhysxCollisionShapeConvexMesh,
                sapien.physx.PhysxCollisionShapeTriangleMesh,
            ),
        )

        body = builder.build_static().find_component_by_type(
            sapien.physx.PhysxRigidBaseComponent
        )
        self.assertTrue(isinstance(body, sapien.physx.PhysxRigidStaticComponent))
        self.assertEqual(
            tuple(c.__class__ for c in body.get_collision_shapes()),
            (
                sapien.physx.PhysxCollisionShapeBox,
                sapien.physx.PhysxCollisionShapeSphere,
                sapien.physx.PhysxCollisionShapeCapsule,
                sapien.physx.PhysxCollisionShapeConvexMesh,
                sapien.physx.PhysxCollisionShapeTriangleMesh,
            ),
        )

        del builder.collision_records[4]
        builder.physx_body_type = "dynamic"
        body = builder.build().find_component_by_type(
            sapien.physx.PhysxRigidBaseComponent
        )
        self.assertFalse(body.kinematic)
        self.assertEqual(
            tuple(c.__class__ for c in body.get_collision_shapes()),
            (
                sapien.physx.PhysxCollisionShapeBox,
                sapien.physx.PhysxCollisionShapeSphere,
                sapien.physx.PhysxCollisionShapeCapsule,
                sapien.physx.PhysxCollisionShapeConvexMesh,
            ),
        )

        # TODO: check details of the built shapes
