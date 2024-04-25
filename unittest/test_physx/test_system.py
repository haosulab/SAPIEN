import unittest
import numpy as np
from common import pose_equal

import sapien


class TestSystem(unittest.TestCase):
    def test_timestep(self):
        system = sapien.physx.PhysxCpuSystem()
        system.timestep = 1 / 240
        self.assertAlmostEqual(system.get_timestep(), 1 / 240)
        system.set_timestep(1 / 260)
        self.assertAlmostEqual(system.timestep, 1 / 260)

    def test_config(self):
        config = sapien.physx.PhysxSceneConfig()
        config.gravity = [0, 0, -1]
        config.bounce_threshold = 1.0
        config.enable_pcm = True
        config.enable_tgs = True
        config.enable_ccd = True
        config.enable_enhanced_determinism = True
        config.enable_friction_every_iteration = False

        sapien.physx.set_scene_config(config)
        system = sapien.physx.PhysxCpuSystem()
        scene = sapien.Scene([system])
        config = scene.physx_system.get_config()

        self.assertAlmostEqual(tuple(config.gravity), (0, 0, -1))
        self.assertAlmostEqual(config.bounce_threshold, 1.0)
        self.assertEqual(config.enable_pcm, True)
        self.assertEqual(config.enable_tgs, True)
        self.assertEqual(config.enable_ccd, True)
        self.assertEqual(config.enable_enhanced_determinism, True)
        self.assertEqual(config.enable_friction_every_iteration, False)

        sapien.physx.set_scene_config(
            gravity=[0, 0, -2],
            bounce_threshold=2.0,
            enable_pcm=False,
            enable_tgs=False,
            enable_ccd=False,
            enable_enhanced_determinism=False,
            enable_friction_every_iteration=True,
        )
        system = sapien.physx.PhysxCpuSystem()
        scene = sapien.Scene([system])
        config = scene.physx_system.get_config()

        self.assertAlmostEqual(tuple(config.gravity), (0, 0, -2))
        self.assertAlmostEqual(config.bounce_threshold, 2.0)
        self.assertEqual(config.enable_pcm, False)
        self.assertEqual(config.enable_tgs, False)
        self.assertEqual(config.enable_ccd, False)
        self.assertEqual(config.enable_enhanced_determinism, False)
        self.assertEqual(config.enable_friction_every_iteration, True)

        config = sapien.physx.PhysxShapeConfig()
        config.contact_offset = 0.02
        config.rest_offset = 0.001
        sapien.physx.set_shape_config(config)
        self.assertAlmostEqual(sapien.physx.get_shape_config().contact_offset, 0.02)
        self.assertAlmostEqual(sapien.physx.get_shape_config().rest_offset, 0.001)

        sapien.physx.set_shape_config(contact_offset=0.01, rest_offset=0)
        self.assertAlmostEqual(sapien.physx.get_shape_config().contact_offset, 0.01)
        self.assertAlmostEqual(sapien.physx.get_shape_config().rest_offset, 0)

        config = sapien.physx.PhysxBodyConfig()
        config.sleep_threshold = 0.001
        config.solver_position_iterations = 25
        config.solver_velocity_iterations = 2
        sapien.physx.set_body_config(config)
        self.assertAlmostEqual(sapien.physx.get_body_config().sleep_threshold, 0.001)
        self.assertEqual(sapien.physx.get_body_config().solver_position_iterations, 25)
        self.assertEqual(sapien.physx.get_body_config().solver_velocity_iterations, 2)

        sapien.physx.set_body_config(
            sleep_threshold=0.005,
            solver_position_iterations=10,
            solver_velocity_iterations=1,
        )
        self.assertAlmostEqual(sapien.physx.get_body_config().sleep_threshold, 0.005)
        self.assertEqual(sapien.physx.get_body_config().solver_position_iterations, 10)
        self.assertEqual(sapien.physx.get_body_config().solver_velocity_iterations, 1)

    def test_cpu_system(self):
        system = sapien.physx.PhysxCpuSystem()
        scene = sapien.Scene([system])
        mat = sapien.physx.PhysxMaterial(0.2, 0.1, 0.05)

        b0 = sapien.physx.PhysxCollisionShapeBox([0.1, 0.2, 0.3], mat)
        c0 = sapien.physx.PhysxRigidDynamicComponent()
        c0.attach(b0)
        e0 = sapien.Entity().add_component(c0)
        scene.add_entity(e0)

        b1 = sapien.physx.PhysxCollisionShapeBox([0.1, 0.2, 0.3], mat)
        c1 = sapien.physx.PhysxRigidDynamicComponent()
        c1.kinematic = True
        c1.attach(b1)
        e1 = sapien.Entity().add_component(c1)
        e1.set_pose(sapien.Pose([0.201, 0, 0]))
        scene.add_entity(e1)

        # test contact
        system.step()
        contacts = system.get_contacts()
        self.assertEqual(len(contacts), 1)
        self.assertEqual(set(contacts[0].bodies), set([c0, c1]))
        self.assertEqual(set(contacts[0].shapes), set([b0, b1]))
        for p in contacts[0].points:
            self.assertAlmostEqual(p.separation, 0.001)

        # test pack unpack
        c1.kinematic_target = sapien.Pose([0, 0, 1])
        p0 = e0.pose
        p1 = e1.pose
        v0 = c0.linear_velocity
        data = system.pack()
        system.step()
        self.assertFalse(pose_equal(e0.pose, p0))
        self.assertFalse(pose_equal(e1.pose, p1))
        self.assertFalse(np.allclose(c0.linear_velocity, v0, atol=1e-5))
        data = system.unpack(data)
        self.assertTrue(pose_equal(e0.pose, p0))
        self.assertTrue(pose_equal(e1.pose, p1))
        self.assertTrue(np.allclose(c0.linear_velocity, v0, atol=1e-5))

    def test_raycast(self):
        system = sapien.physx.PhysxCpuSystem()
        scene = sapien.Scene([system])
        mat = sapien.physx.PhysxMaterial(0.2, 0.1, 0.05)
        b0 = sapien.physx.PhysxCollisionShapePlane(mat)
        c0 = sapien.physx.PhysxRigidStaticComponent()
        b0.set_local_pose(sapien.Pose(p=[0, 0, -1.0], q=[0.7071068, 0, -0.7071068, 0]))
        c0.attach(b0)
        e0 = sapien.Entity().add_component(c0)
        scene.add_entity(e0)

        res = system.raycast([0, 0, 1], [0.70710678, 0.0, -0.70710678], 1000)
        self.assertIsNotNone(res)
        self.assertTrue(np.allclose(res.normal, [0, 0, 1], atol=1e-5))
        self.assertEqual(res.component, c0)
        self.assertTrue(np.allclose(res.distance, 8**0.5))
        self.assertTrue(np.allclose(res.position, [2, 0, -1], atol=1e-5))
