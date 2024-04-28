import unittest
import numpy as np
from common import pose_equal, rand_pose, repeat
from pathlib import Path

import sapien


class TestBodyInertial(unittest.TestCase):
    @repeat(seed=0)
    def test_box(self):
        density = 800
        pose = rand_pose()
        size = [1, 2, 3]
        shape = sapien.physx.PhysxCollisionShapeBox(size, None)
        shape.set_local_pose(pose)
        shape.set_density(density)
        body = sapien.physx.PhysxRigidDynamicComponent()
        body.attach(shape)
        mp = (
            sapien.math.compute_box_mass_properties(size)
            .scale_mass(density)
            .transform(pose)
        )

        R = body.cmass_local_pose.to_transformation_matrix()[:3, :3]
        body_inertia = R @ np.diag(body.inertia) @ R.T

        # computed
        _, pose, inertia = mp.decompose()
        R = pose.to_transformation_matrix()[:3, :3]
        inertia = R @ np.diag(inertia) @ R.T

        self.assertTrue(np.allclose(mp.mass, body.mass))
        self.assertTrue(np.allclose(mp.cm, body.cmass_local_pose.p))
        self.assertTrue(np.allclose(body_inertia, mp.cm_inertia, rtol=1e-3, atol=1e-4))
        self.assertTrue(np.allclose(body_inertia, inertia, rtol=1e-3, atol=1e-4))

    @repeat(seed=1)
    def test_sphere(self):
        density = 700
        pose = rand_pose()
        radius = 0.7
        shape = sapien.physx.PhysxCollisionShapeSphere(radius, None)
        shape.set_local_pose(pose)
        shape.set_density(density)
        body = sapien.physx.PhysxRigidDynamicComponent()
        body.attach(shape)
        mp = (
            sapien.math.compute_sphere_mass_properties(radius)
            .scale_mass(density)
            .transform(pose)
        )

        R = body.cmass_local_pose.to_transformation_matrix()[:3, :3]
        body_inertia = R @ np.diag(body.inertia) @ R.T

        # computed
        _, pose, inertia = mp.decompose()
        R = pose.to_transformation_matrix()[:3, :3]
        inertia = R @ np.diag(inertia) @ R.T

        self.assertTrue(np.allclose(mp.mass, body.mass))
        self.assertTrue(np.allclose(mp.cm, body.cmass_local_pose.p))
        self.assertTrue(np.allclose(body_inertia, mp.cm_inertia, rtol=1e-2, atol=1e-3))
        self.assertTrue(np.allclose(body_inertia, inertia, rtol=1e-2, atol=1e-3))

    @repeat(seed=2)
    def test_capsule(self):
        density = 900
        pose = rand_pose()
        radius = 0.7
        half_length = 0.4
        shape = sapien.physx.PhysxCollisionShapeCapsule(radius, half_length, None)
        shape.set_local_pose(pose)
        shape.set_density(density)
        body = sapien.physx.PhysxRigidDynamicComponent()
        body.attach(shape)
        mp = (
            sapien.math.compute_capsule_mass_properties(radius, half_length)
            .scale_mass(density)
            .transform(pose)
        )

        R = body.cmass_local_pose.to_transformation_matrix()[:3, :3]
        body_inertia = R @ np.diag(body.inertia) @ R.T

        # computed
        _, pose, inertia = mp.decompose()
        R = pose.to_transformation_matrix()[:3, :3]
        inertia = R @ np.diag(inertia) @ R.T

        self.assertTrue(np.allclose(mp.mass, body.mass))
        self.assertTrue(np.allclose(mp.cm, body.cmass_local_pose.p))
        self.assertTrue(np.allclose(body_inertia, mp.cm_inertia, rtol=1e-3, atol=1e-4))
        self.assertTrue(np.allclose(body_inertia, inertia, rtol=1e-3, atol=1e-4))

    @repeat(seed=3)
    def test_cylinder(self):
        density = 900
        pose = rand_pose()
        radius = 0.3
        half_length = 1.1
        shape = sapien.physx.PhysxCollisionShapeCylinder(radius, half_length, None)
        shape.set_local_pose(pose)
        shape.set_density(density)
        body = sapien.physx.PhysxRigidDynamicComponent()
        body.attach(shape)
        mp = (
            sapien.math.compute_cylinder_mass_properties(radius, half_length)
            .scale_mass(density)
            .transform(pose)
        )

        R = body.cmass_local_pose.to_transformation_matrix()[:3, :3]
        body_inertia = R @ np.diag(body.inertia) @ R.T

        # computed
        _, pose, inertia = mp.decompose()
        R = pose.to_transformation_matrix()[:3, :3]
        inertia = R @ np.diag(inertia) @ R.T

        # large tol since cylinder is implemented with mesh
        self.assertTrue(np.allclose(mp.mass, body.mass, rtol=0.05, atol=0.05))
        self.assertTrue(
            np.allclose(mp.cm, body.cmass_local_pose.p, rtol=0.05, atol=0.05)
        )
        self.assertTrue(np.allclose(body_inertia, mp.cm_inertia, rtol=0.05, atol=0.05))
        self.assertTrue(np.allclose(body_inertia, inertia, rtol=0.05, atol=0.05))

    @repeat(seed=4)
    def test_mesh(self):
        density = 1100
        pose = rand_pose()
        scale = [0.5, 0.6, 0.7]
        shape = sapien.physx.PhysxCollisionShapeConvexMesh(
            str(Path(".") / "assets" / "cone.stl"), scale, None
        )
        shape.set_local_pose(pose)
        shape.set_density(density)
        body = sapien.physx.PhysxRigidDynamicComponent()
        body.attach(shape)
        mp = (
            sapien.math.compute_mesh_mass_properties(shape.vertices, shape.triangles)
            .scale_size(scale)
            .scale_mass(density)
            .transform(pose)
        )

        R = body.cmass_local_pose.to_transformation_matrix()[:3, :3]
        body_inertia = R @ np.diag(body.inertia) @ R.T

        # computed
        _, pose, inertia = mp.decompose()
        R = pose.to_transformation_matrix()[:3, :3]
        inertia = R @ np.diag(inertia) @ R.T

        # large tol since cylinder is implemented with mesh
        self.assertTrue(np.allclose(mp.mass, body.mass))
        self.assertTrue(
            np.allclose(mp.cm, body.cmass_local_pose.p, rtol=1e-3, atol=1e-4)
        )
        self.assertTrue(np.allclose(body_inertia, mp.cm_inertia, rtol=1e-3, atol=1e-4))
        self.assertTrue(np.allclose(body_inertia, inertia, rtol=1e-3, atol=1e-4))

    @repeat(seed=5)
    def test_combined(self):
        body = sapien.physx.PhysxRigidDynamicComponent()
        shapes = []

        density = 800
        pose = rand_pose()
        size = [1, 2, 3]
        shape = sapien.physx.PhysxCollisionShapeBox(size, None)
        shapes.append(shape)
        shape.set_local_pose(pose)
        shape.set_density(density)
        body.attach(shape)
        mp0 = (
            sapien.math.compute_box_mass_properties(size)
            .scale_mass(density)
            .transform(pose)
        )

        density = 700
        pose = rand_pose()
        radius = 0.7
        shape = sapien.physx.PhysxCollisionShapeSphere(radius, None)
        shapes.append(shape)
        shape.set_local_pose(pose)
        shape.set_density(density)
        body.attach(shape)
        mp1 = (
            sapien.math.compute_sphere_mass_properties(radius)
            .scale_mass(density)
            .transform(pose)
        )

        density = 900
        pose = rand_pose()
        radius = 0.7
        half_length = 0.4
        shape = sapien.physx.PhysxCollisionShapeCapsule(radius, half_length, None)
        shapes.append(shape)
        shape.set_local_pose(pose)
        shape.set_density(density)
        body.attach(shape)
        mp2 = (
            sapien.math.compute_capsule_mass_properties(radius, half_length)
            .scale_mass(density)
            .transform(pose)
        )

        density = 1100
        pose = rand_pose()
        scale = [0.5, 0.6, 0.7]
        shape = sapien.physx.PhysxCollisionShapeConvexMesh(
            str(Path(".") / "assets" / "cone.stl"), scale, None
        )
        shapes.append(shape)
        shape.set_local_pose(pose)
        shape.set_density(density)
        body.attach(shape)
        mp3 = (
            sapien.math.compute_mesh_mass_properties(shape.vertices, shape.triangles)
            .scale_size(scale)
            .scale_mass(density)
            .transform(pose)
        )

        mp = mp0 + mp1 + mp2 + mp3

        R = body.cmass_local_pose.to_transformation_matrix()[:3, :3]
        body_inertia = R @ np.diag(body.get_inertia()) @ R.T
        _, pose, inertia = mp.decompose()
        R = pose.to_transformation_matrix()[:3, :3]
        inertia = R @ np.diag(inertia) @ R.T
        self.assertTrue(np.allclose(body_inertia, mp.cm_inertia, rtol=1e-2, atol=1e-3))
        self.assertTrue(np.allclose(body_inertia, inertia, rtol=1e-2, atol=1e-3))

        self.assertTrue(np.allclose(mp.mass, body.mass))
        self.assertTrue(np.allclose(mp.cm, body.cmass_local_pose.p))
        self.assertTrue(body.collision_shapes, shapes)
        self.assertTrue(body.get_collision_shapes(), shapes)

        # TODO: test auto compute mass


class TestBody(unittest.TestCase):
    def test_body(self):
        np.random.seed(10)

        body = sapien.physx.PhysxRigidDynamicComponent()
        scene = sapien.Scene([sapien.physx.PhysxCpuSystem()])
        entity = sapien.Entity()
        entity.add_component(body)
        scene.add_entity(entity)

        body.angular_damping = 1
        self.assertAlmostEqual(body.angular_damping, 1)
        body.set_angular_damping(0.5)
        self.assertAlmostEqual(body.get_angular_damping(), 0.5)

        body.linear_damping = 2
        self.assertAlmostEqual(body.linear_damping, 2)
        body.set_linear_damping(0.3)
        self.assertAlmostEqual(body.get_linear_damping(), 0.3)

        body.max_contact_impulse = 10
        self.assertAlmostEqual(body.max_contact_impulse, 10)
        body.set_max_contact_impulse(20)
        self.assertAlmostEqual(body.get_max_contact_impulse(), 20)

        body.max_depenetration_velocity = 15
        self.assertAlmostEqual(body.max_depenetration_velocity, 15)
        body.set_max_depenetration_velocity(25)
        self.assertAlmostEqual(body.get_max_depenetration_velocity(), 25)

        body.disable_gravity = True
        self.assertEqual(body.disable_gravity, True)
        body.set_disable_gravity(False)
        self.assertEqual(body.get_disable_gravity(), False)

        body.linear_velocity = [2, 3, 4]
        self.assertTrue(np.allclose(body.linear_velocity, [2, 3, 4]))
        body.set_linear_velocity([1, 2, 3])
        self.assertTrue(np.allclose(body.get_linear_velocity(), [1, 2, 3]))

        body.angular_velocity = [3, 4, 5]
        self.assertTrue(np.allclose(body.angular_velocity, [3, 4, 5]))
        body.set_angular_velocity([0, 1, 2])
        self.assertTrue(np.allclose(body.get_angular_velocity(), [0, 1, 2]))

        body.mass = 2
        self.assertAlmostEqual(body.mass, 2)
        body.set_mass(0.3)
        self.assertAlmostEqual(body.get_mass(), 0.3)

        body.inertia = [3, 4, 5]
        self.assertTrue(np.allclose(body.inertia, [3, 4, 5]))
        body.set_inertia([1, 1, 2])
        self.assertTrue(np.allclose(body.get_inertia(), [1, 1, 2]))

        pose = rand_pose()
        body.cmass_local_pose = pose
        self.assertTrue(pose_equal(body.cmass_local_pose, pose))
        pose = rand_pose()
        body.set_cmass_local_pose(pose)
        self.assertTrue(pose_equal(body.get_cmass_local_pose(), pose))

        body.kinematic = True
        self.assertEqual(body.kinematic, True)
        pose = rand_pose()
        body.kinematic_target = pose
        self.assertTrue(pose_equal(body.kinematic_target, pose, rtol=1e-4, atol=1e-5))
        pose = rand_pose()
        body.set_kinematic_target(pose)
        self.assertTrue(
            pose_equal(body.get_kinematic_target(), pose, rtol=1e-4, atol=1e-5)
        )
        body.set_kinematic(False)
        self.assertEqual(body.get_kinematic(), False)

        body.wake_up()
        self.assertFalse(body.is_sleeping)
        body.put_to_sleep()
        self.assertTrue(body.is_sleeping)
        body.wake_up()
        self.assertFalse(body.is_sleeping)

        body.set_locked_motion_axes([True, True, False, False, True, True])
        self.assertEqual(
            body.get_locked_motion_axes(), [True, True, False, False, True, True]
        )
        self.assertEqual(
            body.locked_motion_axes, [True, True, False, False, True, True]
        )

        body.solver_position_iterations = 10
        self.assertAlmostEqual(body.solver_position_iterations, 10)
        body.set_solver_position_iterations(20)
        self.assertAlmostEqual(body.get_solver_position_iterations(), 20)

        body.solver_velocity_iterations = 10
        self.assertAlmostEqual(body.solver_velocity_iterations, 10)
        body.set_solver_velocity_iterations(20)
        self.assertAlmostEqual(body.get_solver_velocity_iterations(), 20)

        body.sleep_threshold = 0.003
        self.assertAlmostEqual(body.sleep_threshold, 0.003)
        body.set_sleep_threshold(0.03)
        self.assertAlmostEqual(body.get_sleep_threshold(), 0.03)

    def test_default(self):
        sapien.physx.set_body_config(
            sleep_threshold=0.0001,
            solver_position_iterations=50,
            solver_velocity_iterations=5,
        )
        body = sapien.physx.PhysxRigidDynamicComponent()
        self.assertEqual(body.solver_position_iterations, 50)
        self.assertEqual(body.solver_velocity_iterations, 5)
        self.assertAlmostEqual(body.sleep_threshold, 0.0001)

        link = sapien.physx.PhysxArticulationLinkComponent()
        self.assertEqual(link.articulation.solver_position_iterations, 50)
        self.assertEqual(link.articulation.solver_velocity_iterations, 5)
        self.assertAlmostEqual(link.articulation.sleep_threshold, 0.0001)

    def test_drive(self):
        body0 = sapien.physx.PhysxRigidDynamicComponent()
        body1 = sapien.physx.PhysxRigidDynamicComponent()
        drive = sapien.physx.PhysxDriveComponent(body1)

        entity = sapien.Entity()
        entity.add_component(body0)
        entity.add_component(drive)

        pose = rand_pose()
        drive.drive_target = pose
        self.assertTrue(pose_equal(drive.drive_target, pose))
        pose = rand_pose()
        drive.set_drive_target(pose)
        self.assertTrue(pose_equal(drive.get_drive_target(), pose))

        drive.set_drive_velocity_target([0, 1, 2], [1, 2, 3])
        l, a = drive.get_drive_velocity_target()
        self.assertTrue(np.allclose(l, [0, 1, 2]))
        self.assertTrue(np.allclose(a, [1, 2, 3]))

        props = (
            float(np.random.rand()),
            float(np.random.rand()),
            float(np.random.rand()),
            np.random.choice(["force", "acceleration"]),
        )
        drive.set_drive_property_x(*props)
        for a, b in zip(drive.get_drive_property_x(), props):
            self.assertAlmostEqual(a, b)

        props = (
            float(np.random.rand()),
            float(np.random.rand()),
            float(np.random.rand()),
            np.random.choice(["force", "acceleration"]),
        )
        drive.set_drive_property_y(*props)
        for a, b in zip(drive.get_drive_property_y(), props):
            self.assertAlmostEqual(a, b)

        props = (
            float(np.random.rand()),
            float(np.random.rand()),
            float(np.random.rand()),
            np.random.choice(["force", "acceleration"]),
        )
        drive.set_drive_property_z(*props)
        for a, b in zip(drive.get_drive_property_z(), props):
            self.assertAlmostEqual(a, b)

        props = (
            float(np.random.rand()),
            float(np.random.rand()),
            float(np.random.rand()),
            np.random.choice(["force", "acceleration"]),
        )
        drive.set_drive_property_swing(*props)
        for a, b in zip(drive.get_drive_property_swing(), props):
            self.assertAlmostEqual(a, b)

        props = (
            float(np.random.rand()),
            float(np.random.rand()),
            float(np.random.rand()),
            np.random.choice(["force", "acceleration"]),
        )
        drive.set_drive_property_twist(*props)
        for a, b in zip(drive.get_drive_property_twist(), props):
            self.assertAlmostEqual(a, b)

        props = (
            float(np.random.rand()),
            float(np.random.rand()),
            float(np.random.rand()),
            np.random.choice(["force", "acceleration"]),
        )
        drive.set_drive_property_slerp(*props)
        for a, b in zip(drive.get_drive_property_slerp(), props):
            self.assertAlmostEqual(a, b)

        props = (
            -np.random.rand(),
            np.random.rand(),
            np.random.rand(),
            np.random.rand(),
        )
        drive.set_limit_x(*props)
        for a, b in zip(drive.get_limit_x(), props):
            self.assertAlmostEqual(a, b)

        props = (
            -np.random.rand(),
            np.random.rand(),
            np.random.rand(),
            np.random.rand(),
        )
        drive.set_limit_y(*props)
        for a, b in zip(drive.get_limit_y(), props):
            self.assertAlmostEqual(a, b)

        props = (
            -np.random.rand(),
            np.random.rand(),
            np.random.rand(),
            np.random.rand(),
        )
        drive.set_limit_z(*props)
        for a, b in zip(drive.get_limit_z(), props):
            self.assertAlmostEqual(a, b)

        props = (
            np.random.rand(),
            np.random.rand(),
            np.random.rand(),
            np.random.rand(),
        )
        drive.set_limit_cone(*props)
        for a, b in zip(drive.get_limit_cone(), props):
            self.assertAlmostEqual(a, b)

        props = (
            -np.random.rand(),
            np.random.rand(),
            -np.random.rand(),
            np.random.rand(),
            np.random.rand(),
            np.random.rand(),
        )
        drive.set_limit_pyramid(*props)
        for a, b in zip(drive.get_limit_pyramid(), props):
            self.assertAlmostEqual(a, b)

        props = (
            -np.random.rand(),
            np.random.rand(),
            np.random.rand(),
            np.random.rand(),
        )
        drive.set_limit_twist(*props)
        for a, b in zip(drive.get_limit_twist(), props):
            self.assertAlmostEqual(a, b)

    # TODO: test gear
    # TODO: finish and test tendon
