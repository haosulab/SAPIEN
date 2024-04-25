import unittest

import numpy as np
import sapien
from common import pose_equal, rand_pose, rand_size


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

    def test_add_system(self):
        scene = sapien.Scene([])
        system = sapien.physx.PhysxCpuSystem()
        scene.add_system(system)
        self.assertEqual(scene.get_system("physx"), system)

    def test_clear(self):
        scene = sapien.Scene()
        scene.add_entity(sapien.Entity())
        scene.add_entity(sapien.Entity())
        scene.clear()
        self.assertEqual(scene.entities, [])

    def test_pack_unpack(self):
        p0 = rand_pose()
        p1 = rand_pose()
        e0 = sapien.Entity()
        e1 = sapien.Entity()

        e0.set_pose(p0)
        e1.set_pose(p1)

        scene = sapien.Scene()
        scene.add_entity(e0)
        scene.add_entity(e1)

        data = scene.pack_poses()
        e0.set_pose(sapien.Pose())
        e1.set_pose(sapien.Pose())
        scene.unpack_poses(data)

        self.assertTrue(pose_equal(e0.pose, p0))
        self.assertTrue(pose_equal(e1.pose, p1))

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
