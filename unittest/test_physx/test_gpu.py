import os
import unittest
from pathlib import Path

import numpy as np
import sapien
from common import pose_equal, rand_pose

sapien.physx.enable_gpu()

class TestGpu(unittest.TestCase):
    def test_readwrite(self):
        np.random.seed(0)
        import torch

        scene_count = 16

        px = sapien.physx.PhysxGpuSystem()
        scenes = []
        for i in range(scene_count):
            scene = sapien.Scene([px, sapien.render.RenderSystem()])
            px.set_scene_offset(scene, [i * 10.0, 0.0, 0.0])
            scenes.append(scene)
        urdf_loader = scenes[0].create_urdf_loader()
        builder = urdf_loader.load_file_as_articulation_builder(
            str(Path(".") / "assets" / "movo_simple.urdf")
        )
        robots = []
        for i, scene in enumerate(scenes):
            builder.set_scene(scene)
            robot = builder.build()
            robots.append(robot)

        # balls
        builder = scenes[0].create_actor_builder()
        builder.add_sphere_collision(radius=0.1)
        builder.add_sphere_visual(radius=0.1)
        balls = []
        z = 5
        for scene in scenes:
            builder.set_scene(scene)
            builder.set_initial_pose(sapien.Pose([0, 0, z]))
            ball = builder.build()
            balls.append(ball)
            z += 1
        all_bodies = [
            b.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
            for b in balls
        ]

        px.gpu_init()
        rigid_body_data = px.cuda_rigid_body_data.torch()
        rigid_body_force = px.cuda_rigid_body_force.torch()
        rigid_body_torque = px.cuda_rigid_body_torque.torch()

        articulation_link_data = px.cuda_articulation_link_data.torch()
        articulation_link_force = px.cuda_articulation_link_force.torch()
        articulation_link_torque = px.cuda_articulation_link_torque.torch()

        rigid_dynamic_data = px.cuda_rigid_dynamic_data.torch()
        rigid_dynamic_force = px.cuda_rigid_dynamic_force.torch()
        rigid_dynamic_torque = px.cuda_rigid_dynamic_torque.torch()

        articulation_link_incoming_joint_forces = (
            px.cuda_articulation_link_incoming_joint_forces.torch()
        )
        articulation_qpos = px.cuda_articulation_qpos.torch()
        articulation_qvel = px.cuda_articulation_qvel.torch()
        articulation_qacc = px.cuda_articulation_qacc.torch()
        articulation_qf = px.cuda_articulation_qf.torch()
        articulation_target_qpos = px.cuda_articulation_target_qpos.torch()
        articulation_target_qvel = px.cuda_articulation_target_qvel.torch()

        self.assertEqual(articulation_link_data.shape, (scene_count, 30, 13))
        self.assertEqual(articulation_qpos.shape, (scene_count, 13))
        self.assertEqual(articulation_qvel.shape, (scene_count, 13))
        self.assertEqual(articulation_qf.shape, (scene_count, 13))
        self.assertEqual(articulation_qacc.shape, (scene_count, 13))
        self.assertEqual(
            articulation_link_incoming_joint_forces.shape, (scene_count, 30, 6)
        )
        self.assertEqual(articulation_link_force.shape, (scene_count, 30, 3))
        self.assertEqual(articulation_link_torque.shape, (scene_count, 30, 3))
        self.assertEqual(articulation_target_qpos.shape, (scene_count, 13))
        self.assertEqual(articulation_target_qvel.shape, (scene_count, 13))

        self.assertEqual(rigid_body_data.shape, (31 * scene_count, 13))
        self.assertEqual(rigid_body_force.shape, (31 * scene_count, 3))
        self.assertEqual(rigid_body_torque.shape, (31 * scene_count, 3))
        self.assertEqual(rigid_dynamic_data.shape, (scene_count, 13))
        self.assertEqual(rigid_dynamic_force.shape, (scene_count, 3))
        self.assertEqual(rigid_dynamic_torque.shape, (scene_count, 3))

        # qpos write/read
        rand_qpos = np.random.rand(scene_count, 13) * 0.1
        articulation_qpos[:] = torch.tensor(rand_qpos)
        a = articulation_qpos.cpu().numpy()
        px.gpu_fetch_articulation_qpos()
        b = articulation_qpos.cpu().numpy()
        self.assertFalse(np.allclose(a, b))
        articulation_qpos[:] = torch.tensor(rand_qpos)
        px.gpu_apply_articulation_qpos()
        articulation_qpos.fill_(0)
        px.gpu_fetch_articulation_qpos()
        c = articulation_qpos.cpu().numpy()
        self.assertTrue(np.allclose(a, c))

        # qvel write/read
        rand_qvel = np.random.rand(scene_count, 13) - 0.5
        articulation_qvel[:] = torch.tensor(rand_qvel)
        a = articulation_qvel.cpu().numpy()
        px.gpu_fetch_articulation_qvel()
        b = articulation_qvel.cpu().numpy()
        self.assertFalse(np.allclose(a, b))
        articulation_qvel[:] = torch.tensor(rand_qvel)
        px.gpu_apply_articulation_qvel()
        articulation_qvel.fill_(0)
        px.gpu_fetch_articulation_qvel()
        c = articulation_qvel.cpu().numpy()
        self.assertTrue(np.allclose(a, c))

        # root data write/read
        rand_data = np.random.rand(scene_count, 13) - 0.5
        rand_data[:, 3:7] /= np.linalg.norm(rand_data[:, 3:7], axis=1, keepdims=True)
        rand_data = rand_data.astype(np.float32)
        articulation_root_data = articulation_link_data[:, 0, :]
        articulation_root_data[:] = torch.tensor(rand_data)
        px.gpu_apply_articulation_root_data()
        px.gpu_update_articulation_kinematics()
        articulation_root_data[:].fill_(0)
        self.assertFalse(
            np.allclose(
                rand_data, articulation_root_data.cpu().numpy(), rtol=1e-2, atol=1e-2
            )
        )
        px.gpu_fetch_articulation_link_data()
        self.assertTrue(
            np.allclose(
                rand_data, articulation_root_data.cpu().numpy(), rtol=1e-4, atol=1e-4
            )
        )
        # TODO read link data, update kinematics, qacc, qf, joint forces, link force/torque, target qpos/qvel

        # body data write/read
        rand_data = np.random.rand(scene_count, 13) - 0.5
        rand_data[:, 3:7] /= np.linalg.norm(rand_data[:, 3:7], axis=1, keepdims=True)
        rand_data = rand_data.astype(np.float32)
        rigid_dynamic_data[:] = torch.tensor(rand_data)
        px.gpu_apply_rigid_dynamic_data()
        rigid_dynamic_data[:].fill_(0)
        self.assertFalse(
            np.allclose(
                rand_data, rigid_dynamic_data.cpu().numpy(), rtol=1e-2, atol=1e-2
            )
        )
        px.gpu_fetch_rigid_dynamic_data()
        self.assertTrue(
            np.allclose(
                rand_data, rigid_dynamic_data.cpu().numpy(), rtol=1e-4, atol=1e-4
            )
        )
        # TODO force/torque
