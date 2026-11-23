"""Opt-in: SAPIEN_TEST_GPU_RESET=1 python -m unittest discover -s unittest/test_physx -p test_gpu_indexed.py."""
import os
import unittest

import numpy as np
import sapien
import torch

ENABLED = os.environ.get("SAPIEN_TEST_GPU_RESET") == "1"
if ENABLED:
    sapien.physx.enable_gpu()
pxmod = sapien.physx


@unittest.skipUnless(ENABLED, "explicit GPU reset validation only")
class TestGpuIndexed(unittest.TestCase):
    def setUp(self):
        pxmod.set_scene_config(gravity=[0, 0, 0], enable_tgs=os.environ.get("SAPIEN_TEST_PGS") != "1")
        self.px = pxmod.PhysxGpuSystem()
        self.px.timestep = 0.005
        self.scenes, self.bodies, self.robots = [], [], []
        for i in range(4):
            scene = sapien.Scene(systems=[self.px])
            self.px.set_scene_offset(scene, [11 * i, -7 * i, 3 * i])
            self.scenes.append(scene)
            b = scene.create_actor_builder()
            b.set_mass_and_inertia(1, sapien.Pose(), [0.03, 0.04, 0.05])
            b.set_initial_pose(sapien.Pose([0.2, 0.3, 2 + i]))
            self.bodies.append(b.build().find_component_by_type(pxmod.PhysxRigidDynamicComponent))
            a = scene.create_articulation_builder()
            parent = None
            for j in range(1 + i % 3):
                link = a.create_link_builder(parent)
                link.set_mass_and_inertia(1, sapien.Pose(), [0.03, 0.04, 0.05])
                if parent:
                    link.set_joint_properties("revolute", [[-2, 2]],
                                              sapien.Pose([0, 0, 0.2]), sapien.Pose())
                parent = link
            a.set_initial_pose(sapien.Pose([0.4, 0.5, 3 + i]))
            self.robots.append(a.build(fix_root_link=False))
        self.px.gpu_init()
        self.rows = [self.robots[i].gpu_data_index for i in (2, 0)]
        self.body_rows = [self.bodies[i].gpu_index for i in (2, 0)]

    def test_state_selection_offsets_and_nondefault_stream(self):
        px = self.px
        stream = torch.cuda.Stream()
        px.gpu_set_cuda_stream(stream.cuda_stream)
        with torch.cuda.stream(stream):
            px.gpu_fetch_rigid_dynamic_data()
            state = px.cuda_rigid_dynamic_data.torch()
            before = state.clone()
            target = before.clone()
            target[:, :3] += torch.tensor([0.31, -0.17, 0.29], device=state.device)
            q = torch.tensor([0.8, 0.1, 0.3, 0.4], device=state.device)
            target[:, 3:7] = q / q.norm()
            target[:, 7:13] = torch.tensor([0.11, -0.12, 0.13, 0.21, -0.22, 0.23], device=state.device)
            state.copy_(target)  # Unselected staging rows deliberately contain different values.
            px.gpu_apply_rigid_dynamic_data(self.body_rows)
            px.gpu_fetch_rigid_dynamic_data()
            expected = before.clone()
            expected[self.body_rows] = target[self.body_rows]
            torch.testing.assert_close(state, expected, atol=2e-6, rtol=0)
            state.fill_(42)
            px.gpu_apply_rigid_dynamic_data([])
            px.gpu_fetch_rigid_dynamic_data()
            torch.testing.assert_close(state, expected, atol=2e-6, rtol=0)

            px.gpu_fetch_articulation_link_data()
            roots = px.cuda_articulation_link_data.torch()[:, 0]
            before = roots.clone()
            target = before.clone()
            target[:, :3] += torch.tensor([-0.27, 0.19, 0.33], device=roots.device)
            target[:, 3:7] = q / q.norm()
            target[:, 7:13] = torch.tensor([-0.01, 0.02, -0.03, 0.04, -0.05, 0.06], device=roots.device)
            roots.copy_(target)
            px.gpu_apply_articulation_root_data(self.rows)
            px.gpu_update_articulation_kinematics(self.rows)
            px.gpu_fetch_articulation_link_data()
            expected = before.clone()
            expected[self.rows] = target[self.rows]
            torch.testing.assert_close(roots, expected, atol=3e-6, rtol=0)
        stream.synchronize()
        px.gpu_set_cuda_stream(0)

    def test_joint_rows_and_targets(self):
        px = self.px
        for field, fetch, apply in (
            (px.cuda_articulation_qpos, px.gpu_fetch_articulation_qpos, px.gpu_apply_articulation_qpos),
            (px.cuda_articulation_qvel, px.gpu_fetch_articulation_qvel, px.gpu_apply_articulation_qvel),
            (px.cuda_articulation_target_qpos, px.gpu_fetch_articulation_target_qpos,
             px.gpu_apply_articulation_target_position),
            (px.cuda_articulation_target_qvel, px.gpu_fetch_articulation_target_qvel,
             px.gpu_apply_articulation_target_velocity),
        ):
            with self.subTest(apply=apply.__name__):
                fetch()
                data = field.torch()
                before = data.clone()
                target = torch.arange(data.numel(), device=data.device).reshape(data.shape) * 0.01 + 0.1
                data.copy_(target)
                apply(self.rows)
                fetch()
                # PhysX leaves padded zero-DOF columns unspecified. Check actual joints.
                for robot in self.robots:
                    row, dof = robot.gpu_data_index, robot.dof
                    expected = target if row in self.rows else before
                    torch.testing.assert_close(data[row, :dof], expected[row, :dof], atol=1e-6, rtol=0)

        # Joint-force selection is verified by resulting velocity, with no drives or gravity.
        qf = px.cuda_articulation_qf.torch()
        qf.fill_(1)
        zero = px.cuda_articulation_qvel.torch()
        zero.zero_()
        px.gpu_apply_articulation_qvel()
        px.gpu_apply_articulation_qf([self.robots[2].gpu_data_index])
        px.step()
        px.gpu_fetch_articulation_qvel()
        self.assertGreater(zero[self.robots[2].gpu_data_index, :2].abs().max().item(), 0.01)
        self.assertLess(zero[self.robots[1].gpu_data_index, 0].abs().item(), 1e-6)

    def test_forces_and_torques(self):
        px = self.px
        # Free unit-mass rigid bodies: dv = F*dt/m and dw = torque*dt/inertia.
        px.cuda_rigid_dynamic_force.torch()[:] = torch.tensor([2, 3, 4], device="cuda")
        px.cuda_rigid_dynamic_torque.torch()[:] = torch.tensor([0.03, 0.08, 0.15], device="cuda")
        px.gpu_apply_rigid_dynamic_force(self.body_rows)
        px.gpu_apply_rigid_dynamic_torque(self.body_rows)
        # Articulations 0 and 3 have a single free link and zero joints.
        row = self.robots[0].gpu_data_index
        force, torque = px.cuda_articulation_link_force.torch(), px.cuda_articulation_link_torque.torch()
        force[:] = torch.tensor([2, 3, 4], device=force.device)
        torque[:] = torch.tensor([0.03, 0.08, 0.15], device=force.device)
        px.gpu_apply_articulation_link_force([row])
        px.gpu_apply_articulation_link_torque([row])
        px.step()
        px.gpu_fetch_rigid_dynamic_data()
        px.gpu_fetch_articulation_link_data()
        expected = np.array([2, 3, 4, 1, 2, 3]) * px.timestep
        for body in self.bodies:
            actual = px.cuda_rigid_dynamic_data.torch()[body.gpu_index, 7:13].cpu().numpy()
            np.testing.assert_allclose(actual, expected if body.gpu_index in self.body_rows else 0, atol=2e-5)
        roots = px.cuda_articulation_link_data.torch()[:, 0, 7:13].cpu().numpy()
        np.testing.assert_allclose(roots[row], expected, atol=2e-5)
        np.testing.assert_allclose(roots[self.robots[3].gpu_data_index], 0, atol=1e-6)

    def test_reordered_articulation_ids(self):
        px = self.px
        # Reuse PhysX slots while SAPIEN keeps component creation order.
        entities = [[link.entity for link in robot.links] for robot in self.robots[:2]]
        for scene, group in zip(self.scenes, entities):
            for entity in group:
                scene.remove_entity(entity)
        for scene, group in zip(self.scenes, entities):
            for entity in group:
                scene.add_entity(entity)
        px.gpu_init()
        raw_ids = [a.gpu_index for a in self.robots]
        rows = [a.gpu_data_index for a in self.robots]
        self.assertNotEqual(raw_ids, rows)
        px.gpu_fetch_articulation_link_data()
        roots = px.cuda_articulation_link_data.torch()[:, 0]
        before = roots.clone()
        roots[:, 0] += 0.7
        px.gpu_apply_articulation_root_data([rows[0]])
        px.gpu_update_articulation_kinematics([rows[0]])
        px.gpu_fetch_articulation_link_data()
        before[rows[0], 0] += 0.7
        torch.testing.assert_close(roots, before, atol=2e-6, rtol=0)
        px.gpu_reset_filtering(articulation_indices=[rows[0]])

    def test_empty_joint_rows(self):
        px = pxmod.PhysxGpuSystem()
        scene = sapien.Scene(systems=[px])
        builder = scene.create_articulation_builder()
        builder.create_link_builder().set_mass_and_inertia(1, sapien.Pose(), [0.03, 0.04, 0.05])
        robot = builder.build(fix_root_link=False)
        px.gpu_init()
        self.assertEqual(px.cuda_articulation_qpos.shape, [1, 0])
        for apply in (px.gpu_apply_articulation_qpos, px.gpu_apply_articulation_qvel,
                      px.gpu_apply_articulation_qf, px.gpu_apply_articulation_target_position,
                      px.gpu_apply_articulation_target_velocity):
            apply([robot.gpu_data_index])
        px.gpu_apply_rigid_dynamic_data([])
        px.step()

    def test_validation_and_pending_step(self):
        px = self.px
        for indices in ([-1], [len(self.bodies)], [0, 0]):
            with self.assertRaises((IndexError, ValueError)):
                px.gpu_apply_rigid_dynamic_data(indices)
        with self.assertRaises((IndexError, ValueError)):
            px.gpu_apply_articulation_qpos([len(self.robots)])
        for bodies, arts in (([-1], []), ([0, 0], []), ([0], [len(self.robots)])):
            with self.assertRaises((IndexError, ValueError)):
                px.gpu_reset_filtering(bodies, arts)
        for values, dtype in (([0.5], torch.float32), ([2**40], torch.int64), ([-1], torch.int32)):
            indices = torch.tensor(values, dtype=dtype, device="cuda")
            with self.assertRaises((IndexError, ValueError)):
                px.gpu_reset_filtering(sapien.CudaArray(indices))
        indices = torch.tensor([0, 1, 2, 3], device="cuda")
        with self.assertRaises(RuntimeError):
            px.gpu_reset_filtering(sapien.CudaArray(indices[::2]))
        stream = torch.cuda.Stream()
        px.gpu_set_cuda_stream(stream.cuda_stream)
        with torch.cuda.stream(stream):
            selected = torch.tensor([self.bodies[0].gpu_index], dtype=torch.int64, device="cuda")
            px.gpu_reset_filtering(sapien.CudaArray(selected), [self.robots[2].gpu_data_index])
        px.gpu_set_cuda_stream(0)
        px.gpu_reset_filtering()
        px.step_start()
        try:
            with self.assertRaises(RuntimeError):
                px.gpu_apply_rigid_dynamic_data([0])
            with self.assertRaises(RuntimeError):
                px.reset_filtering([self.bodies[0]])
            with self.assertRaises(RuntimeError):
                px.gpu_reset_filtering([0])
            with self.assertRaises(RuntimeError):
                px.step_start()
        finally:
            px.step_finish()
        with self.assertRaises(RuntimeError):
            px.step_finish()


if __name__ == "__main__":
    unittest.main()
