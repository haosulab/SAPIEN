"""CPU by default; set SAPIEN_TEST_GPU_RESET=1 for the opt-in GPU regression."""
import os
import unittest

import numpy as np
import sapien

GPU = os.environ.get("SAPIEN_TEST_GPU_RESET") == "1"
if GPU:
    import torch
    sapien.physx.enable_gpu()
physx = sapien.physx


class TestResetFiltering(unittest.TestCase):
    def setUp(self):
        physx.set_scene_config(gravity=[0, 0, -9.81], enable_tgs=os.environ.get("SAPIEN_TEST_PGS") != "1")
        physx.set_body_config(solver_position_iterations=16, solver_velocity_iterations=4, sleep_threshold=0)
        physx.set_shape_config(contact_offset=0.002, rest_offset=0)
        self.px = physx.PhysxGpuSystem() if GPU else physx.PhysxCpuSystem()
        self.px.timestep = 0.005
        self.scenes = []
        material = physx.PhysxMaterial(1, 1, 0)
        self.bodies, self.robots = [], []
        for i in range(4):
            scene = sapien.Scene(systems=[self.px])
            self.scenes.append(scene)
            if GPU:
                self.px.set_scene_offset(scene, [10*i, -3*i, 0])
            ground = scene.create_actor_builder()
            ground.add_box_collision(half_size=[0.8, 0.8, 0.1], material=material)
            ground.set_initial_pose(sapien.Pose([0.2 + 2*i, -0.3, -0.1]))
            ground.build_static()
            builder = scene.create_actor_builder() if i < 2 else scene.create_articulation_builder()
            link = builder if i < 2 else builder.create_link_builder()
            link.add_box_collision(half_size=[0.05, 0.04, 0.03], material=material)
            link.set_mass_and_inertia(1, sapien.Pose(), [0.0008333333, 0.0011333333, 0.0013666667])
            builder.set_initial_pose(sapien.Pose([0.2 + 2 * i, -0.3, 0.03]))
            if i < 2:
                body = builder.build().find_component_by_type(physx.PhysxRigidDynamicComponent)
            else:
                robot = builder.build(fix_root_link=False)
                self.robots.append(robot)
                body = robot.root
            self.bodies.append(body)
        self.target = np.array([[0.21 + 2*i, -0.3, 0.03, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                for i in range(4)], dtype=np.float32)
        if GPU:
            self.px.gpu_init()

    def write(self, values):
        if GPU:
            state = self.px.cuda_rigid_body_data.torch()
            for body, value in zip(self.bodies, values):
                state[body.gpu_pose_index] = torch.as_tensor(value, device=state.device)
            self.px.gpu_apply_rigid_dynamic_data()
            self.px.gpu_apply_articulation_root_data()
            self.px.gpu_update_articulation_kinematics()
        else:
            for body, value in zip(self.bodies[:2], values[:2]):
                body.entity.pose = sapien.Pose(value[:3], value[3:7])
                body.linear_velocity = value[7:10]
                body.angular_velocity = value[10:13]
            for robot, value in zip(self.robots, values[2:]):
                robot.root_pose = sapien.Pose(value[:3], value[3:7])
                robot.root_linear_velocity = value[7:10]
                robot.root_angular_velocity = value[10:13]

    def read(self):
        if GPU:
            self.px.gpu_fetch_rigid_dynamic_data()
            self.px.gpu_fetch_articulation_link_data()
            state = self.px.cuda_rigid_body_data.torch()
            return state[[b.gpu_pose_index for b in self.bodies]].cpu().numpy().copy()
        return np.array([np.concatenate((b.entity.pose.p, b.entity.pose.q, b.linear_velocity, b.angular_velocity))
                         for b in self.bodies])

    def test_selected_history_and_repeated_reset(self):
        self.check_history_reset(batched=False)

    @unittest.skipUnless(GPU, "GPU indices require a GPU system")
    def test_batched_index_history_reset(self):
        self.check_history_reset(batched=True)

    def check_history_reset(self, batched):
        for _ in range(200):
            self.px.step()
        # Measure the same solver's un-reset response; PGS and TGS have different
        # kick magnitudes. A fixed TGS threshold is not a valid PGS control.
        self.write(self.target)
        self.px.step()
        warm_velocity = self.read()[:, 7].copy()
        initial = self.target.copy()
        initial[:, 0] -= 0.01
        self.write(initial)
        self.px.reset_filtering(self.bodies)
        for _ in range(200):
            self.px.step()
        if GPU:
            query = self.px.gpu_create_contact_body_impulse_query(self.bodies)
            self.px.gpu_query_contact_body_impulses(query)
        self.write(self.target)
        np.testing.assert_allclose(self.read(), self.target, atol=1e-6, rtol=0)
        # Select one rigid actor and one articulation link; keep the two controls warm.
        if batched:
            body_indices = torch.tensor([self.bodies[0].gpu_index], device="cuda", dtype=torch.int32)
            art_indices = torch.tensor([self.robots[0].gpu_data_index], device="cuda", dtype=torch.int64)
            self.px.gpu_reset_filtering(sapien.CudaArray(body_indices), sapien.CudaArray(art_indices))
        else:
            self.px.reset_filtering_scene(self.scenes[0])
            self.px.reset_filtering([self.bodies[2]])
        if GPU:
            with self.assertRaisesRegex(RuntimeError, "completed step"):
                self.px.gpu_query_contact_body_impulses(query)
        self.px.step()
        velocity = self.read()[:, 7]
        print("first-step horizontal velocities:", velocity, flush=True)
        np.testing.assert_allclose(velocity[[0, 2]], 0, atol=1e-3)
        if GPU:
            self.assertTrue(np.all(warm_velocity[[1, 3]] < -1e-3), warm_velocity)
            np.testing.assert_allclose(velocity[[1, 3]], warm_velocity[[1, 3]], atol=2e-4, rtol=0)
            self.px.gpu_query_contact_body_impulses(query)
            self.assertTrue(torch.all(query.cuda_impulses.torch()[:, 2] > 0.04).item())
        else:
            np.testing.assert_allclose(velocity, 0, atol=1e-3)

        for cycle in range(8):
            self.target[:, 0] += 0.01 if cycle % 2 == 0 else -0.01
            self.write(self.target)
            if batched:
                self.px.gpu_reset_filtering([b.gpu_index for b in self.bodies[:2]],
                                             [a.gpu_data_index for a in self.robots])
            else:
                self.px.reset_filtering(self.bodies)
            self.px.step()
            np.testing.assert_allclose(self.read()[:, 7], 0, atol=1e-3)
        # Re-insertion must use the live GPU pose, not the stale original CPU pose.
        self.target[:, 0] += 7
        self.target[:, 2] = 2
        self.write(self.target)
        self.px.reset_filtering(self.bodies)
        self.px.step()
        actual = self.read()
        np.testing.assert_allclose(actual[:, 0], self.target[:, 0], atol=2e-6)
        self.assertTrue(np.all(actual[:, 2] > 1.99), actual)
        if GPU:
            self.px.gpu_query_contact_body_impulses(query)
            self.assertEqual(torch.count_nonzero(query.cuda_impulses.torch()).item(), 0)

    def test_validation_and_empty_selection(self):
        before = self.read()
        for bodies in ([None], [self.bodies[0], self.bodies[0]]):
            with self.assertRaises(ValueError):
                self.px.reset_filtering(bodies)
        other_px = physx.PhysxGpuSystem() if GPU else physx.PhysxCpuSystem()
        other_scene = sapien.Scene(systems=[other_px])
        builder = other_scene.create_actor_builder()
        foreign = builder.build_static().find_component_by_type(physx.PhysxRigidStaticComponent)
        with self.assertRaises(ValueError):
            self.px.reset_filtering([self.bodies[0], foreign])
        with self.assertRaises(ValueError):
            self.px.reset_filtering_scene(other_scene)
        self.px.reset_filtering([])
        np.testing.assert_allclose(self.read(), before, atol=1e-6, rtol=0)


if __name__ == "__main__":
    unittest.main()
