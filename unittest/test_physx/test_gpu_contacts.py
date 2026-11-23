"""Regressions for compound-body queries and the last contact disappearing.

Run on a CUDA host with a matching PhysX GPU library.
"""

import unittest

import sapien
import torch

sapien.physx.enable_gpu()


class TestGpuContacts(unittest.TestCase):
    def test_compound_contacts_and_separated_row(self):
        px = sapien.physx.PhysxGpuSystem()
        scene = sapien.Scene(systems=[px])
        px.timestep = 0.01
        ground = scene.add_ground(0.0, render=False)
        floor = ground.find_component_by_type(sapien.physx.PhysxRigidStaticComponent)
        bodies = []
        # Opposite hull insertion orders expose competing writes to one row.
        # The elevated hull is in the contact offset but carries no load.
        for x, offsets in [(-0.5, (0.0, 0.008)), (0.5, (0.008, 0.0)), (1.5, (0.0,))]:
            builder = scene.create_actor_builder()
            for dz in offsets:
                builder.add_box_collision(
                    pose=sapien.Pose(p=[-0.06 if dz == 0 else 0.06, 0, dz]),
                    half_size=(0.05, 0.04, 0.03),
                )
            builder.set_initial_pose(sapien.Pose(p=[x, 0, 0.03]))
            body = builder.build().find_component_by_type(
                sapien.physx.PhysxRigidDynamicComponent
            )
            body.mass = 1.0
            body.set_locked_motion_axes([False, False, False, True, True, True])
            for shape in body.get_collision_shapes():
                shape.contact_offset = 0.01
            bodies.append(body)
        for shape in floor.get_collision_shapes():
            shape.contact_offset = 0.01
        px.gpu_init()
        pairs = [(body, floor) for body in bodies]
        points = px.gpu_create_contact_pair_point_query(pairs)
        net = px.gpu_create_contact_pair_impulse_query(pairs)

        def query():
            px.gpu_query_contact_pair_points(points)
            px.gpu_query_contact_pair_impulses(net)
            torch.cuda.synchronize()
            return points.points.torch(), points.impulses.torch(), net.cuda_impulses.torch()

        for _ in range(30):
            px.step()
            positions, impulses, totals = query()
            self.assertTrue(torch.all(totals.norm(dim=-1) > 1e-6).item())
            self.assertTrue(torch.all(impulses.norm(dim=-1) > 0).item())
            self.assertTrue(torch.isfinite(positions).all().item())
        self.assertGreater(positions[2].norm().item(), 0)

        # Keep other contacts alive while removing the single-hull control.
        px.gpu_fetch_rigid_dynamic_data()
        state = px.cuda_rigid_body_data.torch()
        state[bodies[2].gpu_pose_index, 2] = 5.0
        state[bodies[2].gpu_pose_index, 7:13] = 0.0
        torch.cuda.synchronize()
        px.gpu_apply_rigid_dynamic_data()
        px.step()
        positions, impulses, totals = query()
        self.assertTrue(torch.all(totals[:2].norm(dim=-1) > 1e-6).item())
        for output in (positions, impulses, totals):
            self.assertEqual(torch.count_nonzero(output[2]).item(), 0)

    def test_queries_clear_outputs_without_contacts(self):
        px = sapien.physx.PhysxGpuSystem()
        scene = sapien.Scene(systems=[px])
        px.timestep = 0.01
        ground = scene.add_ground(0.0, render=False)
        floor = ground.find_component_by_type(sapien.physx.PhysxRigidStaticComponent)
        builder = scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.05, 0.04, 0.03])
        builder.set_initial_pose(sapien.Pose(p=[0.2, -0.3, 0.03]))
        body = builder.build().find_component_by_type(
            sapien.physx.PhysxRigidDynamicComponent
        )
        px.gpu_init()
        points = px.gpu_create_contact_pair_point_query([(body, floor)])
        pairs = px.gpu_create_contact_pair_impulse_query([(body, floor)])
        bodies = px.gpu_create_contact_body_impulse_query([body])

        def query():
            px.gpu_query_contact_pair_points(points)
            px.gpu_query_contact_pair_impulses(pairs)
            px.gpu_query_contact_body_impulses(bodies)
            torch.cuda.synchronize()
            return [
                points.points.torch(),
                points.impulses.torch(),
                pairs.cuda_impulses.torch(),
                bodies.cuda_impulses.torch(),
            ]

        px.step()
        outputs = query()
        for output in outputs:
            self.assertGreater(output.norm().item(), 0)
        px.gpu_fetch_rigid_dynamic_data()
        state = px.cuda_rigid_body_data.torch()
        state[body.gpu_pose_index, 2] = 5.0
        state[body.gpu_pose_index, 7:13] = 0.0
        torch.cuda.synchronize()
        px.gpu_apply_rigid_dynamic_data()
        for _ in range(2):
            px.step()
        px.gpu_fetch_rigid_dynamic_data()
        self.assertGreater(state[body.gpu_pose_index, 2].item(), 4.9)
        for output in outputs:
            output.fill_(12345.0)
        torch.cuda.synchronize()
        for output in query():
            self.assertEqual(torch.count_nonzero(output).item(), 0)
