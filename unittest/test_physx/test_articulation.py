import os
import unittest
from pathlib import Path

import numpy as np
import sapien
from common import pose_equal, rand_pose


class TestArticulation(unittest.TestCase):
    def test_drive(self):
        scene = sapien.Scene()
        loader = scene.create_urdf_loader()
        robot = loader.load(str(Path(".") / "assets" / "movo_simple.urdf"))

        q = [
            -0.3283431,
            0.39793425,
            -0.81116733,
            0.14926876,
            -0.67020133,
            -0.17215485,
            -0.6565735,
            0.84940983,
            0.40675576,
            0.73837611,
            -0.93073463,
            -0.7414073,
            0.71490287,
        ]
        for j, v in zip(robot.get_active_joints(), q):
            j.set_drive_velocity_target(v)
        self.assertTrue(
            np.allclose(
                [j.get_drive_velocity_target()[0] for j in robot.get_active_joints()], q
            )
        )

        for j, v in zip(robot.get_active_joints(), q):
            j.set_drive_target(v)
        self.assertTrue(
            np.allclose([j.get_drive_target()[0] for j in robot.get_active_joints()], q)
        )

        for j, v in zip(robot.get_active_joints(), q):
            props = (
                float(np.random.rand()),
                float(np.random.rand()),
                float(np.random.rand()),
                np.random.choice(["force", "acceleration"]),
            )
            j.set_drive_properties(*props)
            for a, b in zip(
                [j.stiffness, j.damping, j.force_limit, j.drive_mode], props
            ):
                self.assertAlmostEqual(a, b)

    def test_urdf_loader(self):
        scene = sapien.Scene()
        loader = scene.create_urdf_loader()
        robot = loader.load(str(Path(".") / "assets" / "movo_simple.urdf"))

        q = [0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.3, 0.4, 0.5]
        robot.set_qpos(q)
        self.assertTrue(np.allclose(robot.get_qpos(), q))

        q = [0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.3, 0.4, 0.5]
        robot.set_qvel(q)
        self.assertTrue(np.allclose(robot.get_qvel(), q))

        q = [0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.3, 0.4, 0.5]
        robot.set_qf(q)
        self.assertTrue(np.allclose(robot.get_qf(), q))

    def test_kinematics_dynamics(self):
        scene = sapien.Scene()
        loader = scene.create_urdf_loader()
        robot = loader.load(str(Path(".") / "assets" / "movo_simple.urdf"))

        q = [0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.3, 0.4, 0.5]
        robot.set_qpos(q)

        model = robot.create_pinocchio_model()
        model.compute_forward_kinematics(q)
        for i, l in enumerate(robot.links):
            self.assertTrue(
                pose_equal(l.pose, model.get_link_pose(i), rtol=1e-5, atol=1e-5)
            )

        self.assertTrue(
            np.allclose(
                model.compute_inverse_dynamics(q, np.zeros_like(q), np.zeros_like(q)),
                robot.compute_passive_force(True, True),
                rtol=1e-5,
                atol=1e-5,
            )
        )

    def test_joint(self):
        scene = sapien.Scene()
        loader = scene.create_urdf_loader()
        robot = loader.load(str(Path(".") / "assets" / "movo_simple.urdf"))

        q = [
            -0.3283431,
            0.39793425,
            -0.81116733,
            0.14926876,
            -0.67020133,
            -0.17215485,
            -0.6565735,
            0.84940983,
            0.40675576,
            0.73837611,
            -0.93073463,
            -0.7414073,
            0.71490287,
        ]
        for j in robot.active_joints:
            j.armature = [10]
            self.assertAlmostEqual(j.armature[0], 10)

        for j in robot.joints:
            self.assertTrue(
                pose_equal(j.global_pose, j.child_link.entity_pose * j.pose_in_child)
            )
