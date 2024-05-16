import unittest
import sapien
import numpy as np
import matplotlib.pyplot as plt


class TestPicture(unittest.TestCase):
    def test_offscreen(self):
        scene = sapien.Scene()
        scene.set_timestep(1 / 100.0)  # Set the simulation frequency

        scene.add_ground(altitude=0)  # Add a ground
        actor_builder = scene.create_actor_builder()
        actor_builder.add_box_collision(half_size=[0.5, 0.5, 0.5])
        actor_builder.add_box_visual(
            half_size=[0.5, 0.5, 0.5], material=[1.0, 0.0, 0.0]
        )
        box = actor_builder.build(name="box")
        box.set_pose(sapien.Pose(p=[0, 0, 0.5]))

        scene.set_ambient_light([0.5, 0.5, 0.5])

        actor = scene.create_actor_builder().build_kinematic()
        actor.set_pose(sapien.Pose([-3, 0, 0.5]))
        cam = scene.add_mounted_camera("", actor, sapien.Pose(), 32, 32, 1, 0.01, 10)

        scene.update_render()
        cam.take_picture()
        color = cam.get_picture("Color")

        print(color[16, 16], [0.72974, 0, 0, 1])
        self.assertTrue(
            np.allclose(color[16, 16], [0.72974, 0, 0, 1], rtol=1e-3, atol=1e-3)
        )
