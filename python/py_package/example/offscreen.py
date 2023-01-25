"""Test offscreen renderer for SAPIEN
"""
import sapien.core as sapien
from sapien.utils import Viewer
import numpy as np
from PIL import Image


def main():
    engine = sapien.Engine()  # Create a physical simulation engine
    renderer = sapien.SapienRenderer()  # Create a renderer
    engine.set_renderer(renderer)  # Bind the renderer and the engine

    scene = engine.create_scene()  # Create an instance of simulation world (aka scene)
    scene.set_timestep(1 / 100.0)  # Set the simulation frequency

    # NOTE: How to build actors (rigid bodies) is elaborated in create_actors.py
    scene.add_ground(altitude=0)  # Add a ground
    actor_builder = scene.create_actor_builder()
    actor_builder.add_box_collision(half_size=[0.5, 0.5, 0.5])
    actor_builder.add_box_visual(half_size=[0.5, 0.5, 0.5], color=[1., 0., 0.])
    box = actor_builder.build(name='box')  # Add a box
    box.set_pose(sapien.Pose(p=[0, 0, 0.5]))

    # Add some lights so that you can observe the scene
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    actor = scene.create_actor_builder().build_kinematic()
    actor.set_pose(sapien.Pose([-3, 0, 0.5]))
    cam = scene.add_mounted_camera('', actor, sapien.Pose(), 128, 128, 1, 0.01, 10)

    scene.update_render()
    cam.take_picture()
    color = cam.get_color_rgba()

    Image.fromarray((color[..., :3].clip(0, 1) * 255).astype(np.uint8)).save("output.png")


if __name__ == '__main__':
    main()
