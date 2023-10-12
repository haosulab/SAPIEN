import sapien
import numpy as np
import sapien.render as sr
from sapien.utils import Viewer
from PIL import Image
import pickle


def main():
    scene = sapien.Scene()
    scene.load_widget_from_package(
        "https://storage1.ucsd.edu/datasets/ManiSkill3-fxiang/default_arena.zip",
        "DefaultArena",
    )

    b = scene.create_actor_builder()
    b.add_nonconvex_collisions_from_file("/home/fx/Downloads/cubes/cubes.obj")
    b.add_visual_from_file("/home/fx/Downloads/cubes/cubes.obj")
    e = b.build_kinematic()

    e.pose = sapien.Pose([0, 0, 10])

    viewer = Viewer()
    viewer.set_scene(scene)

    while not viewer.closed:
        scene.step()
        viewer.render()


#     scene = sapien.Scene()
#     scene.load_module("../modules/default_arena.zip")
#     loader = scene.create_urdf_loader()
#     loader.load("../modules/xarm_urdf/xarm7_gripper.urdf")

#     viewer = Viewer(shader_dir="../vulkan_shader/default")
#     viewer.set_scene(scene)

#     while not viewer.closed:
#         viewer.render()


main()
