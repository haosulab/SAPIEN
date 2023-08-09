import sapien
import numpy as np
import sapien.render as sr
from sapien.utils import Viewer
from PIL import Image
import pickle


def main():
    scene = sapien.Scene()
    scene.load_module("../modules/default_arena.zip")
    loader = scene.create_urdf_loader()
    loader.load("../modules/xarm_urdf/xarm7_gripper.urdf")

    viewer = Viewer(shader_dir="../vulkan_shader/default")
    viewer.set_scene(scene)

    while not viewer.closed:
        viewer.render()


main()
