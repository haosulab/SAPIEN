import numpy as np
import sapien.core as sapien
import transforms3d
from sapien.asset import create_dome_envmap
from sapien.core import Pose

from sapien.utils import Viewer

import sapien.core as sapien

def main():
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)

    sapien.render_config.camera_shader_dir = "vertex_color"
    sapien.render_config.viewer_shader_dir = "vertex_color"

    scene_config = sapien.SceneConfig()
    scene = engine.create_scene(scene_config)

    builder = scene.create_actor_builder()
    builder.add_visual_from_file("../assets/models/vertex_color.obj")
    builder.build_static()

    viewer = Viewer(renderer, resolutions=(1920, 1080))
    viewer.set_scene(scene)
    viewer.set_camera_xyz(x=-1.0, y=0, z=0)


    scene.step()
    while not viewer.closed:
        scene.update_render()
        viewer.render()


main()
