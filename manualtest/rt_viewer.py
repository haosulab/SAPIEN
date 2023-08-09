import numpy as np
import sapien
import transforms3d
from sapien.asset import create_dome_envmap
from sapien.core import Pose

from sapien.utils import Viewer

import ctypes

def main():
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)

    sapien.render.set_camera_shader_dir("../vulkan_shader/rt")
    sapien.render.set_viewer_shader_dir("../vulkan_shader/rt")
    sapien.render.set_ray_tracing_samples_per_pixel(32)
    sapien.render.set_ray_tracing_path_depth(8)
    sapien.render.set_ray_tracing_denoiser("oidn")

    scene_config = sapien.SceneConfig()
    scene = engine.create_scene(scene_config)

    scene.set_environment_map(create_dome_envmap())
    scene.set_ambient_light([0.1, 0.1, 0.1])
    scene.add_point_light([0, 0, 0.5], [1, 1, 1])
    scene.add_directional_light([0, 0, -1], [1, 1, 1])

    ground_material = renderer.create_material()
    ground_material.base_color = np.array([202, 164, 114, 256]) / 256
    ground_material.specular = 0.5
    scene.add_ground(0, render_material=ground_material, render_half_size=[1.0, 10.0])
    scene.set_timestep(1 / 240)

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.base_color = [0.2, 0.2, 0.8, 1.0]
    material.roughness = 0.5
    material.metallic = 0.0
    builder.add_sphere_visual(radius=0.06, material=material)
    builder.add_sphere_collision(radius=0.06)
    sphere1 = builder.build()
    sphere1.set_pose(Pose(p=[-0.05, 0.05, 0.06]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.ior = 1.2
    material.transmission = 1.0
    material.base_color = [1.0, 1.0, 1.0, 1.0]
    material.roughness = 0.4
    material.metallic = 0.0
    builder.add_sphere_visual(radius=0.07, material=material)
    builder.add_sphere_collision(radius=0.07)
    sphere2 = builder.build()
    sphere2.set_pose(Pose(p=[0.05, -0.05, 0.07]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.base_color = [0.8, 0.7, 0.1, 1.0]
    material.roughness = 0.1
    material.metallic = 1.0
    builder.add_capsule_visual(radius=0.02, half_length=0.1, material=material)
    builder.add_capsule_collision(radius=0.02, half_length=0.1)
    cap = builder.build()
    cap.set_pose(
        Pose(p=[0.15, -0.01, 0.01], q=transforms3d.euler.euler2quat(0, 0, -0.7))
    )

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.base_color = [0.8, 0.2, 0.2, 1.0]
    material.roughness = 0.1
    material.metallic = 1.0
    builder.add_box_visual(half_size=[0.09, 0.09, 0.09], material=material)
    builder.add_box_collision(half_size=[0.09, 0.09, 0.09])
    box = builder.build()
    box.set_pose(Pose(p=[0.05, 0.17, 0.09]))

    viewer = Viewer(renderer)
    viewer.set_scene(scene)
    viewer.set_camera_xyz(x=-0.5, y=0, z=0.25)
    viewer.set_camera_rpy(r=0, p=-np.arctan2(2, 4), y=0)

    scene.step()
    while not viewer.closed:
        scene.update_render()
        viewer.render()


main()
