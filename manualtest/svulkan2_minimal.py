import sapien.core as sapien
import numpy as np
from sapien.core import Pose
from sapien.utils import Viewer
from transforms3d.quaternions import axangle2quat as aa
from transforms3d.quaternions import qmult, mat2quat, rotate_vector
import time


def main():
    sim = sapien.Engine()
    renderer = sapien.VulkanRenderer()
    sim.set_renderer(renderer)

    config = sapien.SceneConfig()
    scene = sim.create_scene(config)
    scene.add_ground(0)
    scene.set_timestep(1 / 240)

    mount = scene.create_actor_builder().build_static()
    mount.set_pose(Pose([0, 0, 0]))
    camera_pose = Pose([0, 0, 3])
    cam1 = scene.add_mounted_camera(
        "cam", mount, camera_pose, 1920, 1080, 0, 1.6, 0.1, 100)

    builder = scene.create_actor_builder()
    builder.add_box_visual(pose=Pose([0,0,0]), half_size=[0.1, 0.1, 0.1], color=[0.0, 0.0, 1.0])
    builder.add_box_collision(pose=Pose([0,0,0]), half_size=[0.1, 0.1, 0.1])
    a = builder.build()

    # sapien.VulkanRenderer.set_viewer_shader_dir("../vulkan_shader/ibl")
    viewer = Viewer(renderer)
    viewer.set_scene(scene)
    viewer.set_camera_xyz(-4, 0, 0.3)
    viewer.window.set_camera_parameters(0.1, 1000, 1)

    # scene.step()
    # ant.set_qpos([0, 0, 0, 0, 0.7, 0.7, 0.7, 0.7])
    # ant.set_qvel([0] * 8)
    # f = [0.1] * 8
    # acc = ant.compute_forward_dynamics([0.1] * 8)
    # ant.set_qf(f)
    # scene.step()

    # scene.renderer_scene.set_ambient_light([0, 0, 0])
    dirlight = scene.add_directional_light([0, 0, -1], [0.3, 0.3, 0.3], True)
    # plight = scene.add_point_light([0, -1, 1], [2, 1, 2], True)
    # scene.renderer_scene.add_point_light([0, 1, -1], [2, 2, 1])

    while not viewer.closed:
        for i in range(4):
            scene.step()
        scene.update_render()
        viewer.render()

main()

