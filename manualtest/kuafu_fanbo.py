import sapien.core as sapien
import numpy as np
from sapien.core import Pose
from transforms3d.quaternions import axangle2quat as aa
from transforms3d.quaternions import qmult, mat2quat, rotate_vector
import time

import sapien.core.pysapien.renderer as R
from sapien.asset import download_partnet_mobility

from sapien.utils.viewer import Viewer


kuafu = True

sapien.VulkanRenderer.set_log_level("info")
sapien.KuafuRenderer.set_log_level("info")


def main():
    sim = sapien.Engine()

    if kuafu:
        config = sapien.KuafuConfig()
        config.use_viewer = True
        config.use_denoiser = True
        config.spp = 256
        config.max_textures = 20000
        renderer = sapien.KuafuRenderer(config)
    else:
        renderer = sapien.VulkanRenderer(default_mipmap_levels=4, culling="none")
        viewer = Viewer(renderer)
    sim.set_renderer(renderer)

    config = sapien.SceneConfig()
    scene = sim.create_scene(config)
    scene.set_timestep(1 / 240)

    if not kuafu:
        viewer.set_scene(scene)
        viewer.set_camera_xyz(0.152801, 0.303823, 1.193)
        viewer.set_camera_rpy(0, 0, 2)

    mount = scene.create_actor_builder().build_kinematic()
    mount.set_pose(
        Pose([0.152801, 0.303823, 1.193], qmult(aa([0, 0, 1], -2), aa([0, 1, 0], 0)))
    )
    cam1 = scene.add_mounted_camera("cam", mount, Pose(), 960, 540, 0, 1, 0.1, 100)

    prefix = '/home/jet/Downloads/fbtest/objects/'

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(prefix + "scene5.gltf")
    builder.add_nonconvex_collision_from_file(prefix + "col_scene.stl")
    room = builder.build_static()

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(prefix + "cup.gltf")
    builder.add_multiple_collisions_from_file(prefix + "cup.stl")
    cup = builder.build()
    cup.set_pose(Pose([-1.273, -0.781, 0.809], [0.901, 0, 0, 0.433]))

    cup = builder.build()
    cup.set_pose(Pose([-1.273, -0.881, 0.809], [0.964, 0, 0, 0.-265]))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(prefix + "glass.gltf")
    builder.add_multiple_collisions_from_file(prefix + "glass.stl")
    builder.set_mass_and_inertia(0.1, Pose([0, 0, 0.08]), [0.001, 0.001, 0.0002])
    cup = builder.build()
    cup.set_pose(Pose([-1.51, -1.05, 0.810], [1, 0, 0, 0]))
    cup = builder.build()
    cup.set_pose(Pose([-1.49, -0.95, 0.810], [1, 0, 0, 0]))
    cup = builder.build()
    cup.set_pose(Pose([-1.5, -0.85, 0.810], [1, 0, 0, 0]))
    cup = builder.build()
    cup.set_pose(Pose([-1.45, -0.75, 0.810], [1, 0, 0, 0]))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(prefix + "rack.gltf")
    builder.add_multiple_collisions_from_file(prefix + "rack.stl")
    rack = builder.build()
    rack.set_pose(Pose([-1.368, -1.387, 0.822], [0.779, 0, 0, 0.627]))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(prefix + "plate.gltf")
    builder.add_multiple_collisions_from_file(prefix + "plate.stl")
    plate = builder.build()
    plate.set_pose(Pose([-1.343, -1.293, 0.940], [-0.651, -0.654, -0.181, 0.340]))
    plate = builder.build()
    plate.set_pose(Pose([-1.351, -1.327, 0.940], [-0.651, -0.654, -0.181, 0.340]))
    plate = builder.build()
    plate.set_pose(Pose([-1.359, -1.361, 0.940], [-0.651, -0.654, -0.181, 0.340]))

    strength = np.array([1, 0.9, 0.6]) * 0.5
    plight = scene.add_point_light([-0.05, -1.22, 1.67], strength, True, 0.01, 10)
    plight = scene.add_point_light([-0.05, -1.8, 1.5], strength, True, 0.01, 10)
    scene.set_ambient_light([0.1, 0.1, 0.1])
    # scene.set_environment_map("env.ktx")

    scene.update_render()
    count = 0
    while renderer.is_running:
        if kuafu:
            cam1.take_picture()
        else:
            scene.update_render()
            viewer.render()

        for i in range(4):
            scene.step()

        count += 1


main()
