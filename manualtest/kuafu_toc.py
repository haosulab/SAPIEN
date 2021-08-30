import sapien.core as sapien
import numpy as np
from sapien.core import Pose
from transforms3d.quaternions import axangle2quat as aa
from transforms3d.quaternions import qmult, mat2quat, rotate_vector
import time


def main():
    sim = sapien.Engine()
    renderer = sapien.KuafuRenderer()
    renderer.set_assets_path('/zdata/anaconda3/envs/sapien/lib/python3.7/site-packages/sapien/kuafu_assets')
    sim.set_renderer(renderer)

    renderer.init()

    config = sapien.SceneConfig()
    scene = sim.create_scene(config)
    material = renderer.create_material()
    material.set_base_color([1.0, 0.9, 0.7, 1.0])
    scene.add_ground(0, render_material=material)
    scene.set_timestep(1 / 240)

    mount = scene.create_actor_builder().build_kinematic()
    mount.set_pose(Pose([-3, -3, 4]))
    cam1 = scene.add_mounted_camera("cam", mount, Pose([0, 0, 0]), 1920, 1080, 0, 1, 0.1, 100)

    # print(cam1.render_target_names)

    builder = scene.create_actor_builder()
    # material = renderer.create_material()
    # material.set_transparent(True, 1.4)
    # material.set_base_color([0.2, 0.8, 0.2, 1.0])
    # material.set_roughness(0.0)
    # material.set_metallic(100.0)
    # builder.add_sphere_visual(radius=0.7, material=material)
    # builder.add_sphere_collision(radius=0.7)
    builder.add_visual_from_file(
        '/zdata/ssource/ICCV2021_Diagnosis/ocrtoc_materials/models/camera/visual_mesh.obj')
    builder.add_multiple_collisions_from_file(
        '/zdata/ssource/ICCV2021_Diagnosis/ocrtoc_materials/models/camera/collision_mesh.obj')

    obj = builder.build()
    obj.set_pose(Pose(p=[0, 0, 3]))

    scene.step()

    scene.renderer_scene.set_ambient_light([0.4, 0.4, 0.4])

    # dirlight = scene.add_directional_light([0, 0, 0], [5, 1, 1], position=[0, 0, 4])

    # plight = scene.add_point_light(position=[0, 0, 4], color=[5000000, 0, 0])

    # light = scene.renderer_scene.add_spot_light(
    #     [0, 0, 2], [0, 0, -1], np.pi / 2, [1, 1, 1], True
    # )

    # light.set_position([0, 0, 0.1])
    # light.set_direction([0, -100, -1])
    # light.set_color([100, 100, 100])
    # light.set_shadow_parameters(1, 100)

    # light.set_position([0, 0, 5])
    # light.set_direction([0, -1, -1])

    # plight = scene.add_point_light([0, -1, 1], [2, 1, 2], True)
    # scene.renderer_scene.add_point_light([0, 1, -1], [2, 2, 1])

    # print(scene.get_all_lights())

    while True:
        scene.step()
        scene.update_render()
        cam1.take_picture()


main()

# TODO: get targets on camera
