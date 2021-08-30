import os
import sapien.core as sapien
import numpy as np
from sapien.core import Pose
from transforms3d.quaternions import axangle2quat as aa
from transforms3d.quaternions import qmult, mat2quat, rotate_vector
import time

def load_obj(scene, materials_root, obj_name, pose=Pose()):
    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        os.path.join(materials_root, 'models', obj_name, 'visual_mesh.obj'))
    builder.add_multiple_collisions_from_file(
        os.path.join(materials_root, 'models', obj_name, 'collision_mesh.obj'))
    obj = builder.build()
    obj.set_pose(pose)


def main():
    materials_root = '/zdata/ssource/ICCV2021_Diagnosis/ocrtoc_materials/'

    sim = sapien.Engine()
    renderer = sapien.KuafuRenderer()
    renderer.set_assets_path(
        '/zdata/anaconda3/envs/sapien/lib/python3.7/site-packages/sapien/kuafu_assets')
    sim.set_renderer(renderer)

    renderer.init()

    scene_config = sapien.SceneConfig()
    scene_config.solver_iterations = 25
    scene_config.solver_velocity_iterations = 2
    scene_config.enable_pcm = False
    scene_config.default_restitution = 0
    scene_config.default_dynamic_friction = 0.5
    scene_config.default_static_friction = 0.5
    scene = sim.create_scene(scene_config)

    ground_material = renderer.create_material()
    ground_color = np.array([202, 164, 114, 256]) / 256
    ground_material.set_base_color(ground_color)
    ground_material.set_specular(0.5)
    scene.add_ground(-0.8, render_material=ground_material)
    scene.set_timestep(1 / 240)


    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    gripper_material = sim.create_physical_material(2.4, 1.6, 0.01)
    urdf_config = {
        "link": {
            "left_inner_finger_pad": {
                "material": gripper_material,
                "patch_radius": 0.02,
                "min_patch_radius": 0.005
            },
            "right_inner_finger_pad": {
                "material": gripper_material,
                "patch_radius": 0.02,
                "min_patch_radius": 0.005
            }}}

    robot = loader.load('/zdata/ssource/ICCV2021_Diagnosis/sapien_simulator/urdf/xarm7_sapien.urdf', urdf_config)
    init_qpos = np.array([0, -0.5235, 0, 0.5235, 0, 1.0470, -0.78539, 0, 0, 0, 0, 0, 0])

    robot.set_qpos(init_qpos)
    robot.set_drive_target(init_qpos)
    for jidx, joint in enumerate(robot.get_active_joints()):
        if jidx in range(7):
            joint.set_drive_property(3000, 500, 1000)
        elif jidx in range(7, 11):
            joint.set_drive_property(50, 15, 40)
        elif jidx in range(11, 13):
            joint.set_drive_property(1000, 200, 1000)

    load_obj(scene, materials_root, 'camera', Pose(p=[-0.2, 0, 4]))
    load_obj(scene, materials_root, 'voss', Pose(p=[0.3, 0.2, 4]))
    load_obj(scene, materials_root, 'steel_ball', Pose(p=[0.4, -0.2, 4]))
    load_obj(scene, materials_root, 'tennis_ball', Pose(p=[0.3, -0.2, 4]))
    load_obj(scene, materials_root, 'coca_cola', Pose(p=[0.2, 0.3, 4]))

    builder = scene.create_actor_builder()
    cam_mount = builder.build_kinematic(name='real_camera')
    cam_mount.set_pose(sapien.Pose(
        [0.79111, 0.247229, 0.703505], [0.13942, 0.452553, 0.0629925, -0.878516]))
    cam = scene.add_mounted_camera("cam", cam_mount, Pose([0, 0, 0]), 1920, 1080, 0, 1, 0.1, 100)

    scene.step()

    scene.renderer_scene.set_ambient_light([0.5, 0.5, 0.5])

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
        cam.take_picture()


main()

# TODO: get targets on camera
