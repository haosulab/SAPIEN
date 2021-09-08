# In this demo, we will render a TOC scene using Kuafu backend.
# Images will be save to local menu every 400 frames.
# You will need to clone https://github.com/haosulab/ICCV2021_Diagnosis/
# and set `repo_root` to run this demo.
#
# By Jet <i@jetd.me>
#
import os
import sapien.core as sapien
import numpy as np
from sapien.core import Pose


def load_obj(scene, materials_root, obj_name, pose=Pose(), material=None):
    builder = scene.create_actor_builder()
    builder.add_visual_from_file(
        os.path.join(materials_root, 'models', obj_name, 'visual_mesh.obj'), material=material)
    builder.add_multiple_collisions_from_file(
        os.path.join(materials_root, 'models', obj_name, 'collision_mesh.obj'))
    obj = builder.build()
    obj.set_pose(pose)


def main():
    repo_root = '/zdata/ssource/ICCV2021_Diagnosis/'
    materials_root = os.path.join(repo_root, 'ocrtoc_materials')

    sim = sapien.Engine()

    sapien.KuafuRenderer.set_log_level('debug')

    render_config = sapien.KuafuConfig()
    render_config.use_viewer = True
    render_config.viewer_width = 960
    render_config.viewer_height = 540
    render_config.spp = 16
    render_config.max_bounces = 6

    renderer = sapien.KuafuRenderer(render_config)
    sim.set_renderer(renderer)
    # renderer.set_environment_map('/home/jet/Downloads/cocacola.jpg')

    scene_config = sapien.SceneConfig()
    scene_config.solver_iterations = 25
    scene_config.solver_velocity_iterations = 2
    scene_config.enable_pcm = False
    scene_config.default_restitution = 0
    scene_config.default_dynamic_friction = 0.5
    scene_config.default_static_friction = 0.5
    scene = sim.create_scene(scene_config)

    ground_material = renderer.create_material()
    ground_material.base_color = np.array([202, 164, 114, 256]) / 256
    ground_material.specular = 0.5
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

    robot = loader.load(os.path.join(repo_root, 'sapien_simulator/urdf/xarm7_sapien.urdf'), urdf_config)
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

    voss_material = renderer.create_material()
    voss_material.base_color = [1, 1, 1, 1]
    voss_material.roughness = 0.0
    voss_material.metallic = 0.0
    voss_material.transmission = 1.0

    load_obj(scene, materials_root, 'camera', Pose(p=[0.7, 0, 1]))
    load_obj(scene, materials_root, 'voss', Pose(p=[0.3, 0.2, 1]), material=voss_material)
    load_obj(scene, materials_root, 'steel_ball', Pose(p=[0.4, -0.2, 1]))
    load_obj(scene, materials_root, 'tennis_ball', Pose(p=[0.3, -0.2, 1]))
    load_obj(scene, materials_root, 'coca_cola', Pose(p=[0.2, 0.3, 1]))

    # ceiling light
    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.base_color = [1., 1., 1., 1.]
    material.emission = [1., 1., 1., 2.]
    builder.add_box_visual(half_size=[5, 5, 0.1], material=material)
    builder.add_box_collision(half_size=[5, 5, 0.1])
    ceil_light = builder.build_static()
    ceil_light.set_pose(Pose(p=[0, 0, 3]))

    # builder = scene.create_actor_builder()
    # builder.add_visual_from_file(
    #     '/home/jet/sapien_dev/new/SAPIEN/3rd_party/kuafu/resources/all_models/CornellBox.obj')
    # obj = builder.build_kinematic()
    # obj.set_pose(Pose(p=[0, -1, 0]))

    builder = scene.create_actor_builder()
    cam_mount = builder.build_kinematic(name='real_camera')
    cam_mount.set_pose(sapien.Pose(
        [0.79111, 0.247229, 0.703505], [0.13942, 0.452553, 0.0629925, -0.878516]))
    cam = scene.add_mounted_camera("cam", cam_mount, Pose([0, 0, 0]), 1280, 720, 0, 1, 0.1, 100)

    scene.step()

    scene.renderer_scene.set_ambient_light([0.003, 0.003, 0.003])
    # dirlight = scene.add_directional_light(
    #     [0, 0.5, -1], color=[5.0, 5.0, 5.0]
    # )

    # slight = scene.add_spot_light(
    #     position=[0, 0, 1],
    #     direction=[0, 0, -1],
    #     inner_fov=0.5,
    #     outer_fov=0.5,
    #     color=[5.0, 5.0, 5.0]
    # )

    alight = scene.add_active_light(
        pose=Pose([0, 0, 0.8]),
        # pose=cam_mount.get_pose(),
        color=[0, 0, 0],
        fov=1.57,
        tex_path="../3rd_party/kuafu/resources/d415-pattern-sq.png"
    )

    # plight = scene.add_point_light(
    #     [0, 0, 2], [3, 3, 3]
    # )
    # #
    # plight = scene.add_point_light(
    #     [-2, -2, 2], [3, 3, 3]
    # )
    #
    # plight = scene.add_point_light(
    #     [-2, 2, 2], [3, 3, 3]
    # )
    #
    # plight = scene.add_point_light(
    #     [2, -2, 2], [3, 3, 3]
    # )
    #
    # plight = scene.add_point_light(
    #     [2, 2, 2], [3, 3, 3]
    # )

    cnt = 0
    while renderer.is_running:
        scene.step()
        scene.update_render()
        cam.take_picture()     # will update viewer and download rgba for now (sync)
        cnt += 1

        if cnt % 400 == 0:
            ceil_light.set_pose(Pose(p=[0, 0, 3]))
            alight.set_color([0, 0, 0])

        if cnt % 400 == 200:
            ceil_light.set_pose(Pose(p=[100, 100, 3]))
            alight.set_color([1, 0, 0])

        if cnt % 400 == 300:
            p = cam.get_color_rgba()
            import matplotlib.pyplot as plt
            plt.imsave(f'{cnt:04d}.png', p)


main()
