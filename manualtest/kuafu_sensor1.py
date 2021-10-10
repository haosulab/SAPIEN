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
from sapien.sensor import ActiveLightSensor
import PIL.Image as im
import matplotlib.pyplot as plt
import open3d as o3d


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
    sim.set_log_level('warning')
    sapien.KuafuRenderer.set_log_level('warning')

    render_config = sapien.KuafuConfig()
    render_config.use_viewer = False
    render_config.spp = 64
    render_config.max_bounces = 8
    render_config.use_denoiser = False

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

    for i in range(1000):
        scene.step()

    scene.set_ambient_light([0.3, 0.3, 0.3])
    scene.add_directional_light(
        [0, 0.5, -1], color=[2.0, 2.0, 2.0]
    )
    scene.add_spot_light(
        position=[0, 0, 1],
        direction=[0, 0, -1],
        inner_fov=0.5,
        outer_fov=0.5,
        color=[5.0, 5.0, 5.0]
    )

    sensor = ActiveLightSensor(
        'sensor', renderer, scene, sensor_type='fakesense_j415')

    sensor.set_pose(sapien.Pose(
        [0.79111, 0.247229, 0.803505],
        [0.13942, 0.452553, 0.0629925, -0.878516]))

    for _ in range(2):
        scene.step()
        scene.update_render()
        sensor.take_picture()

        rgb = sensor.get_rgb()
        im.fromarray((rgb * 255).astype(np.uint8)).show()

        ir_l, ir_r = sensor.get_ir()
        im.fromarray((ir_l * 255).astype(np.uint8)).show()
        im.fromarray((ir_r * 255).astype(np.uint8)).show()

        depth = sensor.get_depth()
        plt.imshow(depth)
        plt.show()
        # im.fromarray(depth).show()

        pc = sensor.get_pointcloud(frame='world', with_rgb=True)
        pc1 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc[..., :3]))
        pc1.colors = o3d.utility.Vector3dVector(pc[..., 3:])
        o3d.visualization.draw_geometries([pc1])

        sensor.set_pose(sapien.Pose(
            [0.69111, 0.207229, 1.103505],
            [0.13942, 0.452553, 0.0629925, -0.878516]))

    while True:
        pass


    # rgb, ir_l, ir_r, depth, pc


main()