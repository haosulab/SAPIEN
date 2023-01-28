import sapien.core as sapien
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
from sapien.core import Pose
from sapien.sensor import StereoDepthSensor, StereoDepthSensorConfig


def build_scene(sim, renderer):
    scene_config = sapien.SceneConfig()
    scene = sim.create_scene(scene_config)

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.base_color = [1.0, 1.0, 1.0, 1.0]
    material.diffuse_texture_filename = "../assets/aligned/beer_can/texture.png"
    material.metallic = 0.001
    material.roughness = 0.4
    builder.add_visual_from_file("../assets/aligned/beer_can/visual_mesh.obj", material=material)
    beer_can = builder.build()
    beer_can.set_pose(Pose([0.370301, -0.246856, 0.0738802], [0.922673, -0.00379302, -0.00852731, 0.385469]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.base_color = [1.0, 1.0, 1.0, 1.0]
    material.diffuse_texture_filename = "../assets/aligned/champagne/texture.png"
    material.metallic = 0.01
    material.roughness = 0.2
    builder.add_visual_from_file("../assets/aligned/champagne/visual_mesh.obj", material=material)
    champagne = builder.build()
    champagne.set_pose(Pose([0.182963, -0.277838, 0.0873777], [0.723872, 0.00616071, -0.00678847, -0.689874]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.base_color = [1.0, 1.0, 1.0, 1.0]
    material.diffuse_texture_filename = "../assets/aligned/pepsi_bottle/texture.png"
    material.metallic = 0.001
    material.roughness = 0.6
    builder.add_visual_from_file("../assets/aligned/pepsi_bottle/visual_mesh.obj", material=material)
    pepsi_bottle = builder.build()
    pepsi_bottle.set_pose(Pose([0.392403, 0.0504232, 0.116739], [0.991259, -0.00145631, -0.00922613, 0.131601]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.base_color = [0.6, 0.6, 0.6, 1.0]
    material.metallic = 0.8
    material.roughness = 0.2
    builder.add_visual_from_file("../assets/aligned/steel_ball/visual_mesh.obj", material=material)
    steel_ball = builder.build()
    steel_ball.set_pose(Pose([0.192034, 0.131187, 0.0170772], [0.949057, -0.0375225, 0.0676584, -0.305458]))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file("../assets/aligned/water_bottle/water_bottle.glb")
    water_bottle = builder.build()
    water_bottle.set_pose(Pose([0.256327, -0.0162116, -0.01], [0.00627772, -0.0093401, 0.000145366, 1.0008]))
    
    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.base_color = [1.0, 1.0, 1.0, 1.0]
    material.diffuse_texture_filename = "../assets/aligned/table/texture.png"
    material.metallic = 0.1
    material.roughness = 0.3
    builder.add_visual_from_file("../assets/aligned/table/visual_mesh.obj", material=material)
    table = builder.build()
    table.set_pose(Pose([0.405808, 0.022201, -0.043524], [0.999921, -0.000290915, -0.00932814, 0.00842011]))

    scene.set_ambient_light([0., 0., 0.])
    scene.add_point_light([1.0, 0.2, 2.5], [10, 10, 10])
    scene.add_point_light([1.0, -2.7, 2.5], [10, 10, 10])
    scene.add_point_light([1.0, -5.6, 2.5], [10, 10, 10])
    scene.add_point_light([1.0, 3.1, 2.5], [10, 10, 10])
    scene.add_point_light([1.0, 6.0, 2.5], [10, 10, 10])

    return scene

def main():
    sim = sapien.Engine()

    renderer = sapien.SapienRenderer()
    sim.set_renderer(renderer)
    sapien.render_config.camera_shader_dir = "rt"
    sapien.render_config.rt_samples_per_pixel = 32
    sapien.render_config.rt_max_path_depth = 8
    sapien.render_config.rt_use_denoiser = True

    scene = build_scene(sim, renderer)

    sensor_config = StereoDepthSensorConfig()
    sensor = StereoDepthSensor('sensor', scene, sensor_config)
    sensor.set_pose(Pose([-0.13732728, 0.13584249, 0.82386769],[0.84667092, -0.01287458, 0.53195302, -0.00292851]))

    scene.step()
    scene.update_render()
    sensor.take_picture()
    sensor.compute_depth()

    rgb = sensor.get_rgb()
    ir_l, ir_r = sensor.get_ir()
    depth = sensor.get_depth()

    plt.subplot(221)
    plt.title("RGB Image")
    plt.imshow((rgb * 255).astype(np.uint8))
    plt.subplot(222)
    plt.title("Left Infrared Image")
    plt.imshow((ir_l * 255).astype(np.uint8), cmap='gray')
    plt.subplot(223)
    plt.title("Right Infrared Image")
    plt.imshow((ir_r * 255).astype(np.uint8), cmap='gray')
    plt.subplot(224)
    plt.title("Depth Map")
    plt.imshow(depth)
    plt.show()

    pc = sensor.get_pointcloud(with_rgb=True) # In RGB camera frame with x rightward, y downward, z forward
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc[..., :3] * np.array([1, -1, -1]))) # Change axis direction for easier view
    pcd.colors = o3d.utility.Vector3dVector(pc[..., 3:])
    o3d.visualization.draw_geometries([pcd])


main()
