import sapien.core as sapien
import numpy as np
import transforms3d.euler
from sapien.core import Pose
from sapien.sensor import ActiveLightSensorCUDA
import PIL.Image as im
import matplotlib.pyplot as plt
import open3d as o3d
import time

def main():
    sim = sapien.Engine()
    sim.set_log_level('warning')
    sapien.KuafuRenderer.set_log_level('warning')

    render_config = sapien.KuafuConfig()
    render_config.use_viewer = False
    render_config.spp = 32
    render_config.max_bounces = 8
    render_config.use_denoiser = True

    renderer = sapien.KuafuRenderer(render_config)
    sim.set_renderer(renderer)

    scene_config = sapien.SceneConfig()
    scene = sim.create_scene(scene_config)

    ground_material = renderer.create_material()
    ground_material.base_color = np.array([202, 164, 114, 256]) / 256
    ground_material.specular = 0.5
    scene.add_ground(0, render_material=ground_material)
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
    material.ior = 1.4
    material.transmission = 1.0
    material.base_color = [1.0, 1.0, 1.0, 1.0]
    material.roughness = 0.3
    material.metallic = 0.1
    builder.add_sphere_visual(radius=0.07, material=material)
    builder.add_sphere_collision(radius=0.07)
    sphere2 = builder.build()
    sphere2.set_pose(Pose(p=[0.05, -0.05, 0.07]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.base_color = [0.8, 0.7, 0.1, 1.0]
    material.roughness = 0.01
    material.metallic = 0.95
    builder.add_capsule_visual(radius=0.02, half_length=0.1, material=material)
    builder.add_capsule_collision(radius=0.02, half_length=0.1)
    cap = builder.build()
    cap.set_pose(Pose(p=[0.15, -0.01, 0.01], q=transforms3d.euler.euler2quat(0, 0, -0.7)))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.base_color = [0.8, 0.2, 0.2, 1.0]
    material.roughness = 0.01
    material.metallic = 1.0
    builder.add_box_visual(half_size=[0.09, 0.09, 0.09], material=material)
    builder.add_box_collision(half_size=[0.09, 0.09, 0.09])
    box = builder.build()
    box.set_pose(Pose(p=[0.05, 0.17, 0.09]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.base_color = [0.9, 0.6, 0.5, 1.0]
    material.roughness = 0.0
    material.metallic = 1.0
    builder.add_visual_from_file(
        '../3rd_party/kuafu/resources/models/suzanne.dae', scale=[0.1, 0.1, 0.1], material=material)
    builder.add_box_collision(half_size=[0.1, 0.1, 0.1])
    box = builder.build()
    box.set_pose(Pose(p=[0.15, -0.25, 0.1], q=transforms3d.euler.euler2quat(0, 0, -1)))

    scene.set_ambient_light([0.3, 0.3, 0.3])
    scene.add_directional_light([0, 0.5, -1], color=[3.0, 3.0, 3.0])

    sensor_cuda = ActiveLightSensorCUDA(
        'sensor', renderer, scene, sensor_type='fakesense_j415')

    sensor_cuda.set_pose(
        Pose([-0.28, -0.28, 0.46], [0.8876263, -0.135299, 0.3266407, 0.2951603]))

    scene.step()
    scene.update_render()
    sensor_cuda.take_picture()

    rgb = sensor_cuda.get_rgb()
    im.fromarray((rgb * 255).astype(np.uint8)).show()

    start = time.process_time()
    depth_cuda = sensor_cuda.get_depth()
    print("Runtime of get_depth() for gpu sensor: ", time.process_time() - start)

    plt.imshow(depth_cuda)
    plt.show()


main()
