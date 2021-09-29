# A minimal example of using KuafuRenderer
#
# By Jet <i@jetd.me>
#
import sapien.core as sapien
from sapien.core import Pose


def main():
    sim = sapien.Engine()

    sapien.KuafuRenderer.set_log_level("debug")

    kuafu_config = sapien.KuafuConfig()
    kuafu_config.use_denoiser = True
    kuafu_config.use_viewer = True
    renderer = sapien.KuafuRenderer(kuafu_config)

    sim.set_renderer(renderer)

    config = sapien.SceneConfig()
    scene1 = sim.create_scene(config)
    scene1.add_ground(0)
    scene1.set_timestep(1 / 480)

    mount = scene1.create_actor_builder().build_static()
    cam1 = scene1.add_mounted_camera(
        "cam", mount, Pose(), 800, 600, 0, 1.0, 0.1, 100)
    mount.set_pose(Pose([-12, 0, 3]))

    builder = scene1.create_actor_builder()
    material = renderer.create_material()
    material.base_color = [0.2, 0.2, 0.8, 1.0]
    material.roughness = 0.5
    material.metallic = 0.0
    builder.add_sphere_visual(material=material)
    builder.add_sphere_collision()
    sphere1 = builder.build()
    sphere1.set_pose(Pose(p=[0, 3, 3]))

    scene1.set_ambient_light([0.4, 0.4, 0.4])
    scene1.add_directional_light(
        [-1, -1, -1], color=[3.0, 3.0, 3.0]
    )

    # -----------------------------------

    scene2 = sim.create_scene(config)
    scene2.add_ground(0)
    scene2.set_timestep(1 / 480)

    mount = scene2.create_actor_builder().build_static()
    cam2 = scene2.add_mounted_camera(
        "cam", mount, Pose(), 800, 600, 0, 1.0, 0.1, 100)
    mount.set_pose(Pose([-12, 0, 3]))

    builder = scene2.create_actor_builder()
    material = renderer.create_material()
    material.base_color = [0.8, 0.2, 0.2, 1.0]
    material.roughness = 0.05
    material.metallic = 1.0
    builder.add_sphere_visual(material=material)
    builder.add_sphere_collision()
    sphere2 = builder.build()
    sphere2.set_pose(Pose(p=[-2, 0, 3]))

    scene2.set_ambient_light([0.9, 0.9, 0.9])
    scene2.add_directional_light(
        [1, 1, 1], color=[2.0, 2.0, 2.0]
    )

    counter = 0
    while renderer.is_running:
        if counter % 240 == 239:
            print('Switching!!')
            scene1, scene2 = scene2, scene1
            cam1, cam2 = cam2, cam1

        scene1.step()
        scene1.update_render()
        cam1.take_picture()

        counter += 1

main()

