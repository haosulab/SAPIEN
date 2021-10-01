# A minimal example of using KuafuRenderer
#
# By Jet <i@jetd.me>
#
import sapien.core as sapien
from sapien.core import Pose
import matplotlib.pyplot as plt


def main():
    sim = sapien.Engine()

    sapien.KuafuRenderer.set_log_level("debug")

    kuafu_config = sapien.KuafuConfig()
    kuafu_config.use_denoiser = False
    kuafu_config.use_viewer = False
    renderer = sapien.KuafuRenderer(kuafu_config)

    sim.set_renderer(renderer)

    config = sapien.SceneConfig()
    scene = sim.create_scene(config)

    scene.add_ground(0)
    scene.set_timestep(1 / 60)

    mount1 = scene.create_actor_builder().build_static()
    cam1 = scene.add_mounted_camera(
        "cam1", mount1, Pose(), 800, 600, 0, 1.0, 0.1, 100)
    mount1.set_pose(Pose([-6, 0, 3]))

    mount2 = scene.create_actor_builder().build_static()
    cam2 = scene.add_mounted_camera(
        "cam2", mount2, Pose(), 800, 600, 0, 1.0, 0.1, 100)
    mount2.set_pose(Pose([-6, 0, 1]))

    builder = scene.create_actor_builder()
    builder.add_box_visual()
    builder.add_box_collision()
    box = builder.build()
    box.set_pose(Pose(p=[0, 0, 4]))

    scene.set_ambient_light([0.6, 0.6, 0.6])

    scene.step()
    scene.update_render()
    cam1.take_picture()
    cam2.take_picture()

    plt.imsave('1.png', cam1.get_color_rgba())
    plt.imsave('2.png', cam2.get_color_rgba())


main()

