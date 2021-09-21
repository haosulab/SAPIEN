# A minimal example of using KuafuRenderer
#
# By Jet <i@jetd.me>
#
import sapien.core as sapien
import numpy as np


def main():
    sim = sapien.Engine()

    sapien.KuafuRenderer.set_log_level("debug")

    config = sapien.KuafuConfig()
    config.use_viewer = True
    config.spp = 1

    renderer = sapien.KuafuRenderer(config)

    sim.set_renderer(renderer)

    config = sapien.SceneConfig()
    scene = sim.create_scene(config)

    scene.add_ground(0)
    scene.set_timestep(1 / 60)

    mount = scene.create_actor_builder().build_static()
    cam1 = scene.add_mounted_camera(
        "cam", mount, sapien.Pose(), 800, 600, 0, 1.0, 0.1, 100)
    mount.set_pose(sapien.Pose([-12, 0, 3]))

    for i in range(128):
        builder = scene.create_actor_builder()
        builder.add_capsule_visual()
        builder.add_capsule_collision()
        sphere = builder.build()
        sphere.set_pose(sapien.Pose(p=[np.random.rand(), np.random.rand(), i * 10]))

    scene.set_ambient_light([0.4, 0.4, 0.4])
    dirlight = scene.add_directional_light(
        [-1, -1, -1], color=[3.0, 3.0, 3.0]
    )

    while renderer.is_running:
        scene.step()
        scene.update_render()
        cam1.take_picture()

main()

