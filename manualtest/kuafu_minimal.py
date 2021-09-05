# A minimal example of using KuafuRenderer
#
# By Jet <i@jetd.me>
#
import sapien.core as sapien
from sapien.core import Pose


def main():
    sim = sapien.Engine()

    sapien.KuafuRenderer.set_log_level("debug")
    renderer = sapien.KuafuRenderer()

    sim.set_renderer(renderer)

    config = sapien.SceneConfig()
    scene = sim.create_scene(config)

    scene.add_ground(0)
    scene.set_timestep(1 / 60)

    mount = scene.create_actor_builder().build_static()
    cam1 = scene.add_mounted_camera(
        "cam", mount, Pose(), 800, 600, 0, 1.0, 0.1, 100)
    mount.set_pose(Pose([-12, 0, 3]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.base_color = [0.2, 0.2, 0.8, 1.0]
    material.roughness = 0.5
    material.metallic = 0.0
    builder.add_sphere_visual(material=material)
    builder.add_sphere_collision()
    sphere1 = builder.build()
    sphere1.set_pose(Pose(p=[0, 3, 1]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.base_color = [0.8, 0.2, 0.2, 1.0]
    material.roughness = 0.05
    material.metallic = 1.0
    builder.add_sphere_visual(material=material)
    builder.add_sphere_collision()
    sphere2 = builder.build()
    sphere2.set_pose(Pose(p=[-2, 0, 1]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.transmission = 1.0
    material.ior = 1.45
    material.base_color = [0.2, 0.8, 0.2, 1.0]
    # material.roughness = 1.0
    material.roughness = 0.0
    material.specular = 0.0
    builder.add_box_visual(material=material)
    builder.add_box_collision()
    box = builder.build()
    box.set_pose(Pose(p=[0, 0, 4]))

    scene.set_ambient_light([0.1, 0.1, 0.1])
    dirlight = scene.add_directional_light(
        [0, 0, -1], color=[2.0, 2.0, 2.0]
    )

    while renderer.is_running:
        scene.step()
        scene.update_render()
        cam1.take_picture()
        box.set_angular_velocity([0, 0, 1])

main()

