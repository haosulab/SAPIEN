import sapien.core as sapien
import numpy as np
from PIL import Image

# Renderer = sapien.VulkanRenderer
Renderer = sapien.KuafuRenderer


def test_point_light():
    engine = sapien.Engine()
    renderer = Renderer()
    engine.set_renderer(renderer)

    mat = renderer.create_material()
    mat.set_base_color([1, 1, 1, 1])
    mat.set_metallic(0)
    mat.set_specular(0)
    scene = engine.create_scene()
    scene.add_ground(0, render_material=mat)

    b = scene.create_actor_builder()
    b.add_box_visual(half_size=[0.2, 0.2, 0.4])
    b.build_kinematic()

    cam = scene.add_camera("camera", 512, 512, np.pi / 2, 0.1, 10)
    cam.set_local_pose(sapien.Pose([0, 0, 1], [0.7071068, 0, 0.7071068, 0]))

    l1 = scene.add_point_light([0.5, 0, 0.5], [1, 0, 0], False)
    l2 = scene.add_point_light([-0.5, 0.5, 0.5], [0, 1, 0], False)
    l3 = scene.add_point_light([-0.5, -0.5, 0.5], [0, 0, 1], False)

    scene.update_render()
    cam.take_picture()
    img = cam.get_color_rgba()
    Image.fromarray((img * 255).astype(np.uint8)).save("test_point_light.png")

    scene.remove_light(l1)
    scene.remove_light(l2)
    scene.remove_light(l3)

    l1 = scene.add_point_light([0.5, 0, 0.5], [1, 0, 0], True)
    l2 = scene.add_point_light([-0.5, 0.5, 0.5], [0, 1, 0], True)
    l3 = scene.add_point_light([-0.5, -0.5, 0.5], [0, 0, 1], True)

    scene.update_render()
    cam.take_picture()
    img = cam.get_color_rgba()
    Image.fromarray(
        (img * 255).astype(np.uint8)).save("test_point_light_shadow.png")


def test_dir_light():
    engine = sapien.Engine()
    renderer = Renderer()
    engine.set_renderer(renderer)

    mat = renderer.create_material()
    mat.set_base_color([1, 1, 1, 1])
    mat.set_metallic(0)
    mat.set_specular(0)
    scene = engine.create_scene()
    scene.add_ground(0, render_material=mat)

    b = scene.create_actor_builder()
    b.add_sphere_visual(radius=0.2)
    b.build_kinematic()

    cam = scene.add_camera("camera", 512, 512, np.pi / 2, 0.1, 10)
    cam.set_local_pose(sapien.Pose([0, 0, 1], [0.7071068, 0, 0.7071068, 0]))

    l1 = scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])
    l2 = scene.add_directional_light([0, -1, -1], [0.5, 0.5, 0.5])
    l3 = scene.add_directional_light([1, 0, -1], [0.5, 0.5, 0.5])

    scene.update_render()
    cam.take_picture()
    img = cam.get_color_rgba()
    Image.fromarray((img * 255).astype(np.uint8)).save("test_dir_light.png")

    scene.remove_light(l1)
    scene.remove_light(l2)
    scene.remove_light(l3)

    l1 = scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5],
                                     True,
                                     shadow_map_size=2048)
    l2 = scene.add_directional_light([0, -1, -1], [0.5, 0.5, 0.5],
                                     True,
                                     shadow_map_size=4096)
    l3 = scene.add_directional_light([1, 0, -1], [0.5, 0.5, 0.5],
                                     True,
                                     shadow_map_size=8192)

    scene.update_render()
    cam.take_picture()
    img = cam.get_color_rgba()
    Image.fromarray(
        (img * 255).astype(np.uint8)).save("test_dir_light_shadow.png")


def test_spot_light():
    engine = sapien.Engine()
    renderer = Renderer()
    engine.set_renderer(renderer)

    mat = renderer.create_material()
    mat.set_base_color([1, 1, 1, 1])
    mat.set_metallic(0)
    mat.set_specular(0)
    scene = engine.create_scene()
    scene.add_ground(0, render_material=mat)

    b = scene.create_actor_builder()
    b.add_sphere_visual(radius=0.2)
    b.build_kinematic().set_pose(sapien.Pose([0, 0, 0.2]))

    cam = scene.add_camera("camera", 512, 512, np.pi / 2, 0.1, 10)
    cam.set_local_pose(sapien.Pose([0, 0, 1], [0.7071068, 0, 0.7071068, 0]))

    l1 = scene.add_spot_light([0, 0, 0.5], [0, 0, -1], np.pi / 2, np.pi / 2,
                              [1, 1, 1])
    l2 = scene.add_spot_light([1, 0, 0.3], [-1, 0, 0], 0, np.pi / 4, [5, 0, 0])

    scene.update_render()
    cam.take_picture()
    img = cam.get_color_rgba()
    Image.fromarray((img * 255).astype(np.uint8)).save("test_spot_light.png")

    scene.remove_light(l1)
    scene.remove_light(l2)

    l1 = scene.add_spot_light([0, 0, 0.5], [0, 0, -1], np.pi / 2, np.pi / 2,
                              [1, 1, 1], True)
    l2 = scene.add_spot_light([1, 0, 0.3], [-1, 0, 0], 0, np.pi / 4, [5, 0, 0],
                              True)

    scene.update_render()
    cam.take_picture()
    img = cam.get_color_rgba()
    Image.fromarray(
        (img * 255).astype(np.uint8)).save("test_spot_light_shadow.png")


def test_active_light():
    engine = sapien.Engine()
    renderer = Renderer()
    engine.set_renderer(renderer)

    mat = renderer.create_material()
    mat.set_base_color([1, 1, 1, 1])
    mat.set_metallic(0)
    mat.set_specular(0)
    scene = engine.create_scene()
    scene.add_ground(0, render_material=mat)

    b = scene.create_actor_builder()
    b.add_sphere_visual(radius=0.1)
    b.build_kinematic().set_pose(sapien.Pose([0, 0, 0.1]))

    cam = scene.add_camera("camera", 512, 512, np.pi / 2, 0.1, 10)
    cam.set_local_pose(sapien.Pose([0, 0, 1], [0.7071068, 0, 0.7071068, 0]))

    scene.add_active_light(sapien.Pose([0, 0, 0.5]), [1, 1, 1], np.pi / 2,
                           "../assets/images/flashlight.jpg")

    scene.update_render()
    cam.take_picture()
    img = cam.get_color_rgba()
    Image.fromarray((img * 255).astype(np.uint8)).save("test_active_light.png")


# test_point_light()
# test_dir_light()
# test_spot_light()
# test_active_light()
