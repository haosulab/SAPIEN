# An example of using KuafuRenderer
# Objects (and materials) are randomly generated and
# added into the scene.
#
# TODO:
#  1. fix ErrorOutOfHostMemory by resource managing
#  2. fix segfault when use viewer and resize
#  3. close window
#
# By Jet <i@jetd.me>
#
import sapien.core as sapien
import numpy as np
from sapien.core import Pose
from transforms3d.quaternions import axangle2quat as aa


def main():
    sim = sapien.Engine()

    render_config = sapien.KuafuConfig()
    render_config.spp = 16
    render_config.max_bounces = 5
    render_config.use_viewer = True
    render_config.width = 800
    render_config.height = 600

    renderer = sapien.KuafuRenderer(render_config)
    sim.set_renderer(renderer)

    def create_ant_builder(scene, copper):
        builder = scene.create_articulation_builder()
        body = builder.create_link_builder()
        body.add_sphere_collision(Pose(), 0.25)
        body.add_sphere_visual(Pose(), 0.25, copper)
        body.add_capsule_collision(Pose([0.141, 0, 0]), 0.08, 0.141)
        body.add_capsule_visual(Pose([0.141, 0, 0]), 0.08, 0.141, copper)
        body.add_capsule_collision(Pose([-0.141, 0, 0]), 0.08, 0.141)
        body.add_capsule_visual(Pose([-0.141, 0, 0]), 0.08, 0.141, copper)
        body.add_capsule_collision(Pose([0, 0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141)
        body.add_capsule_visual(Pose([0, 0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141, copper)
        body.add_capsule_collision(Pose([0, -0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141)
        body.add_capsule_visual(Pose([0, -0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141, copper)
        body.set_name("body")

        l1 = builder.create_link_builder(body)
        l1.set_name("l1")
        l1.set_joint_name("j1")
        l1.set_joint_properties(
            "revolute",
            [[-0.5236, 0.5236]],
            Pose([0.282, 0, 0], [0.7071068, 0, 0.7071068, 0]),
            Pose([0.141, 0, 0], [-0.7071068, 0, 0.7071068, 0]),
            0.1,
        )
        l1.add_capsule_collision(Pose(), 0.08, 0.141)
        l1.add_capsule_visual(Pose(), 0.08, 0.141, copper)

        l2 = builder.create_link_builder(body)
        l2.set_name("l2")
        l2.set_joint_name("j2")
        l2.set_joint_properties(
            "revolute",
            [[-0.5236, 0.5236]],
            Pose([-0.282, 0, 0], [0, -0.7071068, 0, 0.7071068]),
            Pose([0.141, 0, 0], [-0.7071068, 0, 0.7071068, 0]),
            0.1,
        )
        l2.add_capsule_collision(Pose(), 0.08, 0.141)
        l2.add_capsule_visual(Pose(), 0.08, 0.141, copper)

        l3 = builder.create_link_builder(body)
        l3.set_name("l3")
        l3.set_joint_name("j3")
        l3.set_joint_properties(
            "revolute",
            [[-0.5236, 0.5236]],
            Pose([0, 0.282, 0], [0.5, -0.5, 0.5, 0.5]),
            Pose([0.141, 0, 0], [0.7071068, 0, -0.7071068, 0]),
            0.1,
        )
        l3.add_capsule_collision(Pose(), 0.08, 0.141)
        l3.add_capsule_visual(Pose(), 0.08, 0.141, copper)

        l4 = builder.create_link_builder(body)
        l4.set_name("l4")
        l4.set_joint_name("j4")
        l4.set_joint_properties(
            "revolute",
            [[-0.5236, 0.5236]],
            Pose([0, -0.282, 0], [0.5, 0.5, 0.5, -0.5]),
            Pose([0.141, 0, 0], [0.7071068, 0, -0.7071068, 0]),
            0.1,
        )
        l4.add_capsule_collision(Pose(), 0.08, 0.141)
        l4.add_capsule_visual(Pose(), 0.08, 0.141, copper)

        f1 = builder.create_link_builder(l1)
        f1.set_name("f1")
        f1.set_joint_name("j11")
        f1.set_joint_properties(
            "revolute",
            [[0.5236, 1.222]],
            Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
            Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]),
            0.1,
        )
        f1.add_capsule_collision(Pose(), 0.08, 0.282)
        f1.add_capsule_visual(Pose(), 0.08, 0.282, copper)

        f2 = builder.create_link_builder(l2)
        f2.set_name("f2")
        f2.set_joint_name("j21")
        f2.set_joint_properties(
            "revolute",
            [[0.5236, 1.222]],
            Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
            Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]),
            0.1,
        )
        f2.add_capsule_collision(Pose(), 0.08, 0.282)
        f2.add_capsule_visual(Pose(), 0.08, 0.282, copper)

        f3 = builder.create_link_builder(l3)
        f3.set_name("f3")
        f3.set_joint_name("j31")
        f3.set_joint_properties(
            "revolute",
            [[0.5236, 1.222]],
            Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
            Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]),
            0.1,
        )
        f2.add_capsule_collision(Pose(), 0.08, 0.282)
        f3.add_capsule_visual(Pose(), 0.08, 0.282, copper)

        f4 = builder.create_link_builder(l4)
        f4.set_name("f4")
        f4.set_joint_name("j41")
        f4.set_joint_properties(
            "revolute",
            [[0.5236, 1.222]],
            Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
            Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]),
            0.1,
        )
        f2.add_capsule_collision(Pose(), 0.08, 0.282)
        f4.add_capsule_visual(Pose(), 0.08, 0.282, copper)

        return builder

    config = sapien.SceneConfig()
    scene = sim.create_scene(config)
    material = renderer.create_material()
    material.set_base_color([1.0, 0.9, 0.7, 1.0])
    scene.add_ground(0, render_material=material)
    scene.set_timestep(1 / 60)

    mount = scene.create_actor_builder().build_kinematic()
    mount.set_pose(Pose([-3, -3, 4], [0.8876263, -0.135299, 0.3266407, 0.2951603]))
    cam1 = scene.add_mounted_camera("cam", mount, Pose([0, 0, 0]), 1920, 1080, 0, 1, 0.1, 100)

    # print(cam1.render_target_names)
    copper = renderer.create_material()
    copper.set_ior(1.4)
    copper.set_transmission(0.0)
    copper.set_base_color([0.875, 0.553, 0.221, 1])
    copper.set_metallic(1.0)
    copper.set_roughness(0.2)
    ant_builder = create_ant_builder(scene, copper)
    ant = ant_builder.build()
    ant.set_root_pose(Pose([1, 0, 8]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.set_base_color([0.2, 0.2, 0.8, 1.0])
    material.set_roughness(0.5)
    material.set_metallic(0.0)
    builder.add_sphere_visual(radius=0.6, material=material)
    builder.add_sphere_collision(radius=0.6)
    sphere1 = builder.build()
    sphere1.set_pose(Pose(p=[0, -2, 4]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.set_ior(1.4)
    material.set_transmission(1.0)
    material.set_base_color([0.2, 0.8, 0.2, 1.0])
    material.set_roughness(0.0)
    material.set_metallic(0.1)
    builder.add_sphere_visual(radius=0.7, material=material)
    builder.add_sphere_collision(radius=0.7)
    sphere2 = builder.build()
    sphere2.set_pose(Pose(p=[0, 0, 6]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.set_ior(1.4)
    material.set_transmission(1.0)
    material.set_base_color([0.2, 0.9, 0.7, 1.0])
    material.set_metallic(0.0)
    builder.add_sphere_visual(radius=0.4, material=material)
    builder.add_sphere_collision(radius=0.4)
    sphere3 = builder.build()
    sphere3.set_pose(Pose(p=[-0.5, 0, 10]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.set_base_color([0.8, 0.2, 0.2, 1.0])
    material.set_roughness(0.0)
    material.set_metallic(1.0)
    builder.add_box_visual(half_size=[0.7, 0.7, 0.7], material=material)
    builder.add_box_collision(half_size=[0.7, 0.7, 0.7])
    box = builder.build()
    box.set_pose(Pose(p=[0, 2, 4]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.set_base_color([0.8, 0.2, 0.2, 1.0])
    material.set_roughness(0.0)
    material.set_metallic(1.0)
    builder.add_capsule_visual(radius=0.08, half_length=0.141, material=material)
    builder.add_capsule_collision(radius=0.08, half_length=0.141)
    cap = builder.build()
    cap.set_pose(Pose(p=[0, 2, 5]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.set_base_color([0.8, 0.1, 0.9, 1.0])
    material.set_roughness(0.0)
    material.set_metallic(1.0)
    builder.add_capsule_visual(radius=0.2, half_length=0.3, material=material)
    builder.add_capsule_collision(radius=0.2, half_length=0.3)
    cap = builder.build()
    cap.set_pose(Pose(p=[0, 2, 12]))

    for i in range(3):
        builder = scene.create_actor_builder()
        material = renderer.create_material()
        material.set_transmission(np.random.rand() > 0.2)
        material.set_ior(1 + np.random.rand())
        material.set_base_color([
            np.random.rand(),
            np.random.rand(),
            np.random.rand(), 1.0])
        material.set_roughness(np.random.rand())
        material.set_metallic(np.random.rand())
        r = 0.2 + np.random.rand() * 0.5
        builder.add_sphere_visual(radius=r, material=material)
        builder.add_sphere_collision(radius=r)
        s = builder.build()
        s.set_pose(Pose(p=[
            np.random.rand(), np.random.rand(), 12 + 6 * i + np.random.rand()]))

        builder = scene.create_actor_builder()
        material.set_transmission(np.random.rand() > 0.2)
        material.set_ior(1 + np.random.rand())
        material.set_base_color([
            np.random.rand(),
            np.random.rand(),
            np.random.rand(), 1.0])
        material.set_roughness(np.random.rand())
        material.set_metallic(np.random.rand())
        r = 0.1 + np.random.rand() * 0.4
        l = 0.2 + np.random.rand() * 0.5
        builder.add_capsule_visual(radius=r, half_length=l, material=material)
        builder.add_capsule_collision(radius=r, half_length=l)
        s = builder.build()
        s.set_pose(Pose(p=[
            np.random.rand(), np.random.rand(), 14 + 6 * i + np.random.rand()]))

        builder = scene.create_actor_builder()
        material.set_transmission(np.random.rand() > 0.2)
        material.set_ior(1 + np.random.rand())
        material.set_base_color([
            np.random.rand(),
            np.random.rand(),
            np.random.rand(), 1.0])
        material.set_roughness(np.random.rand())
        material.set_metallic(np.random.rand())
        r = np.random.rand() * 0.9
        builder.add_box_visual(half_size=[r, r, r], material=material)
        builder.add_box_collision(half_size=[r, r, r])
        s = builder.build()
        s.set_pose(Pose(p=[
            np.random.rand(), np.random.rand(), 16 + 6 * i + np.random.rand()]))

    builder = scene.create_actor_builder()
    builder.add_box_visual(half_size=[0.1, 10, 4])
    builder.add_box_collision(half_size=[0.1, 10, 4])
    w1 = builder.build_static()
    w1.set_pose(Pose(p=[4, 0, 0]))

    builder = scene.create_actor_builder()
    builder.add_box_visual(half_size=[0.1, 10, 4])
    builder.add_box_collision(half_size=[0.1, 10, 4])
    w2 = builder.build_static()
    w2.set_pose(Pose(p=[-4, 0, 0]))

    builder = scene.create_actor_builder()
    builder.add_box_visual(half_size=[10, 0.1, 4])
    builder.add_box_collision(half_size=[10, 0.1, 4])
    w3 = builder.build_static()
    w3.set_pose(Pose(p=[0, 4, 0]))

    builder = scene.create_actor_builder()
    builder.add_box_visual(half_size=[10, 0.1, 4])
    builder.add_box_collision(half_size=[10, 0.1, 4])
    w4 = builder.build_static()
    w4.set_pose(Pose(p=[0, -4, 0]))

    # builder = scene.create_actor_builder()
    # material = renderer.create_material()
    # material.set_base_color([1.0, 1.0, 1.0, 1.0])
    # material.set_ior(1.4)
    # material.set_transmission(0.0)
    # material.set_roughness(1000.0)
    # builder.add_box_visual(half_size=[5, 5, 0.1], material=material)
    # w5 = builder.build_static()
    # w5.set_pose(Pose(p=[0, 0, 8], q=[0.5771257, 0.4940158, 0.4940158, 0.4228743]))

    scene.step()
    ant.set_qpos([0, 0, 0, 0, 0.7, 0.7, 0.7, 0.7])
    ant.set_qvel([0] * 8)
    f = [0.1] * 8
    acc = ant.compute_forward_dynamics([0.1] * 8)
    ant.set_qf(f)
    scene.step()

    scene.renderer_scene.set_ambient_light([0.7, 0.7, 0.65])

    scene.add_directional_light([0.4, 0.4, -1], [4, 4, 4])

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

    cnt = 0
    while True:
        scene.step()
        scene.update_render()
        cam1.take_picture()
        cnt += 1
        if cnt % 100 == 0:
            p = cam1.get_color_rgba()
            import matplotlib.pyplot as plt
            plt.imsave(f'{cnt:04d}.png', p)

main()

# TODO: get targets on camera
