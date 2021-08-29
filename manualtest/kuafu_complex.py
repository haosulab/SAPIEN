import sapien.core as sapien
import numpy as np
from sapien.core import Pose
from transforms3d.quaternions import axangle2quat as aa
from transforms3d.quaternions import qmult, mat2quat, rotate_vector
import time


def main():
    sim = sapien.Engine()
    renderer = sapien.KuafuRenderer()
    renderer.set_assets_path('/zdata/anaconda3/envs/sapien/lib/python3.7/site-packages/sapien/kuafu_assets')
    sim.set_renderer(renderer)

    renderer.init()

    def create_ant_builder(scene):
        copper = renderer.create_material()
        copper.set_transparent(False, 1.4)
        copper.set_base_color([0.875, 0.553, 0.221, 1])
        copper.set_metallic(0)
        copper.set_roughness(0)

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
    scene.set_timestep(1 / 240)

    mount = scene.create_actor_builder().build_kinematic()
    mount.set_pose(Pose([-3, -3, 4], [0.8876263, -0.135299, 0.3266407, 0.2951603]))
    cam1 = scene.add_mounted_camera("cam", mount, Pose([0, 0, 0]), 1920, 1080, 0, 1, 0.1, 100)

    # print(cam1.render_target_names)

    ant_builder = create_ant_builder(scene)
    ant = ant_builder.build()
    ant.set_root_pose(Pose([1, 0, 8]))


    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.set_base_color([0.2, 0.2, 0.8, 1.0])
    material.set_roughness(1000.0)
    material.set_metallic(0.0)
    builder.add_sphere_visual(radius=0.6, material=material)
    builder.add_sphere_collision(radius=0.6)
    sphere1 = builder.build()
    sphere1.set_pose(Pose(p=[0, -2, 4]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.set_transparent(True, 1.4)
    material.set_base_color([0.2, 0.8, 0.2, 1.0])
    material.set_roughness(0.0)
    material.set_metallic(100.0)
    builder.add_sphere_visual(radius=0.7, material=material)
    builder.add_sphere_collision(radius=0.7)
    sphere2 = builder.build()
    sphere2.set_pose(Pose(p=[0, 0, 6]))

    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.set_transparent(True, 1.4)
    material.set_base_color([0.2, 0.9, 0.7, 1.0])
    material.set_metallic(0.0)
    builder.add_sphere_visual(radius=0.4, material=material)
    builder.add_sphere_collision(radius=0.4)
    sphere3 = builder.build()
    sphere3.set_pose(Pose(p=[-0.5, 0, 15]))

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

    # builder = scene.create_actor_builder()
    # material = renderer.create_material()
    # material.set_base_color([1.0, 1.0, 1.0, 1.0])
    # material.set_transparent(False, 1.4)
    # material.set_roughness(1000.0)
    # material.set_specular(0.0)
    # builder.add_box_visual(half_size=[5, 5, 5], material=material)
    # # builder.add_box_collision(half_size=[5, 5, 5])
    # room = builder.build_static()
    # room.set_pose(Pose(p=[0, 0, 0]))
    #
    # builder = scene.create_actor_builder()
    # material = renderer.create_material()
    # material.set_base_color([1.0, 1.0, 1.0, 1.0])
    # material.set_transparent(False, 1.4)
    # material.set_roughness(1000.0)
    # material.set_specular(0.0)
    # builder.add_box_visual(half_size=[0.1, 10, 10], material=material)
    # # builder.add_box_collision(half_size=[5, 5, 5])
    # w1 = builder.build_static()
    # w1.set_pose(Pose(p=[4, 0, 0]))
    #
    # builder = scene.create_actor_builder()
    # material = renderer.create_material()
    # material.set_base_color([1.0, 1.0, 1.0, 1.0])
    # material.set_transparent(False, 1.4)
    # material.set_roughness(1000.0)
    # material.set_specular(0.0)
    # builder.add_box_visual(half_size=[0.1, 10, 10], material=material)
    # # builder.add_box_collision(half_size=[5, 5, 5])
    # w2 = builder.build_static()
    # w2.set_pose(Pose(p=[-4, 0, 0]))
    #
    # builder = scene.create_actor_builder()
    # material = renderer.create_material()
    # material.set_base_color([1.0, 1.0, 1.0, 1.0])
    # material.set_transparent(False, 1.4)
    # material.set_roughness(1000.0)
    # material.set_specular(0.0)
    # builder.add_box_visual(half_size=[10, 0.1, 10], material=material)
    # # builder.add_box_collision(half_size=[5, 5, 5])
    # w3 = builder.build_static()
    # w3.set_pose(Pose(p=[0, 4, 0]))
    #
    # builder = scene.create_actor_builder()
    # material = renderer.create_material()
    # material.set_base_color([1.0, 1.0, 1.0, 1.0])
    # material.set_transparent(False, 1.4)
    # material.set_roughness(1000.0)
    # material.set_specular(0.0)
    # builder.add_box_visual(half_size=[10, 0.1, 10], material=material)
    # # builder.add_box_collision(half_size=[5, 5, 5])
    # w4 = builder.build_static()
    # w4.set_pose(Pose(p=[0, -4, 0]))
    #
    # builder = scene.create_actor_builder()
    # material = renderer.create_material()
    # material.set_base_color([1.0, 1.0, 1.0, 1.0])
    # material.set_transparent(False, 1.4)
    # material.set_roughness(1000.0)
    # material.set_specular(0.0)
    # builder.add_box_visual(half_size=[10, 10, 0.1], material=material)
    # # builder.add_box_collision(half_size=[5, 5, 5])
    # w5 = builder.build_static()
    # w5.set_pose(Pose(p=[0, 0, 6]))

    scene.step()
    ant.set_qpos([0, 0, 0, 0, 0.7, 0.7, 0.7, 0.7])
    ant.set_qvel([0] * 8)
    f = [0.1] * 8
    acc = ant.compute_forward_dynamics([0.1] * 8)
    ant.set_qf(f)
    scene.step()

    scene.renderer_scene.set_ambient_light([0.4, 0.4, 0.4])

    # dirlight = scene.add_directional_light([0, 0, 0], [5, 1, 1], position=[0, 0, 4])

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

    while True:
        scene.step()
        scene.update_render()
        cam1.take_picture()


main()

# TODO: get targets on camera
