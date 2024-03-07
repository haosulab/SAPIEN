import sapien
import numpy as np
from sapien import Pose
from transforms3d.quaternions import axangle2quat as aa
from transforms3d.quaternions import qmult, mat2quat, rotate_vector

from sapien.asset import download_partnet_mobility, create_dome_envmap

import argparse

from sapien.utils import Viewer


sapien.render.set_viewer_shader_dir("../vulkan_shader/default")
sapien.render.set_camera_shader_dir("../vulkan_shader/default")


def create_table(
    scene: sapien.Scene,
    pose: sapien.Pose,
    size,
    height,
    thickness=0.1,
    color=(0.8, 0.6, 0.4),
    name="table",
):
    """Create a table (a collection of collision and visual shapes)."""
    builder = scene.create_actor_builder()

    # Tabletop
    tabletop_pose = sapien.Pose(
        [0.0, 0.0, -thickness / 2]
    )  # Make the top surface's z equal to 0
    tabletop_half_size = [size / 2, size / 2, thickness / 2]
    builder.add_box_shape(pose=tabletop_pose, half_size=tabletop_half_size)
    builder.add_box_visual(
        pose=tabletop_pose, half_size=tabletop_half_size, color=color
    )

    # Table legs (x4)
    for i in [-1, 1]:
        for j in [-1, 1]:
            x = i * (size - thickness) / 2
            y = j * (size - thickness) / 2
            table_leg_pose = sapien.Pose([x, y, -height / 2])
            table_leg_half_size = [thickness / 2, thickness / 2, height / 2]
            builder.add_box_shape(pose=table_leg_pose, half_size=table_leg_half_size)
            builder.add_box_visual(
                pose=table_leg_pose, half_size=table_leg_half_size, color=color
            )

    table = builder.build(name=name)
    table.set_pose(pose)
    return table


def main():
    sapien.render.set_log_level("info")
    sapien.render.set_global_config(default_mipmap_levels=4)

    sim = sapien.Engine()
    renderer = sapien.SapienRenderer()
    sim.set_renderer(renderer)

    copper = renderer.create_material()
    copper.set_base_color([0.875, 0.553, 0.221, 1])
    copper.set_metallic(1)
    copper.set_roughness(0.2)

    viewer = Viewer(renderer)

    def create_ant_builder(scene):
        builder = scene.create_articulation_builder()
        body = builder.create_link_builder()
        body.add_sphere_collision(Pose(), 0.25)
        body.add_sphere_visual(Pose(), 0.25, copper)
        body.add_capsule_collision(Pose([0.141, 0, 0]), 0.08, 0.141)
        body.add_capsule_visual(Pose([0.141, 0, 0]), 0.08, 0.141, copper)
        body.add_capsule_collision(Pose([-0.141, 0, 0]), 0.08, 0.141)
        body.add_capsule_visual(Pose([-0.141, 0, 0]), 0.08, 0.141, copper)
        body.add_capsule_collision(
            Pose([0, 0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141
        )
        body.add_capsule_visual(
            Pose([0, 0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141, copper
        )
        body.add_capsule_collision(
            Pose([0, -0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141
        )
        body.add_capsule_visual(
            Pose([0, -0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141, copper
        )
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
        f3.add_capsule_collision(Pose(), 0.08, 0.282)
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
        f4.add_capsule_collision(Pose(), 0.08, 0.282)
        f4.add_capsule_visual(Pose(), 0.08, 0.282, copper)

        return builder

    config = sapien.SceneConfig()
    scene = sim.create_scene(config)
    scene.add_ground(0)
    scene.set_timestep(1 / 240)

    create_dome_envmap()
    scene.set_environment_map(sapien.render.RenderCubemap("/tmp/sapien_dome.ktx"))

    mount = scene.create_actor_builder().build_kinematic()
    mount.set_pose(Pose([-3, 0, 2], qmult(aa([0, 0, 1], 0.3), aa([0, 1, 0], 0.5))))
    cam1 = scene.add_mounted_camera("cam", mount, Pose(), 1920, 1080, 1, 0.1, 100)

    print(cam1.get_projection_matrix())

    mount = scene.create_actor_builder().build_kinematic()
    mount.set_pose(Pose([-3, 0, 2], qmult(aa([0, 0, 1], 0.3), aa([0, 1, 0], 0.5))))
    cam2 = scene.add_mounted_camera("cam", mount, Pose(), 1920, 1080, 1, 0.1, 100)

    ant_builder = create_ant_builder(scene)
    ant = ant_builder.build()
    ant.set_root_pose(Pose([0, 0, 5]))

    loader = scene.create_urdf_loader()
    loader.fix_root_link = True

    robots, actors = loader.load_multiple("../assets/robot/movo/movo.urdf")
    robot = robots[0]
    robot.set_pose(sapien.Pose([-1, 0, 0]))

    # for j in robot.active_joints:
    #     j.set_drive_properties(1000, 100)

    pan_link = next(l for l in robot.links if l.name == "pan_link")
    body_link = next(l for l in robot.links if l.name == "linear_actuator_link")
    shoulder_link = next(l for l in robot.links if l.name == "right_shoulder_link")
    arm_link = next(l for l in robot.links if l.name == "right_arm_half_1_link")

    for c in (
        body_link.collision_shapes
        + pan_link.collision_shapes
        + shoulder_link.collision_shapes
        + arm_link.collision_shapes
    ):
        c.set_collision_groups([1, 1, 1, 0])

    scene.add_directional_light([0, 0, -1], [0.2, 0.2, 0.2], True)
    scene.add_spot_light([0.5, 0, 1], [0, 0, -1], 0.5, 1.0, [2, 0, 0], True)

    # data = sapien.serialization.serialize_scene(scene)

    viewer.set_scene(scene)
    viewer.set_camera_xyz(-4, 0, 0.3)
    viewer.window.set_camera_parameters(0.1, 1000, 1)

    while not viewer.closed:
        for i in range(4):
            robot.set_qf(robot.compute_passive_force())
            scene.physx_system.step()
        scene.render_system.step()

        viewer.render()

    viewer.close()


main()
