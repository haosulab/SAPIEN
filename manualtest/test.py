import sapien
import numpy as np
from sapien.utils import Viewer
import os
import warnings


# sapien.physx.enable_gpu()

# sapien.render.set_viewer_shader_dir("../vulkan_shader/rt")
# sapien.render.set_viewer_shader_dir("../vulkan_shader/trivial")
# sapien.render.set_viewer_shader_dir("rt")


def main():
    engine = sapien.Engine()
    config = sapien.SceneConfig()
    config.solver_iterations = 20
    scene = engine.create_scene(config)

    reinstall = False

    scene.load_widget_from_package("demo_arena", "DemoArena")
    scene.set_timestep(1 / 200)

    # scene.create_urdf_loader().load("/home/fx/Downloads/1000/1000/mobility_cvx.urdf")

    xarm = scene.load_widget_from_package("xarm7", "XArm7")
    franka = scene.load_widget_from_package("franka", "Franka")

    franka.robot.pose = sapien.Pose([0, 1, 0])

    # filename = "/home/fx/Downloads/Spray_Bottle_5.glb"
    # filename = "/home/fx/Downloads/untitled.obj"
    # bottle = (
    #     scene.create_actor_builder()
    #     .add_visual_from_file(filename)
    #     .add_multiple_convex_collisions_from_file(filename)
    #     .build()
    # )

    # entity = (
    #     scene.create_actor_builder()
    #     .add_visual_from_file("./1ce2ee00bf9442cca7327cd58c88775d.glb")
    #     .build()
    # )
    # comp = entity.find_component_by_type(sapien.render.RenderBodyComponent)

    viewer = Viewer()
    viewer.set_scene(scene)

    # b = scene.create_actor_builder()
    # size = [0.015, 0.015, 0.015]
    # b.add_box_collision(half_size=size)
    # b.add_box_visual(half_size=size, material=[1, 0, 0])
    # box = b.build_kinematic(name="red cube")
    # box.set_pose(sapien.Pose([0.4, 0, 0.015]))

    # aabb = bottle.find_component_by_type(
    #     sapien.physx.PhysxRigidBaseComponent
    # ).get_global_aabb_fast()

    # aabb = bottle.find_component_by_type(
    #     sapien.render.RenderBodyComponent
    # ).compute_global_aabb_tight()

    # aabb = bottle.find_component_by_type(
    #     sapien.physx.PhysxRigidBaseComponent
    # ).compute_global_aabb_tight()

    # aabb_handle = viewer.draw_aabb(aabb[0], aabb[1], [1, 0, 1])

    scene.step()

    count = 0
    while not viewer.closed:
        for _ in range(4):
            scene.step()
            count += 1
        # scene.update_render()
        # viewer.render()

        xarm.set_gripper_target((np.sin(count / 100) + 1) * 0.4)

        franka.set_gripper_target((np.sin(count / 100) + 1) * 0.02)

        # aabb = bottle.find_component_by_type(
        #     sapien.render.RenderBodyComponent
        # ).compute_global_aabb_tight()

        # aabb = bottle.find_component_by_type(
        #     sapien.physx.PhysxRigidBaseComponent
        # ).get_global_aabb_fast()

        # aabb = bottle.find_component_by_type(
        #     sapien.physx.PhysxRigidBaseComponent
        # ).compute_global_aabb_tight()

        # viewer.update_aabb(aabb_handle, aabb[0], aabb[1])
        # print(aabb)


main()
