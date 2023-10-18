import sapien
import numpy as np
from sapien.utils import Viewer
import os
import warnings


class Comp(sapien.Component):
    pass


def main():
    engine = sapien.Engine()
    config = sapien.SceneConfig()
    config.solver_iterations = 20
    scene = engine.create_scene(config)

    reinstall = True

    scene.load_widget_from_package(
        "https://storage1.ucsd.edu/datasets/ManiSkill3-fxiang/default_arena.zip",
        "DefaultArena",
        reinstall=reinstall,
    )
    scene.set_timestep(1 / 200)

    xarm = scene.load_widget_from_package(
        "https://storage1.ucsd.edu/datasets/ManiSkill3-fxiang/xarm7.zip",
        "XArm7",
        reinstall=reinstall,
    )

    scene.create_actor_builder().add_visual_from_file(
        "/home/fx/Downloads/Spray_Bottle_5.glb"
    ).add_multiple_convex_collisions_from_file(
        "/home/fx/Downloads/Spray_Bottle_5.glb"
    ).build()

    # e: sapien.Entity = xarm.robot.links[1].entity
    # comp = e.find_component_by_type(sapien.physx.PhysxArticulationLinkComponent)

    viewer = Viewer()
    viewer.set_scene(scene)

    b = scene.create_actor_builder()
    size = [0.015, 0.015, 0.015]
    b.add_box_collision(half_size=size)
    b.add_box_visual(half_size=size, material=[1, 0, 0])
    box = b.build_kinematic(name="red cube")
    box.set_pose(sapien.Pose([0.4, 0, 0.015]))

    scene.step()

    count = 0
    while not viewer.closed:
        for _ in range(4):
            scene.step()
            count += 1
        scene.update_render()
        viewer.render()
        xarm.set_gripper_target((np.sin(count / 100) + 1) * 0.4)


main()
