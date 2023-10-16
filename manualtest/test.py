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

    reinstall = False

    scene.load_widget_from_package(
        "https://storage1.ucsd.edu/datasets/ManiSkill3-fxiang/default_arena.zip",
        "DefaultArena",
        reinstall=reinstall,
    )
    scene.set_timestep(1 / 120)

    xarm = scene.load_widget_from_package(
        "https://storage1.ucsd.edu/datasets/ManiSkill3-fxiang/xarm7.zip",
        "XArm7",
        reinstall=reinstall,
    )

    e: sapien.Entity = xarm.robot.links[1].entity

    comp = e.find_component_by_type(sapien.physx.PhysxArticulationLinkComponent)







    # e.find_component_by_type(sapien.physx.PhysxArticulationLinkComponent)
    # print(e.find_component_by_type(sapien.render.RenderBodyComponent))

    viewer = Viewer()
    viewer.set_scene(scene)

    b = scene.create_actor_builder()
    size = [0.015, 0.015, 0.015]
    b.add_box_collision(half_size=size)
    b.add_box_visual(half_size=size, material=[1, 0, 0])
    box = b.build(name="red cube")
    box.set_pose(sapien.Pose([0.4, 0, 0.015]))

    scene.step()

    count = 0
    while not viewer.closed:
        scene.step()
        viewer.render()
        xarm.set_gripper_target((np.sin(count / 100) + 1) * 0.4)
        count += 1


main()
