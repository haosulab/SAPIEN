import sapien.core as sapien
from sapien.utils import Viewer
import numpy as np


def main():
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer(default_mipmap_levels=4)
    renderer_context: R.Context = renderer._internal_context
    engine.set_renderer(renderer)

    scene = engine.create_scene()
    scene.set_timestep(1 / 120)

    b = scene.create_actor_builder().add_box_visual(half_size=[0.5, 0.5, 0.1])

    scene.set_ambient_light([0.3, 0.3, 0.3])

    gear0 = b.build()
    gear1 = b.build()

    gear0.set_pose(sapien.Pose(q=[0.9996573, 0, 0, 0.0261769]))

    j0 = scene.create_drive(
        None,
        sapien.Pose(q=[0.7071, 0.0, -0.7071, 0.0]),
        gear0,
        sapien.Pose(q=[0.7071, 0.0, -0.7071, 0.0]),
    )
    j0.lock_motion(1, 1, 1, 0, 0, 0)

    j1 = scene.create_drive(None, sapien.Pose([0, -1.77, 0]), gear1, sapien.Pose())
    j1.lock_motion(1, 1, 1, 0, 0, 0)

    gear = scene.create_gear(
        gear0,
        sapien.Pose(q=[0.7071068, 0, -0.7071068, 0]),
        gear1,
        sapien.Pose(q=[0.7071068, 0, -0.7071068, 0]),
    )
    gear.gear_ratio = 1.0

    j0.set_target_velocity([0, 0, 0], [1, 0, 0])
    j0.set_x_twist_properties(0, 10)

    viewer = Viewer(renderer)
    viewer.set_scene(scene)

    viewer.set_camera_xyz(-0.5, 0, 0.5)
    viewer.set_camera_rpy(0, -0.5, 0)

    while not viewer.closed:
        scene.step()
        viewer.render()


main()
