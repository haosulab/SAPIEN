import sapien.core as sapien
from sapien.core import Pose
import sapien.core.pysapien.renderer as R
from transforms3d.quaternions import axangle2quat as aa
import numpy as np

from sapien.utils import Viewer


sapien.SapienRenderer.set_log_level("info")

engine = sapien.Engine()
renderer = sapien.SapienRenderer()
renderer_context: R.Context = renderer._internal_context
engine.set_renderer(renderer)

viewer = Viewer(renderer)

brdf_lut = renderer_context.create_brdf_lut()
cubemap = renderer_context.create_cubemap_from_files(
    [
        "../assets/images/cube/px2.png",
        "../assets/images/cube/nx2.png",
        "../assets/images/cube/py2.png",
        "../assets/images/cube/ny2.png",
        "../assets/images/cube/pz2.png",
        "../assets/images/cube/nz2.png",
    ],
    6,
)
viewer.window._internal_renderer.set_custom_cubemap("Environment", cubemap)
viewer.window._internal_renderer.set_custom_texture("BRDFLUT", brdf_lut)


scene = engine.create_scene()
scene.add_ground(-0.01)
scene.set_timestep(1 / 240)

builder = scene.create_actor_builder()
builder.add_box_collision(Pose([0, -0.5, 1]), [0.05, 0.05, 1])
builder.add_box_collision(Pose([0, 0.5, 1]), [0.05, 0.05, 1])
builder.add_box_collision(Pose([0, 0, 2.05]), [0.05, 0.55, 0.05])
builder.add_box_visual(Pose([0, -0.5, 1]), [0.05, 0.05, 1])
builder.add_box_visual(Pose([0, 0.5, 1]), [0.05, 0.05, 1])
builder.add_box_visual(Pose([0, 0, 2.05]), [0.05, 0.55, 0.05])
builder.set_collision_groups(1, 1, 1, 0)
frame = builder.build_kinematic()

builder = scene.create_actor_builder()
builder.add_box_collision(Pose([0, 0, 1]), [0.05, 0.5, 0.97])
builder.add_box_visual(Pose([0, 0, 1]), [0.05, 0.4, 0.97])
builder.set_collision_groups(1, 1, 1, 0)
door = builder.build()

shaft = scene.create_drive(
    frame,
    Pose([0, 0.5, 0], aa([0, 1, 0], -np.pi / 2)),
    door,
    Pose([0, 0.4, 0], aa([0, 1, 0], -np.pi / 2)),
)
shaft.lock_motion(1, 1, 1, 0, 1, 1)
shaft.set_x_twist_limit(-np.pi / 2, 0)

builder = scene.create_actor_builder()
builder.add_box_collision(Pose(), [0.2, 0.015, 0.01])
builder.add_box_visual(Pose(), [0.2, 0.015, 0.01], [0, 1, 1])
builder.set_collision_groups(1, 1, 1, 0)
rod1 = builder.build()

builder = scene.create_actor_builder()
builder.add_box_collision(Pose(), [0.22, 0.015, 0.01])
builder.add_box_visual(Pose(), [0.22, 0.015, 0.01], [0, 0, 1])
builder.set_collision_groups(1, 1, 1, 0)
rod2 = builder.build()

builder = scene.create_actor_builder()
builder.add_box_collision(Pose(), [0.02, 0.02, 0.02])
builder.add_box_visual(Pose(), [0.02, 0.02, 0.02], [1, 0, 0])
builder.set_collision_groups(1, 1, 1, 0)
slider = builder.build()

r1 = scene.create_drive(
    frame,
    Pose([0, 0, 2.01], aa([0, 1, 0], -np.pi / 2)),
    rod1,
    Pose([0.2, -0.1, 0], aa([0, 1, 0], -np.pi / 2)),
)
r1.lock_motion(1, 1, 1, 0, 1, 1)
r1.set_x_twist_properties(0, 10)

r2 = scene.create_drive(
    rod1,
    Pose([-0.2, 0, -0.01], aa([0, 1, 0], -np.pi / 2)),
    rod2,
    Pose([-0.22, 0, 0.01], aa([0, 1, 0], -np.pi / 2)),
)
r2.lock_motion(1, 1, 1, 0, 1, 1)
r2.set_x_twist_properties(0, 10)

p1 = scene.create_drive(door, Pose([-0.07, 0, 1.98]), slider, Pose())
p1.lock_motion(1, 0, 1, 1, 1, 1)
p1.set_y_properties(1e3, 1e3)
p1.set_y_limit(0, 0.2)

p1.set_target(Pose([0, 0.4, 0]))

r3 = scene.create_drive(
    rod2,
    Pose([0.22, 0, 0], aa([0, 1, 0], -np.pi / 2)),
    slider,
    Pose([0, 0, 0], aa([0, 1, 0], -np.pi / 2)),
)
r3.lock_motion(1, 1, 1, 0, 1, 1)
r2.set_target(Pose())
r2.set_x_twist_properties(1e4, 1e3)

viewer.set_scene(scene)
viewer.set_camera_xyz(-4, 0, 3)
viewer.set_camera_rpy(0, -0.6, 0)
viewer.window.set_camera_parameters(0.1, 100, 1)

shaft.set_x_twist_properties(100, 100)
shaft.set_target(Pose([0, 0, 0], aa([-1, 0, 0], 1)))

count = 0
while not viewer.closed:
    for i in range(4):
        scene.step()
    scene.update_render()
    viewer.render()
    if count == 240:
        shaft.set_x_twist_properties(0, 0)
    count += 1
