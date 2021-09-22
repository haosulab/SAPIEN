import numpy as np
import matplotlib.pyplot as plt
import sapien.core as sapien

engine = sapien.Engine()
renderer = sapien.VulkanRenderer(True)
engine.set_renderer(renderer)

scene = engine.create_scene()
b = scene.create_actor_builder()
actor = b.build_kinematic()
actor.set_pose(sapien.Pose([-5, 0, 0]))

cam = scene.add_mounted_camera(
    "", actor, sapien.Pose(), 64, 64, 1, 0.01, 10,
)

b = scene.create_actor_builder()
b.add_box_visual(half_size=[0.25, 0.25, 0.25], color=[1, 0, 0, 1])
b.add_box_visual(
    sapien.Pose([0, 0.5, 0]), half_size=[0.25, 0.25, 0.25], color=[0, 1, 0, 1]
)
b.build_kinematic()

scene.set_ambient_light([0.5, 0.5, 0.5])
scene.update_render()

cam.take_picture()
color = cam.get_color_rgba()
albedo = cam.get_albedo_rgba()
position = cam.get_position_rgba()
segmentation = cam.get_visual_actor_segmentation()
