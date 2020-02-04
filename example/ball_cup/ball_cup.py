import sapien.core as sapien
from sapien.core import Pose
import numpy as np

# setup
# create renderer to display
renderer = sapien.OptifuserRenderer()  # default renderer for sapien
render_control = sapien.OptifuserController(renderer)  # used to control the renderer

# create simulcation and scene
sim = sapien.Simulation()
sim.set_renderer(renderer)
scene = sim.create_scene()
scene.set_timestep(1 / 60)

# create scene
scene.add_ground(-1)
scene.set_ambient_light([0.2, 0.2, 0.2])
scene.set_shadow_light([1, -1, -1], [.5, .5, .5])

# load cup
cup_file_name = "cup.stl"
cup_pose = Pose([0, 0, 0])
cup_scale = [10] * 3
cup_builder = scene.create_actor_builder()
cup_builder.add_convex_shape_from_file(cup_file_name, cup_pose, cup_scale)
cup_builder.add_visual_from_file(cup_file_name, cup_pose, cup_scale)
cup = cup_builder.build()


# make spheres
def build_sphere(pos, r):
    sphere_builder = scene.create_actor_builder()
    # add physics for sphere
    sphere_builder.add_sphere_shape(Pose(pos), r, None, 400)  # 400 is the density
    # add visual display for sphere
    sphere_builder.add_sphere_visual(Pose(pos), r, color=[1, 1, 1])
    return sphere_builder.build()


build_sphere([0,0,2], 0.02)
# num_balls = 10
# ball_r = 0.1
# balls_distr_r = 0.03
# for i in range(num_balls):
#     pos_z = np.random.uniform(1, 2)
#     rand_theta = np.random.rand() * np.pi * 2
#     pos_x = np.cos(rand_theta) * balls_distr_r
#     pos_y = np.cos(rand_theta) * balls_distr_r
#     pos = [pos_x, pos_y, pos_z]
#     build_sphere(pos, ball_r)


# render
render_control.set_current_scene(scene)
render_control.set_camera_position(-10, -10, 5)
render_control.set_camera_rotation(np.pi / 4, -np.pi / 9)
render_control.show_window()

# simulation loop
while not render_control.should_quit:
    # simulate physics
    scene.step()
    # update render
    scene.update_render()
    # render the frame
    render_control.render()
