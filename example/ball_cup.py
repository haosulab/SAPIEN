import sapien.core as sapien
from sapien.core import Pose
import numpy as np

np.random.seed(1)

# setup
# create renderer to display
renderer = sapien.OptifuserRenderer()  # default renderer for sapien
render_control = sapien.OptifuserController(renderer)  # used to control the renderer

# create simulcation and scene
sim = sapien.Simulation()
sim.set_renderer(renderer)
scene: sapien.Scene = sim.create_scene()
scene.set_timestep(1 / 60)

# create scene
scene.add_ground(-1)
scene.set_ambient_light([0.2, 0.2, 0.2])
scene.set_shadow_light([1, -1, -1], [.5, .5, .5])

# load cup
cup_file_name = "ball_cup/cup.obj"
cup_pose = Pose([0, 0, 0])
cup_scale = [7] * 3
cup_builder = scene.create_actor_builder()
cup_builder.add_multiple_convex_shapes_from_file(cup_file_name, cup_pose, cup_scale)
cup_builder.add_visual_from_file(cup_file_name, cup_pose, cup_scale)
cup1 = cup_builder.build()

cup2 = cup_builder.build()
cup2.set_pose(Pose([2, 0, 0]))


# make spheres
def build_sphere(pos, r):
    sphere_builder = scene.create_actor_builder()
    # add physics for sphere
    sphere_builder.add_sphere_shape(Pose(pos), r, None, 400)  # 400 is the density
    # add visual display for sphere
    sphere_builder.add_sphere_visual(Pose(pos), r, color=[0, 1, 1])
    return sphere_builder.build()


num_balls = 100
ball_r = 0.09
balls_distr_r = 0.03

balls = []
for i in range(num_balls):
    pos_z = np.random.uniform(1, 1)
    rand_theta = np.random.rand() * np.pi * 2
    pos_x = np.cos(rand_theta) * balls_distr_r
    pos_y = np.cos(rand_theta) * balls_distr_r
    pos = [pos_x, pos_y, pos_z + i * 0.2]
    b = build_sphere(pos, ball_r)
    b.set_name('ball' + str(i))
    balls.append(b)

# render
render_control.set_current_scene(scene)
render_control.set_camera_position(-5, -5, 4)
render_control.set_camera_rotation(np.pi / 4, -np.pi / 9)
render_control.show_window()


ball_data = [b.pack() for b in balls]
cup1_data = cup1.pack()
cup2_data = cup2.pack()

count = 0
# simulation loop
while not render_control.should_quit:
    ground_ball = set()
    count += 1

    if 130 < count < 200:
        cup1.add_force_torque([100, 0, 3900], [0, 0, 20])
    if 200 < count < 240:
        cup1.add_force_torque([0, 0, 3700], [0, 0, -20])
    if 240 < count < 300:
        cup1.add_force_torque([30, 0, 3200], [0, 100, 0])
    if 300 < count < 480:
        cup1.add_force_torque([0, 0, 3100], [0, -50, 0])

    if count == 660:
        for b, d in zip(balls, ball_data):
            b.unpack(d)
        cup1.unpack(cup1_data)
        cup2.unpack(cup2_data)
        count = 0

    contacts = scene.get_contacts()
    for c in contacts:
        if c.actor1.name == 'ground':
            a = c.actor2
        elif c.actor2.name == 'ground':
            a = c.actor1
        else:
            continue
        if a.name.startswith('ball') and c.separation < 1e-6:
            ground_ball.add(a)

    if count % 60 == 0:
        print(len(ground_ball), 'balls are on the ground!')

    # simulate physics
    scene.step()
    # update render
    scene.update_render()
    # render the frame
    render_control.render()

scene = 0
