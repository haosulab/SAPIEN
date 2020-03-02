import sapien.core as sapien
from sapien.core import Pose
import numpy as np
import transforms3d

RIGHT = 262
LEFT = 263
DOWN = 264
UP = 265
HOME = 268
END = 269
INSERT = 260
DELETE = 261
PAGE_UP = 266
PAGE_DOWN = 267

np.random.seed(1)

# setup
# create renderer to display
renderer = sapien.OptifuserRenderer()  # default renderer for sapien
render_control = sapien.OptifuserController(renderer)  # used to control the renderer

# create simulcation and scene
sim = sapien.Engine()
sim.set_renderer(renderer)
scene: sapien.Scene = sim.create_scene()
scene.set_timestep(1 / 480)

# create scene
scene.add_ground(-1)
scene.set_ambient_light([0.2, 0.2, 0.2])
scene.set_shadow_light([1, -1, -1], [.5, .5, .5])

# setup cups
scale = 0.2
cup_material = sim.create_physical_material(1, 1, 0)  # large friction
cup_builder: sapien.ActorBuilder = scene.create_actor_builder()
cup_builder.add_box_shape(Pose([0, 0, -scale]), [scale, scale, scale / 10], cup_material, density=500)
cup_builder.add_box_visual(Pose([0, 0, -scale]), [scale, scale, scale / 10])

cup_builder.add_box_shape(Pose([scale, 0, 0]), [scale / 10, scale, scale], cup_material, density=500)
cup_builder.add_box_visual(Pose([scale, 0, 0]), [scale / 10, scale, scale])

cup_builder.add_box_shape(Pose([-scale, 0, 0]), [scale / 10, scale, scale], cup_material, density=500)
cup_builder.add_box_visual(Pose([-scale, 0, 0]), [scale / 10, scale, scale])

cup_builder.add_box_shape(Pose([0, scale, 0]), [scale, scale / 10, scale], cup_material, density=500)
cup_builder.add_box_visual(Pose([0, scale, 0]), [scale, scale / 10, scale])

cup_builder.add_box_shape(Pose([0, -scale, 0]), [scale, scale / 10, scale], cup_material, density=500)
cup_builder.add_box_visual(Pose([0, -scale, 0]), [scale, scale / 10, scale])

cup1: sapien.Actor = cup_builder.build()
cup2 = cup_builder.build()
cup2.set_pose(Pose([np.random.rand() * 2 + 1, np.random.rand() * 2 + 1, 0]))


# Setup Balls
def build_sphere(r):
    sphere_builder = scene.create_actor_builder()
    # add physics for sphere
    sphere_builder.add_sphere_shape(Pose(), r, density=1000)
    # add visual display for sphere
    sphere_builder.add_sphere_visual(Pose(), r, color=[0, 1, 1])
    return sphere_builder.build()


num_balls = 80
ball_r = scale / 6
ball_d = ball_r * 1.5
balls = []
for i in range(num_balls):
    p = cup1.pose.p
    rand_theta = np.random.rand() * np.pi * 2
    pos = [p[0], p[1], p[2] + ball_d * i]
    b: sapien.Actor = build_sphere(ball_r)
    b.set_pose(Pose(pos))
    b.set_name('ball' + str(i))
    balls.append(b)

print('total mass', sum([b.mass for b in balls]) + cup1.mass)

# set up gripper
gf = 0.01
gd = 0.1

gripper_material = sim.create_physical_material(10, 10, 0.1)  # large friction
gripper_builder: sapien.ArticulationBuilder = scene.create_articulation_builder()
base_link: sapien.LinkBuilder = gripper_builder.create_link_builder()
fake_x = gripper_builder.create_link_builder(base_link)
fake_x.set_joint_properties(sapien.ArticulationJointType.PRISMATIC, [[-np.inf, np.inf]], Pose(), Pose(), gf,
                            gd)
fake_y = gripper_builder.create_link_builder(fake_x)
q = transforms3d.quaternions.axangle2quat([0, 0, 1], np.pi / 2)
fake_y.set_joint_properties(sapien.ArticulationJointType.PRISMATIC, [[-np.inf, np.inf]], Pose(q=q), Pose(q=q),
                            gf, gd)
fake_z = gripper_builder.create_link_builder(fake_y)
q = transforms3d.quaternions.axangle2quat([0, 1, 0], -np.pi / 2)
fake_z.set_joint_properties(sapien.ArticulationJointType.PRISMATIC, [[-np.inf, np.inf]], Pose(q=q), Pose(q=q),
                            gf, gd)
fake_roll = gripper_builder.create_link_builder(fake_z)
fake_roll.set_joint_properties(sapien.ArticulationJointType.REVOLUTE, [[-np.inf, np.inf]], Pose(), Pose(), gf,
                               gd)
fake_pitch = gripper_builder.create_link_builder(fake_roll)
q = transforms3d.quaternions.axangle2quat([gf, gd, 1], np.pi / 2)
fake_pitch.set_joint_properties(sapien.ArticulationJointType.REVOLUTE, [[-np.inf, np.inf]], Pose(q=q),
                                Pose(q=q), gf, gd)
fake_yaw: sapien.LinkBuilder = gripper_builder.create_link_builder(fake_pitch)
q = transforms3d.quaternions.axangle2quat([0, 1, 0], -np.pi / 2)
fake_yaw.set_joint_properties(sapien.ArticulationJointType.REVOLUTE, [[-np.inf, np.inf]], Pose(q=q),
                              Pose(q=q), gf, gd)
fake_yaw.set_mass_and_inertia(0.5, Pose(), [0.1, 0.1, 0.1])

R = transforms3d.quaternions.axangle2quat([0, 0, 1], np.pi / 2)
left_finger: sapien.LinkBuilder = gripper_builder.create_link_builder(fake_yaw)
left_finger.add_box_shape(Pose(), [0.1, 0.02, 0.1], gripper_material)
left_finger.add_box_visual(Pose(), [0.1, 0.02, 0.1], color=[1, 0, 0])
left_finger.set_joint_properties(sapien.ArticulationJointType.PRISMATIC, [[0.01, 0.5]], Pose(q=R), Pose(q=R),
                                 0, 0)
R2 = transforms3d.quaternions.axangle2quat([0, 0, 1], -np.pi / 2)
right_finger: sapien.LinkBuilder = gripper_builder.create_link_builder(fake_yaw)
right_finger.add_box_shape(Pose(), [0.1, 0.02, 0.1], gripper_material)
right_finger.add_box_visual(Pose(), [0.1, 0.02, 0.1], color=[1, 0, 0])
right_finger.set_joint_properties(sapien.ArticulationJointType.PRISMATIC, [[0.01, 0.5]], Pose(q=R2),
                                  Pose(q=R2), 0, 0)
gripper: sapien.Articulation = gripper_builder.build(True)
gripper.set_root_pose(Pose([-1, 0, -0.5]))

# render
render_control.set_current_scene(scene)
render_control.set_camera_position(-5, -5, 4)
render_control.set_camera_rotation(np.pi / 4, -np.pi / 9)
render_control.show_window()


def do_action(array):
    array = np.array(array)
    F = gripper.compute_passive_force(True, True, False)
    gripper.set_qf(array + F)


def random():
    return np.random.rand() * 10 - 5


count = 0
# simulation loop
while not render_control.should_quit:
    ground_ball = set()
    count += 1

    I: sapien.Input = render_control.input
    action = np.zeros(8)
    S = 3
    if I.get_key_state(DOWN):
        action[0] -= S
    if I.get_key_state(UP):
        action[0] += S
    if I.get_key_state(LEFT):
        action[1] += S
    if I.get_key_state(RIGHT):
        action[1] -= S
    if I.get_key_state(HOME):
        action[2] += S
    if I.get_key_state(END):
        action[2] -= S
    if I.get_key_state(INSERT):
        action[6] += 2
        action[7] += 2
    if I.get_key_state(DELETE):
        action[6] -= 2
        action[7] -= 2

    if I.get_key_state(PAGE_UP):
        action[6] -= 50
        action[7] -= 50
        action[2] += 300
    if I.get_key_state(PAGE_DOWN):
        action[2] -= 300

    do_action(action)

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
    for i in range(6):
        scene.step()
    # update render
    scene.update_render()
    # render the frame
    render_control.render()

scene = 0
