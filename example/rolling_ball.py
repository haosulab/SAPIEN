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

# stratic friction, dynamic driction, restitution
smooth_material = sim.create_physical_material(10, 10, 1)


def build_sphere(pos, r):
    sphere_builder = scene.create_actor_builder()
    # add physics for sphere
    sphere_builder.add_sphere_shape(Pose(pos), r, None, 400)  # 400 is the density
    # add visual display for sphere
    sphere_builder.add_sphere_visual(Pose(pos), r, color=[1, 1, 1])
    return sphere_builder.build()


def build_box(pos, size):
    box_builder = scene.create_actor_builder()
    # add physics for box
    box_builder.add_box_shape(Pose(pos), size, smooth_material, 1)  # 1 is the density
    # add visual display for box
    box_builder.add_box_visual(Pose(pos), size, color=np.random.uniform(0, 1, size=(3)))
    return box_builder.build()


sphere = build_sphere([0, 0, 0], 0.5)

# random create 5 boxes
BOX_RANDOM_RANGE = 10
for i in range(5):
    pos = np.random.uniform(-BOX_RANDOM_RANGE, BOX_RANDOM_RANGE, size=(3))
    pos[2] = 0

    size = np.random.uniform(0.2, 1, size=(3))
    box = build_box(pos, size)

# render
render_control.set_current_scene(scene)
render_control.set_camera_position(-10, -10, 5)
render_control.set_camera_rotation(np.pi / 4, -np.pi / 9)
render_control.focus(sphere)  # camera follows sphere until mannually move camera
render_control.show_window()


# handle control
def get_rot_matrix(theta):
    '''
        theta: 2D rotation in radians

        return 2D ratation np matrix
    '''
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])


key_to_rotate = {
    'i': get_rot_matrix(0),  # forward
    'j': get_rot_matrix(np.pi / 2),  # left
    'k': get_rot_matrix(np.pi),  # right
    'l': get_rot_matrix(-np.pi / 2)  # backward
}

BALL_MOVE_FORCE = 1000


def on_press(key):
    if key in key_to_rotate.keys():
        rot = key_to_rotate[key]

        # get forward
        cam_p = render_control.get_camera_pose().p[:2]
        print(key)
        ball_p = sphere.pose.p[:2]
        forward = ball_p - cam_p
        forward = forward / np.linalg.norm(forward)

        force_vec = rot @ forward
        force_vec = force_vec / np.linalg.norm(force_vec) * BALL_MOVE_FORCE
        sphere.add_force_torque([force_vec[0], force_vec[1], 0], [0, 0, 0])

print('\n\nUse mouse right click to move view point\n' + 'i: forward\nj: left\nk: backward\nl:right\n' +
      'Camera follows sphere until user moves camera with w,a,s,d')

# simulation loop
while not render_control.should_quit:
    if render_control.input.get_key_state(ord('I')):
        on_press('i')
    if render_control.input.get_key_state(ord('K')):
        on_press('k')
    if render_control.input.get_key_state(ord('J')):
        on_press('j')
    if render_control.input.get_key_state(ord('L')):
        on_press('l')

    # simulate physics
    scene.step()
    # update render
    scene.update_render()
    # render the frame
    render_control.render()
