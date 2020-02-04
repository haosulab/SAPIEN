import sapien.core as sapien
import transforms3d
from sapien.core import Pose
import numpy as np

sim = sapien.Simulation()
renderer = sapien.OptifuserRenderer()
sim.set_renderer(renderer)
render_controller = sapien.OptifuserController(renderer)

render_controller.show_window()

scene: sapien.Scene = sim.create_scene()
scene.add_ground(-1)
scene.set_timestep(1 / 240)

scene.set_ambient_light([0.5, 0.5, 0.5])
scene.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

loader = scene.create_urdf_loader()
loader.fix_root_link = 1
robot = loader.load("../assets/robot/all_robot.urdf")

actor_builder: sapien.ActorBuilder = scene.create_actor_builder()
actor_builder.add_sphere_visual(Pose(), 0.1, [1, 0, 0], "anchor")
anchor: sapien.Actor = actor_builder.build_static()
anchor.set_pose(Pose([1, 0, 0.5]))

render_controller.set_camera_position(-0.5, 2, 0.5)
render_controller.set_camera_rotation(-1, 0)
render_controller.set_current_scene(scene)

gripper = [l for l in robot.get_links() if l.name == 'right_gripper_base_link'][0]


def move_anchor(dx, dy, dz, droll, dpitch, dyaw):
    quat = transforms3d.euler.euler2quat(droll, dpitch, dyaw)
    anchor.set_pose(Pose([dx, dy, dz]) * anchor.pose * Pose([0, 0, 0], quat))

    # p = anchor.pose.p
    # anchor.set_pose(Pose(p + [dx, dy, dz]))


drive = scene.create_drive(anchor, Pose(), gripper, Pose())
drive.set_properties(4000, 8000)
drive.set_target(Pose())

while not render_controller.should_quit:
    if render_controller.input.get_key_state(ord('I')):
        if render_controller.input.get_key_mods(ord('I')) & 1:
            move_anchor(0, 0, 0, 0, 0.01, 0)
        else:
            move_anchor(0.05, 0, 0, 0, 0, 0)
    if render_controller.input.get_key_state(ord('K')):
        if render_controller.input.get_key_mods(ord('K')) & 1:
            move_anchor(0, 0, 0, 0, -0.01, 0)
        else:
            move_anchor(-0.05, 0, 0, 0, 0, 0)
    if render_controller.input.get_key_state(ord('J')):
        if render_controller.input.get_key_mods(ord('J')) & 1:
            move_anchor(0, 0, 0, -0.01, 0, 0)
        else:
            move_anchor(0, 0.05, 0, 0, 0, 0)
    if render_controller.input.get_key_state(ord('L')):
        if render_controller.input.get_key_mods(ord('L')) & 1:
            move_anchor(0, 0, 0, 0.01, 0, 0)
        else:
            move_anchor(0, -0.05, 0, 0, 0, 0)
    if render_controller.input.get_key_state(ord('U')):
        if render_controller.input.get_key_mods(ord('U')) & 1:
            move_anchor(0, 0, 0, 0, 0, 0.05)
        else:
            move_anchor(0, 0, 0.05, 0, 0, 0)
    if render_controller.input.get_key_state(ord('O')):
        if render_controller.input.get_key_mods(ord('O')) & 1:
            move_anchor(0, 0, 0, 0, 0, -0.05)
        else:
            move_anchor(0, 0, -0.05, 0, 0, 0)

    scene.update_render()
    for i in range(4):
        scene.step()
    render_controller.render()

scene = None
