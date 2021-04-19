import sapien.core as sapien
import transforms3d
from sapien.core import Pose
import numpy as np

sim = sapien.Engine()
renderer = sapien.OptifuserRenderer()
sim.set_renderer(renderer)
render_controller = sapien.OptifuserController(renderer)

render_controller.show_window()

scene: sapien.Scene = sim.create_scene()
scene.set_timestep(1 / 500)

scene.set_ambient_light([0.5, 0.5, 0.5])
scene.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

loader = scene.create_urdf_loader()
loader.fix_root_link = 1
robot = loader.load("assets/robot/jaco2/jaco2.urdf")

actor_builder: sapien.ActorBuilder = scene.create_actor_builder()
actor_builder.add_sphere_visual(Pose(), 0.02, [1, 0, 0], "anchor")
anchor = actor_builder.build_static()
anchor.set_pose(Pose([0.2, 0, 0.2]))

render_controller.set_camera_position(-0.5, 2, 0.5)
render_controller.set_camera_rotation(-1, 0)
render_controller.set_current_scene(scene)

gripper = [l for l in robot.get_links() if l.name == 'j2s7s300_end_effector'][0]

def move_anchor(dx, dy, dz, droll, dpitch, dyaw):
    quat = transforms3d.euler.euler2quat(droll, dpitch, dyaw)
    anchor.set_pose(Pose([dx, dy, dz]) * anchor.pose * Pose([0, 0, 0], quat))


drive = scene.create_drive(anchor, Pose(), gripper, Pose())
drive.set_properties(4000, 8000)
drive.set_target(Pose())

arm_init_qpos = [4.71, 2.84, 0, 0.75, 4.62, 4.48, 4.88]
gripper_init_qpos = [0.] * 2
init_qpos = arm_init_qpos + gripper_init_qpos
print(robot.dof)
robot.set_qpos(init_qpos)
robot.set_qvel([0] * 9)

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
    for i in range(1):
        scene.step()
    render_controller.render()

scene = None
