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
scene.add_ground(0)
scene.set_timestep(1 / 60)
render_controller.set_current_scene(scene)

scene.set_ambient_light([0.5, 0.5, 0.5])
scene.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

render_controller.set_camera_position(-4, 0, 1)

loader = scene.create_urdf_loader()
loader.fix_root_link = 1
loader.collision_is_visual = 1
robot = loader.load("../assets/robot/bullet_human.urdf")
robot.set_root_pose(Pose([0, 0, 2]))

while not render_controller.should_quit:

    scene.update_render()
    scene.step()
    render_controller.render()

scene = None
