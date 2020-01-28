import pysapien
import numpy as np
from transforms3d.quaternions import axangle2quat as aa


sim = pysapien.Simulation()
renderer = pysapien.OptifuserRenderer()
sim.set_renderer(renderer)
render_controller = pysapien.OptifuserController(renderer)

render_controller.show_window()

s0 = sim.create_scene()
s0.add_ground(-1)
s0.set_timestep(1 / 240)

s0.set_ambient_light([0.5, 0.5, 0.5])
s0.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

loader = s0.create_urdf_loader()
loader.fix_base = 0
chair = loader.load("../assets/robot/all_robot.urdf")

render_controller.camera.set_position([-5, 0, 0])
render_controller.set_current_scene(s0)

while not render_controller.should_quit:
    s0.update_render()
    for i in range(4):
        s0.step()
    render_controller.render()

s0 = None
