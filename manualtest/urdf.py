import pysapien
import numpy as np
from transforms3d.quaternions import axangle2quat as aa

sim = pysapien.Engine()
renderer = pysapien.OptifuserRenderer()
sim.set_renderer(renderer)
render_controller = pysapien.OptifuserController(renderer)

render_controller.show_window()

s0 = sim.create_scene()
s0.add_ground(-1)
s0.set_timestep(1 / 240)

s0.set_ambient_light([0.5, 0.5, 0.5])
s0.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

loader: pysapien.URDFLoader = s0.create_urdf_loader()
loader.fix_root_link = 0
chair = loader.load_kinematic("assets/robot/locobot_description.urdf")

render_controller.set_camera_position(-5, 0, 0)
render_controller.set_current_scene(s0)

steps = 0
while not render_controller.should_quit:
    s0.update_render()
    for i in range(4):
        s0.step()
        steps += 1
    render_controller.render()

    # if steps > 10000:
    #     for link in chair.get_base_links():
    #         name = link.get_name()
    #         pose = link.get_pose()
    #         print(name, pose)
    #     break

s0 = None
