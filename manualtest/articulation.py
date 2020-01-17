import pysapien
import numpy as np
from pysapien import Pose

sim = pysapien.Simulation()
renderer = pysapien.OptifuserRenderer()
sim.set_renderer(renderer)
controller = pysapien.OptifuserController(renderer)

controller.show_window()

s0 = sim.create_scene("Scene 1", [0, 0, -9.8], pysapien.SolverType.PGS, False, False)
s0.add_ground(-1)
s0.set_timestep(1 / 60)

s0.set_ambient_light([0.5, 0.5, 0.5])
s0.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

ab = s0.create_articulation_builder()
body = ab.create_link_builder()
body.add_box_shape()
body.add_box_visual()
body.set_name("body")

leg = ab.create_link_builder(body)
leg.add_capsule_shape()
leg.add_capsule_visual()
leg.set_name("leg")
leg.set_joint_properties(pysapien.ArticulationJointType.REVOLUTE, [[-np.inf, np.inf]], Pose(), Pose(), 0.1, 0)
leg.set_joint_name("l_joint")

a = ab.build()

controller.camera.set_position([-5, 0, 0])
controller.set_current_scene(s0)

while not controller.should_quit:
    s0.update_render()
    s0.step()
    controller.render()

s0 = None
