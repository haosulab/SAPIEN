import pysapien
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

builder = s0.create_actor_builder()
builder.add_box_shape()
builder.add_box_visual()
actor = builder.build()

actor.set_pose(Pose([0, 0, 2]))
controller.camera.set_position([-5, 0, 0])

controller.set_current_scene(s0)

while not controller.should_quit:
    s0.update_render()
    s0.step()
    controller.render()
