import pysapien

sim = pysapien.Simulation()
renderer = pysapien.OptifuserRenderer()
sim.set_renderer(renderer)
controller = pysapien.OptifuserController(renderer)

controller.show_window()

s0 = sim.create_scene("Scene 1", [0, 0, -9.8], pysapien.SolverType.PGS, False, False)
s0.add_ground(-1)
s0.set_timestep(1 / 60)

s0.set_ambient_light([0.5, 0.5, 0.5])

controller.set_current_scene(s0)

while not controller.should_quit:
    s0.update_render()
    s0.step()
    controller.render()
