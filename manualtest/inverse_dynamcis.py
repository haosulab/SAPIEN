import sapien.core as pysapien
import sapien
import numpy as np

sim = pysapien.Engine()
renderer = pysapien.OptifuserRenderer()
sim.set_renderer(renderer)
render_controller = pysapien.OptifuserController(renderer)

render_controller.show_window()

s0 = sim.create_scene(gravity=[0, 0, -9.8])
s0.add_ground(-1)
s0.set_timestep(1 / 240)

s0.set_ambient_light([0.5, 0.5, 0.5])
s0.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

loader: pysapien.URDFLoader = s0.create_urdf_loader()
loader.fix_root_link = 1
urdf = sapien.asset.download_partnet_mobility(45092,
                                              "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJlbWFpbCI6InkxcWluQGVuZy51Y3NkLmVkdSIsImlwIjoiNjYuNzUuMjQwLjYwIiwicHJpdmlsZWdlIjoxMCwiaWF0IjoxNTg2NzM0NDQxLCJleHAiOjE2MTgyNzA0NDF9.5Q_eJyzP_4Jf7wxTa068sB8q666AJgcFE2ojf3UsHkc")
robot = loader.load(urdf)

render_controller.set_camera_position(-5, 0, 0)
render_controller.set_current_scene(s0)

steps = 0
while not render_controller.should_quit:
    s0.update_render()
    for i in range(1):
        s0.step()
        steps += 1
    render_controller.render()

    if steps == 100:
        qf = np.ones([robot.dof]) * 1
        robot.set_qf(qf)
        s0.step()
        qacc = robot.get_qacc()
        active_force = robot.compute_inverse_dynamics(qacc)
        passive_force = robot.compute_passive_force(external=False)
        total_force = active_force + passive_force
        print(total_force)
        print(qf)
        print(np.allclose(qf, total_force))

    if steps == 200:
        target_qacc = np.ones(robot.dof) * 0.1
        active_force = robot.compute_inverse_dynamics(target_qacc)
        passive_force = robot.compute_passive_force()
        robot.set_qf(active_force + passive_force)
        s0.step()
        qacc = robot.get_qacc()
        print(qacc)
        print(target_qacc)
        print(np.allclose(qacc, target_qacc))

s0 = None
