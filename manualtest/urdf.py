import sapien.core as sapien
import numpy as np
from transforms3d.quaternions import axangle2quat as aa
from sapien.core import SceneConfig, Pose

sim = sapien.Engine()
renderer = sapien.OptifuserRenderer()
sim.set_renderer(renderer)
render_controller = sapien.OptifuserController(renderer)

scene = sim.create_scene(config=SceneConfig())
scene.add_ground(0)
scene.set_timestep(1 / 100)

scene.set_ambient_light([0.5, 0.5, 0.5])
scene.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

loader = scene.create_urdf_loader()
loader.fix_root_link = 1
robot = loader.load_kinematic("../assets_local/robot/panda.urdf")
model = robot.create_pinocchio_model()

targetPose = Pose([-0.01, -0.39, 0.96], [0.21, 0.73, 0.41, 0.5])
result, success, error = model.compute_inverse_kinematics(9, targetPose)
print('success: ', success)
print('error: ', error)

robot.set_qpos(result)

render_controller.show_window()
render_controller.set_camera_position(-5, 0, 0)
render_controller.set_current_scene(scene)

steps = 0
while not render_controller.should_quit:
    scene.update_render()
    for i in range(4):
        scene.step()
        steps += 1
    render_controller.render()
# scene = None
