import sapien.core as sapien
import numpy as np
from transforms3d.quaternions import axangle2quat as aa
from sapien.core import SceneConfig, Pose

sim = sapien.Engine()
renderer = sapien.SapienRenderer()
sim.set_renderer(renderer)

scene = sim.create_scene(config=SceneConfig())
scene.add_ground(0)
scene.set_timestep(1 / 100)

scene.set_ambient_light([0.5, 0.5, 0.5])
scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

mat = sim.create_physical_material(1, 1, 0.5)

loader = scene.create_urdf_loader()
loader.fix_root_link = 1
config = {
    "material": mat,
    "density": 100,
    "link": {
        "panda_leftfinger": {"patch_radius": 0.5},
        "panda_rightfinger": {"collision": {0: {"patch_radius": 0.4}}},
    },
}
robot = loader.load("../assets/robot/panda/panda.urdf", config)
model = robot.create_pinocchio_model()

targetPose = Pose([-0.01, -0.39, 0.96], [0.21, 0.73, 0.41, 0.5])
result, success, error = model.compute_inverse_kinematics(9, targetPose)
print("success: ", success)
print("error: ", error)
