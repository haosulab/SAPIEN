import sapyen
from sapyen import Pose
import numpy as np
import os

DIR = '/home/fx/source/partnet-mobility-scripts/mobility_verified'
files = os.listdir(DIR)
urdf = os.path.join(DIR, np.random.choice(files), 'mobility.urdf')
urdf = os.path.join(DIR, '22301', 'mobility.urdf')
print(urdf)

renderer = sapyen.OptifuserRenderer()
renderer.cam.set_position(np.array([0, -2, 1]))
renderer.cam.rotate_yaw_pitch(0, -0.5)

sim = sapyen.Simulation()
sim.set_renderer(renderer)
sim.set_time_step(1.0 / 200.0)

loader = sim.create_urdf_loader()
wrapper = loader.load(urdf)

builder = sim.create_actor_builder()
actor = builder.build(False, True, "Camera Mount")
cam = sim.add_mounted_camera("Floating Camera", actor, Pose([0, 0, 0], [1, 0, 0, 0]), 512, 424,
                             1.22172944444, 1.0509561565825727, 0.01, 100)
actor.set_global_pose(Pose([-2, 0, 2], [0.9238795, 0, 0.3826834, 0]))

zeros = np.zeros(wrapper.dof())
wrapper.set_qpos(zeros)
wrapper.set_qvel(zeros)
wrapper.set_qf(zeros)

limits = wrapper.get_joint_limits()
print(limits)

sim.update_renderer()
# cam0 = renderer.get_camera(0)
# cam0.take_picture()
# color = cam0.get_color_rgba()
# depth = cam0.get_depth()
# seg = cam0.get_segmentation()

renderer.show_window()
while True:
    sim.step()
    sim.update_renderer()
    renderer.render()
