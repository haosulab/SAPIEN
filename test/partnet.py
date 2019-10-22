import sapyen
from sapyen import Pose
import numpy as np
import os
import transforms3d


def rand_cam_pose(rlow, rhigh, plow, phigh):
    yaw = np.random.rand() * np.pi * 2
    pitch = np.random.rand() * (phigh - plow) + plow
    r = np.random.rand() * (rhigh - rlow) + rlow

    x = r * np.cos(pitch) * np.cos(yaw)
    y = r * np.cos(pitch) * np.sin(yaw)
    z = r * np.sin(pitch)

    x2 = -np.array([x, y, z])
    x2 /= np.linalg.norm(x2)

    y2 = np.cross(np.array([0, 0, 1]), x2)
    y2 /= np.linalg.norm(y2)

    z2 = np.cross(x2, y2)

    quat = transforms3d.quaternions.mat2quat(np.array([x2, y2, z2]).T)
    return Pose([x, y, z], quat)


def rand_qpos(low, high):
    return np.random.rand() * (high - low) + low


DIR = '/home/fx/source/partnet-mobility-scripts/mobility_verified'
files = os.listdir(DIR)
rand_file = np.random.choice(files)
urdf = os.path.join(DIR, rand_file, 'mobility.urdf')
cues = os.path.join(DIR, rand_file, 'cues.txt')
with open(cues, 'r') as f:
    cues = f.readlines()
print(urdf)

renderer = sapyen.OptifuserRenderer()
renderer.cam.set_position(np.array([0, -2, 1]))
renderer.cam.rotate_yaw_pitch(0, -0.5)

renderer.set_ambient_light([.4, .4, .4])
renderer.set_shadow_light([1, -1, -1], [.5, .5, .5])
renderer.add_point_light([2, 2, 2], [1, 1, 1])
renderer.add_point_light([2, -2, 2], [1, 1, 1])
renderer.add_point_light([-2, 0, 2], [1, 1, 1])

sim = sapyen.Simulation()
sim.set_renderer(renderer)
sim.set_time_step(1.0 / 200.0)

loader = sim.create_urdf_loader()
wrapper = loader.load(urdf)
print(wrapper.get_link_names())
print(wrapper.get_link_ids())
print(cues)

builder = sim.create_actor_builder()
mount = builder.build(False, True, "Camera Mount")
cam = sim.add_mounted_camera("Floating Camera", mount, Pose([0, 0, 0], [1, 0, 0, 0]), 512, 512, 1.22172944444,
                             1.22172944444, 0.01, 100)

zeros = np.zeros(wrapper.dof())
wrapper.set_qpos(zeros)
wrapper.set_qvel(zeros)
wrapper.set_qf(zeros)

limits = wrapper.get_joint_limits()
limits = [[max(-100, l), min(100, h)] for l, h in limits]
cam0 = renderer.get_camera(0)

pics = []
for i in range(20):
    mount.set_global_pose(rand_cam_pose(2, 4, 20 * np.pi / 180, 60 * np.pi / 180))
    p = [rand_qpos(l, h) for l, h in limits]
    wrapper.set_qpos(p)
    sim.step()
    sim.step()
    sim.update_renderer()
    cam0.take_picture()
    pics.append(cam0.get_color_rgba())

# import matplotlib.pyplot as plt
# for i, p in enumerate(pics):
#     plt.figure()
#     plt.imshow(p)
#     plt.savefig('{}.png'.format(i))

sim.update_renderer()
cam0 = renderer.get_camera(0)
cam0.take_picture()

renderer.show_window()
while True:
    sim.step()
    sim.update_renderer()
    renderer.render()
    depth = cam0.get_depth()
