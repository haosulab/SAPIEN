import sapyen
from tifffile import imwrite
from sapyen import Pose
import numpy as np
import os
import transforms3d
from PIL import Image
import sys

print(sys.argv[1], file=sys.stderr)


def rand_cam_pose(rlow, rhigh, plow, phigh, ylow=0, yhigh=2 * np.pi):
    yaw = np.random.rand() * (yhigh - ylow) + ylow
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


DIR = '/home/fx/source/partnet-mobility-scripts/mobility-v0-prealpha3/mobility_verified'
files = os.listdir(DIR)
rand_file = np.random.choice(files)
urdf = os.path.join(DIR, sys.argv[1], 'mobility.urdf')
print(urdf)

semantics = os.path.join(DIR, sys.argv[1], 'semantics.txt')
with open(semantics, 'r') as f:
    semantics=  dict((line.strip().split()) for line in f if line.strip())


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

id2name = dict(zip(wrapper.get_link_ids(), wrapper.get_link_names()))
id2semantic = [[i, semantics[id2name[i]]] for i in id2name if id2name[i] in semantics]

# pics = []
# for i in range(20):
#     mount.set_global_pose(rand_cam_pose(2, 4, 20 * np.pi / 180, 60 * np.pi / 180))
#     p = [rand_qpos(l, h) for l, h in limits]
#     wrapper.set_qpos(p)
#     sim.step()
#     sim.step()
#     sim.update_renderer()
#     cam0.take_picture()
#     pics.append(cam0.get_color_rgba())

mount.set_global_pose(
    rand_cam_pose(3, 3, 30 * np.pi / 180, 30 * np.pi / 180, 220 * np.pi / 180, 220 * np.pi / 180))
sim.update_renderer()
cam0 = renderer.get_camera(0)
cam0.take_picture()
Image.fromarray((cam0.get_color_rgba() * 255).astype(np.uint8)).save(f'images/{sys.argv[1]}_rgba.png')
Image.fromarray((cam0.get_normal_rgba() * 255).astype(np.uint8)).save(f'images/{sys.argv[1]}_normal.png')
depth = cam0.get_depth() 
imwrite(f'images/{sys.argv[1]}_depth.tif', depth, compress=6, photometric='minisblack')
seg = cam0.get_segmentation()
imwrite(f'images/{sys.argv[1]}_segmentation.tif', seg, compress=6, photometric='minisblack')


# renderer.show_window()
# while True:
#     sim.step()
#     sim.update_renderer()
#     renderer.render()
#     depth = cam0.get_depth()
