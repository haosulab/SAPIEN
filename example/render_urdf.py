import sapien.core as sapien
import os
import transforms3d
from sapien.core import Pose
import numpy as np
from PIL import Image
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('urdf', type=str)
args = parser.parse_args()

sim = sapien.Simulation()
renderer = sapien.OptifuserRenderer()
sim.set_renderer(renderer)

scene: sapien.Scene = sim.create_scene()
scene.set_timestep(1 / 60)

scene.set_ambient_light([0.5, 0.5, 0.5])
scene.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])
scene.add_point_light([1, 2, 2], [1, 1, 1])
scene.add_point_light([1, -2, 2], [1, 1, 1])
scene.add_point_light([-1, 0, 1], [1, 1, 1])

cam_mount: sapien.Actor = scene.create_actor_builder().build(is_kinematic=True)
camera: sapien.ICamera = scene.add_mounted_camera('', cam_mount, Pose(), 256, 256, np.deg2rad(35),
                                                  np.deg2rad(35), 0.1, 100)

x = -4
y = -4
z = 3

p = np.array([x, y, z])

forward = -p / np.linalg.norm(p)
left = np.cross([0, 0, 1], forward)
left = left / np.linalg.norm(left)
up = np.cross(forward, left)

quat = transforms3d.quaternions.mat2quat(np.linalg.inv(np.array([forward, left, up])))

cam_mount.set_pose(Pose(p, quat))

loader = scene.create_urdf_loader()
loader.fix_root_link = 1
robot: sapien.KinematicArticulation = loader.load_kinematic(args.urdf)

joints = robot.get_base_joints()

print('joint information')
for j in joints:
    l: sapien.KinematicLink = j.get_parent_link()
    print(f'"{j.name}"', j.get_limits())


if not robot:
    exit(1)

# update world
scene.step()
# update render
scene.update_render()

camera.take_picture()
img = camera.get_color_rgba()

img = (img * 255).clip(0, 255).astype('uint8')
img = Image.fromarray(img)
img.save('output.png')

scene.remove_kinematic_articulation(robot)

scene = None


# 179, 2440, 723
