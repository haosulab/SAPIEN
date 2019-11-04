import sapyen
from tifffile import imwrite
from sapyen import Pose
import numpy as np
import os
import transforms3d
from PIL import Image
import sys

def rand_cam_pose(rlow, rhigh, plow, phigh, ylow=0, yhigh=360):
    plow *= np.pi / 180
    phigh *= np.pi / 180
    ylow *= np.pi / 180
    yhigh *= np.pi / 180

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


urdf = '/home/fx/source/partnet-mobility-scripts/mobility_v1_alpha3/45940/mobility.urdf'
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


builder = sim.create_actor_builder()
mount = builder.build(False, True, "Camera Mount")
fov = 1.22172944444
near = 0.01
far = 100
width = 512
height = 512
cam = sim.add_mounted_camera("Floating Camera", mount, Pose([0, 0, 0], [1, 0, 0, 0]), width, height, fov, fov,
                             near, far)
mount.set_global_pose(rand_cam_pose(2, 2, 20, 20, 200, 200))

cam0 = renderer.get_camera(0)
loader = sim.create_urdf_loader()
wrapper = loader.load(urdf)

print(sim.get_render_name_dict())

colormap = np.random.rand(1000, 3)

sim.step()
sim.update_renderer()
cam0.take_picture()
seg = cam0.get_obj_segmentation()
import matplotlib.pyplot as plt
plt.imshow(colormap[seg])
plt.show()
