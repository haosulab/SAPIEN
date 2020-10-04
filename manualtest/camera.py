import pysapien
import numpy as np
from pysapien import Pose
from transforms3d.quaternions import axangle2quat as aa
import matplotlib.pyplot as plt

sim = pysapien.Simulation()
renderer = pysapien.OptifuserRenderer()
sim.set_renderer(renderer)

s0 = sim.create_scene()
s0.add_ground(-1)
s0.set_timestep(1 / 240)

s0.set_ambient_light([0.5, 0.5, 0.5])
s0.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

loader = s0.create_urdf_loader()
loader.fix_base = 0
chair = loader.load("../assets/robot/all_robot.urdf")

cam = s0.find_mounted_camera("kinect2_ir_sensor")

for i in range(4):
    s0.step()
s0.update_render()

cam.take_picture()
img = cam.get_color_rgba()
print(img.shape)
plt.imshow(img)
plt.show()

s0 = None
