import sapien.core as sapien
from sapien.core import Pose
import numpy as np
from PIL import Image, ImageColor
import open3d
from sapien.asset import download_partnet_mobility

my_token = "Your Access Token"
sapien_assets_id = 179
urdf = download_partnet_mobility(sapien_assets_id, token=my_token)

sim = sapien.Engine()
renderer = sapien.VulkanRenderer(False)
sim.set_renderer(renderer)
scene = sim.create_scene()
scene.set_timestep(1 / 60)

scene.set_ambient_light([0.5, 0.5, 0.5])
scene.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])
scene.add_point_light([1, 2, 2], [1, 1, 1])
scene.add_point_light([1, -2, 2], [1, 1, 1])
scene.add_point_light([-1, 0, 1], [1, 1, 1])

controller = sapien.VulkanController(renderer)
controller.set_current_scene(scene)
controller.set_free_camera_position(-3, 0, 0)

loader = scene.create_urdf_loader()
loader.fix_root_link = 1
asset = loader.load_kinematic(urdf)
assert asset, "No SAPIEN asset is loaded"

while not controller.is_closed:
    scene.step()
    scene.update_render()
    controller.render()

controller = None
scene = None
