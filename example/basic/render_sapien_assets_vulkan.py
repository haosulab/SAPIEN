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
renderer = sapien.VulkanRenderer(True)
sim.set_renderer(renderer)
scene = sim.create_scene()
scene.set_timestep(1 / 60)

scene.set_ambient_light([0.5, 0.5, 0.5])
scene.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])
scene.add_point_light([1, 2, 2], [1, 1, 1])
scene.add_point_light([1, -2, 2], [1, 1, 1])
scene.add_point_light([-1, 0, 1], [1, 1, 1])

near, far = 0.1, 100
camera_mount_actor = scene.create_actor_builder().build(is_kinematic=True)
camera = scene.add_mounted_camera(
    "first_camera",
    camera_mount_actor,
    Pose(),
    640,
    480,
    np.deg2rad(35),
    np.deg2rad(35),
    near,
    far,
)

pos = np.array([-2, -2, 3])
forward = -pos / np.linalg.norm(pos)
left = np.cross([0, 0, 1], forward)
left = left / np.linalg.norm(left)
up = np.cross(forward, left)
mat44 = np.eye(4)
mat44[:3, :3] = np.array([forward, left, up]).T
mat44[:3, 3] = pos
camera_mount_actor.set_pose(Pose.from_transformation_matrix(mat44))

loader = scene.create_urdf_loader()
loader.fix_root_link = 1
asset = loader.load_kinematic(urdf)
assert asset, "No SAPIEN asset is loaded"

scene.step()
scene.update_render()
camera.take_picture()

rgba = camera.get_color_rgba()
rgba = (rgba * 255).clip(0, 255).astype("uint8")
rgba = Image.fromarray(rgba)
rgba.save("color.png")

depth = camera.get_depth()
y, x = np.where(depth < 1)
points = camera.get_position_rgba()[depth != 1]
points[..., 3] = 1
points[..., [0, 1, 2]] = points[..., [2, 0, 1]]
points[..., [0, 1]] *= -1
points = (mat44 @ points.T).T[:, :3]

cloud = open3d.geometry.PointCloud()
cloud.points = open3d.utility.Vector3dVector(points)
obj_segmentation = camera.get_obj_segmentation()[y, x]
color_map = (
    np.array([ImageColor.getrgb(color) for color in ImageColor.colormap.keys()]) / 255
)
cloud.colors = open3d.utility.Vector3dVector(color_map[obj_segmentation])

open3d.io.write_point_cloud("cloud.pcd", cloud)
open3d.visualization.draw_geometries([cloud])

scene = None
