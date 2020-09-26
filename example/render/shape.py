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

loader = scene.create_urdf_loader()
loader.fix_root_link = 1
asset = loader.load_kinematic(urdf)
assert asset, "No SAPIEN asset is loaded"

scene.step()
scene.update_render()

meshes = []
for link in asset.get_base_links():
    visual_bodies = link.get_visual_bodies()
    for v in visual_bodies:
        shapes = v.get_render_shapes()
        for s in shapes:
            pose = link.pose * s.pose
            scale = s.scale
            T = pose.to_transformation_matrix()
            T = T @ np.diag(list(scale) + [1])

            vertices = s.mesh.vertices
            indices = s.mesh.indices

            mesh = open3d.geometry.TriangleMesh(
                open3d.utility.Vector3dVector(vertices),
                open3d.utility.Vector3iVector(indices.reshape(-1, 3)),
            )
            mesh.transform(T)
            meshes.append(mesh)
open3d.visualization.draw_geometries(meshes)

scene = None
