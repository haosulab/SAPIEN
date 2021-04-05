"""Camera.

Concepts:
    - Create and mount cameras
    - Render RGB images, point clouds, segmentation masks
"""

import sapien.core as sapien
import numpy as np
from PIL import Image, ImageColor
import open3d as o3d


def main():
    engine = sapien.Engine()
    renderer = sapien.VulkanRenderer(offscreen_only=True)
    engine.set_renderer(renderer)

    scene = engine.create_scene()
    scene.set_timestep(1 / 100.0)

    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    urdf_path = '../assets/179/mobility.urdf'
    asset = loader.load_kinematic(urdf_path)  # load as a kinematic articulation
    assert asset, 'URDF not loaded.'

    scene.set_ambient_light([0.5, 0.5, 0.5])
    rscene = scene.get_render_scene()
    rscene.add_shadow_directional_light([0, 1, -1], [0.5, 0.5, 0.5])
    rscene.add_shadow_point_light([1, 2, 2], [1, 1, 1])
    rscene.add_shadow_point_light([1, -2, 2], [1, 1, 1])
    rscene.add_shadow_point_light([-1, 0, 1], [1, 1, 1])

    # ---------------------------------------------------------------------------- #
    # Camera
    # ---------------------------------------------------------------------------- #
    near, far = 0.1, 100
    width, height = 640, 480
    camera_mount_actor = scene.create_actor_builder().build(is_kinematic=True)
    camera = scene.add_mounted_camera(
        name="camera",
        actor=camera_mount_actor,
        pose=sapien.Pose(),  # relative to the mounted actor
        width=width,
        height=height,
        fovx=np.deg2rad(35),
        fovy=np.deg2rad(35),
        near=near,
        far=far,
    )
    print('Intrinsic matrix\n', camera.get_camera_matrix())

    # Compute the camera pose by specifying forward(x), left(y) and up(z)
    cam_pos = np.array([-2, -2, 3])
    forward = -cam_pos / np.linalg.norm(cam_pos)
    left = np.cross([0, 0, 1], forward)
    left = left / np.linalg.norm(left)
    up = np.cross(forward, left)
    mat44 = np.eye(4)
    mat44[:3, :3] = np.stack([forward, left, up], axis=1)
    mat44[:3, 3] = cam_pos
    camera_mount_actor.set_pose(sapien.Pose.from_transformation_matrix(mat44))

    scene.step()  # make everything set
    scene.update_render()
    camera.take_picture()

    # ---------------------------------------------------------------------------- #
    # RGBA
    # ---------------------------------------------------------------------------- #
    rgba = camera.get_float_texture('Color')  # [H, W, 4]
    rgba_img = (rgba * 255).clip(0, 255).astype("uint8")
    rgba_pil = Image.fromarray(rgba_img)
    rgba_pil.save('color.png')

    # ---------------------------------------------------------------------------- #
    # XYZ position in the camera space
    # ---------------------------------------------------------------------------- #
    # Each pixel is (x, y, z, is_valid) in camera space (OpenGL/Blender)
    position = camera.get_float_texture('Position')  # [H, W, 4]

    # OpenGL/Blender: y up and -z forward
    points_opengl = position[..., :3][position[..., 3] > 0]
    points_color = rgba[position[..., 3] > 0][..., :3]
    # Model matrix is the transformation from OpenGL camera space to SAPIEN world space
    # camera.get_model_matrix() must be called after scene.update_render()!
    model_matrix = camera.get_model_matrix()
    points_world = points_opengl @ model_matrix[:3, :3].T + model_matrix[:3, 3]
    
    # SAPIEN CAMERA: z up and x forward
    # points_camera = points_opengl[..., [2, 0, 1]] * [-1, -1, 1]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_world)
    pcd.colors = o3d.utility.Vector3dVector(points_color)
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
    o3d.visualization.draw_geometries([pcd, coord_frame])

    # Depth
    depth = -position[..., 2]
    depth_image = (depth * 1000.0).astype(np.uint16)
    depth_pil = Image.fromarray(depth_image)
    depth_pil.save('depth.png')

    # ---------------------------------------------------------------------------- #
    # Segmentation labels
    # ---------------------------------------------------------------------------- #
    # Each pixel is (visual_id, actor_id/link_id, 0, 0)
    # visual_id is the unique id of each visual shape
    seg_labels = camera.get_uint32_texture('Segmentation')  # [H, W, 4]
    color_palette = np.array([ImageColor.getrgb(color)
                             for color in ImageColor.colormap.keys()], 
                             dtype=np.uint8)
    label0_image = seg_labels[..., 0].astype(np.uint8)
    label1_image = seg_labels[..., 1].astype(np.uint8)
    label0_pil = Image.fromarray(color_palette[label0_image])
    label0_pil.save('label0.png')
    label1_pil = Image.fromarray(color_palette[label1_image])
    label1_pil.save('label1.png')


if __name__ == '__main__':
    main()
