"""Camera.

Concepts:
    - Create and mount cameras
    - Render RGB images, point clouds, segmentation masks
"""

import sapien.core as sapien
import numpy as np
from PIL import Image, ImageColor
import open3d as o3d
from sapien.utils.viewer import Viewer
from transforms3d.euler import mat2euler


def main():
    engine = sapien.Engine()

    config = sapien.KuafuConfig()
    # config.spp = 64
    # config.use_denoiser = True
    renderer = sapien.KuafuRenderer(config)
    engine.set_renderer(renderer)

    scene = engine.create_scene()
    scene.set_timestep(1 / 100.0)

    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    urdf_path = '../assets/179/mobility.urdf'
    # load as a kinematic articulation
    asset = loader.load_kinematic(urdf_path)
    assert asset, 'URDF not loaded.'


    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)
    scene.add_point_light([1, 2, 2], [1, 1, 1], shadow=True)
    scene.add_point_light([1, -2, 2], [1, 1, 1], shadow=True)
    scene.add_point_light([-1, 0, 1], [1, 1, 1], shadow=True)

    # ---------------------------------------------------------------------------- #
    # Camera
    # ---------------------------------------------------------------------------- #
    near, far = 0.1, 100
    width, height = 640, 480
    camera_mount_actor = scene.create_actor_builder().build_kinematic()
    camera = scene.add_mounted_camera(
        name="camera",
        actor=camera_mount_actor,
        pose=sapien.Pose(),  # relative to the mounted actor
        width=width,
        height=height,
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
    # An alias is also provided
    # rgba = camera.get_color_rgba()  # [H, W, 4]
    rgba_img = (rgba * 255).clip(0, 255).astype("uint8")
    rgba_pil = Image.fromarray(rgba_img)
    rgba_pil.save('kuafu_color.png')

if __name__ == '__main__':
    main()
