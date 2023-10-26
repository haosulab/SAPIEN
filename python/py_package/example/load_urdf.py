import sapien as sapien
import numpy as np
from sapien.utils import Viewer


def main(filename):
    config = sapien.physx.PhysxSceneConfig()
    config.gravity = np.array([0, 0, 0])

    scene = sapien.Scene(
        [sapien.physx.PhysxSystem(config), sapien.render.RenderSystem()]
    )
    scene.set_timestep(1 / 125)
    scene.set_ambient_light([0.4, 0.4, 0.4])
    scene.add_directional_light([1, -1, -1], [0.5, 0.5, 0.5])
    scene.add_point_light([2, 2, 2], [1, 1, 1])
    scene.add_point_light([2, -2, 2], [1, 1, 1])
    scene.add_point_light([-2, 0, 2], [1, 1, 1])

    viewer = scene.create_viewer()
    viewer.set_camera_xyz(-1, 0, 1)
    viewer.set_camera_rpy(0, -0.8, 0)

    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    robot = loader.load(filename)

    while not viewer.closed:
        scene.step()
        scene.update_render()
        viewer.render()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "filename", type=str, help="Filename of the urdf you would like load."
    )
    args = parser.parse_args()
    main(args.filename)
