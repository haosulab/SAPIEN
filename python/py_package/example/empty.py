import sapien
from sapien.utils import Viewer


def main():
    scene = sapien.Scene()
    scene.add_ground(0, render_half_size=[2, 2])

    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], True)

    viewer = Viewer()
    viewer.set_scene(scene)
    viewer.set_camera_xyz(-4, 0, 1)

    while not viewer.closed:
        scene.step()
        scene.update_render()
        viewer.render()


if __name__ == "__main__":
    main()
