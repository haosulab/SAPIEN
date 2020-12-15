import sapien.core as sapien
import numpy as np


def main(filename):
    engine = sapien.Engine()
    renderer = sapien.VulkanRenderer()
    engine.set_renderer(renderer)

    config = sapien.SceneConfig()
    config.gravity = np.array([0, 0, 0])
    sim = engine.create_scene(config=config)
    sim.set_timestep(1 / 125)
    sim.set_ambient_light([.4, .4, .4])
    sim.set_shadow_light([1, -1, -1], [.5, .5, .5])
    sim.add_point_light([2, 2, 2], [1, 1, 1])
    sim.add_point_light([2, -2, 2], [1, 1, 1])
    sim.add_point_light([-2, 0, 2], [1, 1, 1])

    controller = sapien.VulkanController(renderer)
    controller.set_current_scene(sim)
    controller.set_free_camera_position(0, 1, 3)
    controller.set_free_camera_rotation(0, -0.8, 0)

    loader = sim.create_urdf_loader()
    loader.fix_root_link = True
    robot = loader.load(filename)

    print("Press q to quit......")

    while not controller.is_closed:
        sim.step()
        sim.update_render()
        controller.render()

    sim = None


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("filename", type=str, help="Filename of the urdf you would like load.")
    args = parser.parse_args()
