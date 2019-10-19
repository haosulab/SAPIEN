import open3d
import sapyen
from sapyen import Pose
import os
import numpy as np


def main():
    renderer = sapyen.OptifuserRenderer()
    renderer.cam.set_position([0.5, -4, 0.5])
    renderer.cam.set_forward([0, 1, 0])
    renderer.cam.set_up([0, 0, 1])

    sim = sapyen.Simulation()
    sim.set_renderer(renderer)
    sim.set_time_step(1.0 / 300)
    sim.add_ground(0, material=None)

    DIR = '/home/sim/project/mobility_verified'
    name2 = "46768/mobility.urdf"
    urdf = os.path.join(DIR, name2)
    print(urdf)

    loader = sim.create_urdf_loader()
    obj1 = loader.load(urdf)
    obj1.set_root_pose([3, 0, 0.5], [1, 0, 0, 0])
    loader.fix_loaded_object = True
    loader.balance_passive_force = True
    wrapper = loader.load('../assets/robot/all_robot.urdf')
    wrapper.set_qpos(np.zeros(13))



    builder = sim.create_actor_builder()
    actor = builder.build(False, True, "Camera Mount")
    cam = sim.add_mounted_camera("Floating Camera", actor, Pose([0, 0, 0], [1, 0, 0, 0]), 512, 424,
                                 1.22172944444, 1.0509561565825727, 0.01, 100)
    actor.set_global_pose(Pose([-2, 0, 3], [0.9238795, 0, 0.3826834, 0]))
    camera = renderer.get_camera(1)
    # extrinsic = camera.get_model_mat()
    camera_matrix = camera.get_camera_matrix()
    height = camera.get_height()
    width = camera.get_width()
    mapping_matrix = get_camera_mapping(height, width, camera_matrix)

    renderer.show_window()

    while True:
        # wrapper.set_qpos(np.random.random(13))
        sim.step()
        sim.update_renderer()
        renderer.render()
        camera.take_picture()
        depth = camera.get_depth()
        depth = 1 / (depth * (1 / 100 - 1 / 0.01) + 1 / 0.01)
        cloud = mapping_matrix * depth[:, :, np.newaxis]
        color = camera.get_color_rgba()[:,:,:3]
        pc = open3d.geometry.PointCloud()
        pc.points = open3d.utility.Vector3dVector(np.reshape(cloud, [-1, 3]))
        pc.colors = open3d.utility.Vector3dVector(np.reshape(color, [-1, 3]))
        open3d.visualization.draw_geometries([pc])


def get_camera_mapping(height: int, width: int, camera_matrix: np.ndarray):
    x = np.linspace(0.5, width - 0.5, width)
    y = np.linspace(0.5, height - 0.5, height)
    x, y = np.meshgrid(x, y)

    cor = np.stack([x.flatten(), y.flatten(), np.ones([x.size])], axis=0)
    mapping = np.linalg.inv(camera_matrix[:3, :3]) @ cor
    # mapping = mapping / np.linalg.norm(mapping, axis=0, keepdims=True)
    return np.reshape(mapping.T, [height, width, 3])


if __name__ == '__main__':
    main()
