import open3d
import transforms3d
import sapyen
from sapyen import Pose
import os
import numpy as np


def rand_cam_pose(rlow, rhigh, plow, phigh, ylow=0, yhigh=360):
    plow *= np.pi / 180
    phigh *= np.pi / 180
    ylow *= np.pi / 180
    yhigh *= np.pi / 180

    yaw = np.random.rand() * (yhigh - ylow) + ylow
    pitch = np.random.rand() * (phigh - plow) + plow
    r = np.random.rand() * (rhigh - rlow) + rlow

    x = r * np.cos(pitch) * np.cos(yaw)
    y = r * np.cos(pitch) * np.sin(yaw)
    z = r * np.sin(pitch)

    x2 = -np.array([x, y, z])
    x2 /= np.linalg.norm(x2)

    y2 = np.cross(np.array([0, 0, 1]), x2)
    y2 /= np.linalg.norm(y2)

    z2 = np.cross(x2, y2)

    quat = transforms3d.quaternions.mat2quat(np.array([x2, y2, z2]).T)
    return Pose([x, y, z], quat)


renderer = sapyen.OptifuserRenderer()
renderer.cam.set_position(np.array([0, -2, 1]))
renderer.cam.rotate_yaw_pitch(0, -0.5)

renderer.set_ambient_light([.4, .4, .4])
renderer.set_shadow_light([1, -1, -1], [.5, .5, .5])
renderer.add_point_light([2, 2, 2], [1, 1, 1])
renderer.add_point_light([2, -2, 2], [1, 1, 1])
renderer.add_point_light([-2, 0, 2], [1, 1, 1])

sim = sapyen.Simulation()
sim.set_renderer(renderer)
sim.set_time_step(1.0 / 200.0)


def add_camera(name, pose):

    builder = sim.create_actor_builder()
    mount = builder.build(False, True, "Camera Mount")

    fov = 1.22172944444
    near = 0.01
    far = 100
    width = 512
    height = 512
    sim.add_mounted_camera(name, mount, Pose([0, 0, 0], [1, 0, 0, 0]), width, height, fov, fov, near, far)
    mount.set_global_pose(pose)
    return renderer.get_camera(renderer.get_camera_count() - 1)


cam0 = add_camera('cam0', rand_cam_pose(2, 2, -40, 40, 30, 30))
cam1 = add_camera('cam1', rand_cam_pose(2, 2, -40, 40, 60, 60))
cam2 = add_camera('cam2', rand_cam_pose(2, 2, -40, 40, 205, 205))
cam3 = add_camera('cam3', rand_cam_pose(2, 2, -40, 40, 310, 310))

urdf = '../assets/45940/mobility.urdf'
loader = sim.create_urdf_loader()
wrapper = loader.load(urdf)

sim.step()
sim.update_renderer()


def point_cloud_from_depth(depth, color, proj, model):
    W, H = depth.shape

    WS = np.repeat(np.linspace(1 / (2 * W), 1 - 1 / (2 * W), W).reshape([1, -1]), H, axis=0)
    HS = np.repeat(np.linspace(1 / (2 * H), 1 - 1 / (2 * H), H)[::-1].reshape([-1, 1]), W, axis=1)
    points = np.stack([WS, HS, depth, np.ones_like(depth)], 2)

    color = color[depth < 1]
    points = points[depth < 1]
    points = points * 2 - 1
    cam_points = np.linalg.inv(proj) @ points.T
    cam_points /= cam_points[3]

    world_points = model @ cam_points
    world_points = world_points.T
    print(model)

    return world_points[:, :3], color[:, :3]


def get_pc(cam):
    cam.take_picture()
    depth = cam.get_depth()
    color = cam.get_color_rgba()

    xyz, rgb = point_cloud_from_depth(depth, color, cam.get_projection_mat(), cam.get_model_mat())
    pc = open3d.geometry.PointCloud()
    pc.points = open3d.utility.Vector3dVector(xyz)
    pc.colors = open3d.utility.Vector3dVector(rgb)
    return pc


np.set_printoptions(3, suppress=True)

pc0 = get_pc(cam0)
pc1 = get_pc(cam1)
pc2 = get_pc(cam2)
pc3 = get_pc(cam3)

mesh_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])

open3d.visualization.draw_geometries([mesh_frame, pc0, pc1, pc2, pc3])
