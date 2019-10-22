import numpy as np
import copy
import warnings
import sapyen
import open3d
import transforms3d
import os
from robot.python.demonstration.recorder import PARTNET_DIR

RGBD_CAMERA_THRESHOLD = 10


class Replayer:
    def __init__(self, partnet_id: str):
        # Rendering
        self.renderer = sapyen.OptifuserRenderer()
        self.renderer.set_ambient_light([.4, .4, .4])
        self.renderer.set_shadow_light([1, -1, -1], [.5, .5, .5])
        self.renderer.add_point_light([2, 2, 2], [1, 1, 1])
        self.renderer.add_point_light([2, -2, 2], [1, 1, 1])
        self.renderer.add_point_light([-2, 0, 2], [1, 1, 1])
        self.renderer.cam.set_position([0.5, -4, 0.5])
        self.renderer.cam.set_forward([0, 1, 0])
        self.renderer.cam.set_up([0, 0, 1])

        # Simulation
        self.sim = sapyen.Simulation()
        self.sim.set_renderer(self.renderer)
        self.sim.set_time_step(1.0 / 200)
        self.sim.add_ground(0, material=None)
        self.builder = self.sim.create_actor_builder()

        # Partnet Object
        urdf = os.path.join(PARTNET_DIR, partnet_id, "mobility.urdf")
        self.loader = self.sim.create_urdf_loader()
        self.obj = self.loader.load(urdf)
        self.obj.set_root_pose([3, 0, 0.5], [1, 0, 0, 0])

        # Robot Model and controller manger
        self.loader.fix_loaded_object = True
        self.loader.balance_passive_force = True
        self.robot = self.loader.load('../assets/robot/all_robot.urdf')

        self.camera_name_list = []
        self.cam_list = []
        self.mount_actor_list = []
        self.mapping_list = []
        self.depth_lambda_list = []

        self.simulation_steps = 0
        self.data = np.load("data/{}_v0.p".format(partnet_id), allow_pickle=True)
        self.init_camera()

    def init_camera(self):
        num = self.renderer.get_camera_count()
        for i in range(num):
            camera = self.renderer.get_camera(i)
            height, width = camera.get_height(), camera.get_width()
            self.mount_actor_list.append(None)
            self.camera_name_list.append(camera.get_name())
            self.cam_list.append(camera)
            self.build_camera_mapping(height, width, camera.get_camera_matrix())
            self.depth_lambda_list.append(
                lambda depth: 1 / (depth * (1 / camera.far - 1 / camera.near) + 1 / camera.near))

    def add_camera(self, name, camera_pose: np.ndarray, width: int, height: int, fov=1.1, near=0.01, far=100):
        actor = self.builder.build(False, True, "{}".format(name), True)
        self.mount_actor_list.append(actor)
        self.camera_name_list.append(name)

        pose = sapyen.Pose(camera_pose[:3, 3])
        pose.set_q(transforms3d.quaternions.mat2quat(camera_pose[:3, :3]))
        self.sim.add_mounted_camera(name, actor, sapyen.Pose([0, 0, 0], [1, 0, 0, 0]), width, height, fov, fov,
                                    near, far)
        actor.set_global_pose(pose)

        camera = self.renderer.get_camera(len(self.cam_list))
        self.cam_list.append(camera)
        self.build_camera_mapping(height, width, camera.get_camera_matrix())
        self.depth_lambda_list.append(lambda depth: 1 / (depth * (1 / far - 1 / near) + 1 / near))

    def build_camera_mapping(self, height: int, width: int, camera_matrix: np.ndarray):
        x = np.linspace(0.5, width - 0.5, width)
        y = np.linspace(0.5, height - 0.5, height)
        x, y = np.meshgrid(x, y)
        cor = np.stack([x.flatten(), y.flatten(), np.ones([x.size])], axis=0)
        mapping = np.linalg.inv(camera_matrix[:3, :3]) @ cor
        self.mapping_list.append(np.reshape(mapping.T, [height, width, 3]))

    @property
    def mounted_camera(self):
        return self.camera_name_list.copy()

    def get_camera_id(self, name: str):
        if name in self.camera_name_list:
            return self.camera_name_list.index(name)
        else:
            warnings.warn("Camera name {} not found, valid camera names: {}".format(name, self.camera_name_list))

    def render_point_cloud(self, cam_id, rgb=True, use_open3d=False):
        camera = self.cam_list[cam_id]
        camera.take_picture()
        depth = self.depth_lambda_list[cam_id](camera.get_depth())[:, :, np.newaxis]
        result = self.mapping_list[cam_id] * depth
        valid = result[:, :, 2] < RGBD_CAMERA_THRESHOLD

        if rgb:
            color = camera.get_color_rgba()
            result = np.concatenate([result, color], axis=2)

        if not use_open3d:
            return result, valid, None

        else:
            pc = open3d.geometry.PointCloud()
            pc.points = open3d.utility.Vector3dVector(np.reshape(result[valid, :3], [-1, 3]))
            if rgb:
                pc.colors = open3d.utility.Vector3dVector(np.reshape(color[:, :, :3][valid], [-1, 3]))
            return result, valid, pc

    def step(self):
        self.sim.step()
        self.sim.update_renderer()
        self.sim.pack(self.data['state'][self.simulation_steps, :])
        self.simulation_steps += 1
