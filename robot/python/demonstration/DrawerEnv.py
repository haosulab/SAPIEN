import sapyen
import sapyen_robot
import os
import transforms3d
import numpy as np
import warnings

PARTNET_DIR = '/home/sim/project/mobility-v0-prealpha3/mobility_verified'
RGBD_CAMERA_THRESHOLD = 10
CAMERA_TO_LINK = np.zeros([4, 4])
CAMERA_TO_LINK[[0, 1, 2, 3], [2, 0, 1, 3]] = [1, -1, -1, 1]


def transform2mat(trans: sapyen.Pose):
    mat = np.eye(4)
    mat[:3, :3] = transforms3d.quaternions.quat2mat(trans.q)
    mat[:3, 3] = trans.p
    return mat


def mat2transform(mat: np.ndarray):
    pose = sapyen.Pose(mat[: 3, 3], transforms3d.quaternions.mat2quat(mat[:3, :3]))
    return pose


class DrawerEnv:
    def __init__(self, partnet_id: str, root_pose=((3, 0, 0.5), (1, 0, 0, 0))):
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
        self.sim.add_ground(0, material=None)
        self.simulation_hz = 200
        self.sim.set_time_step(1 / self.simulation_hz)

        # Partnet Object
        urdf = os.path.join(PARTNET_DIR, partnet_id, "mobility.urdf")
        self.loader = self.sim.create_urdf_loader()
        self.obj = self.loader.load(urdf)
        self.obj.set_root_pose(root_pose[0], root_pose[1])

        # Robot Model and controller manger
        self.loader.fix_loaded_object = True
        self.loader.balance_passive_force = True
        self.robot = self.loader.load('../assets/robot/all_robot.urdf')
        controllable_wrapper = self.sim.create_controllable_articulation(self.robot)
        self.manger = sapyen_robot.ControllerManger("movo", controllable_wrapper)

        # Init
        self.renderer.show_window()
        self.mapping = self.build_semantic_mapping(os.path.join(PARTNET_DIR, partnet_id))
        self.root_theta = 0
        self.root_pos = np.array([0, 0], dtype=np.float)

        # Camera
        self.builder = self.sim.create_actor_builder()
        self.camera_frame_id = []
        self.camera_pose = []
        self.camera_name_list = []
        self.cam_list = []
        self.mount_actor_list = []
        self.mapping_list = []
        self.depth_lambda_list = []
        self.init_camera()

        # Init robot pose and controller
        self.robot.set_pd(5000, 1500, 50000)
        self.init_qpos = [0.25, -1.9347, 0, -1.5318, 0, 0.9512, -2.24, 0.34, 0.64, -1.413, 0, 0, 0]
        self.robot.set_drive_qpos(self.init_qpos)
        self.robot.set_qpos(self.init_qpos)
        self.step()

    def step(self):
        self.sim.step()
        self.sim.update_renderer()
        self.renderer.render()

    def generate_header(self):
        header = {}
        header.update({"robot_joint_name": self.robot.get_joint_names()})
        header.update({"robot_link_name": self.robot.get_link_names()})
        header.update({"object_joint_name": self.obj.get_joint_names()})
        header.update({"object_link_name": self.obj.get_link_names()})
        return header

    def build_semantic_mapping(self, part_dir: str):
        header = {'id2link': {}, 'id2semantic': {}, 'id2motion': {}}
        semantics = os.path.join(part_dir, 'semantics.txt')
        link2semantics = {}
        link2motion = {}
        semantics2link = {}
        with open(semantics, 'r') as f:
            for line in f:
                if line.strip():
                    link, motion, semantics = line.split()
                    link2semantics[link] = semantics
                    semantics2link[semantics] = link
                    link2motion[link] = motion

        for wrapper in [self.robot, self.obj]:
            header['id2link'].update(dict(zip(wrapper.get_link_ids(), wrapper.get_link_names())))
            id2name = header['id2link']
            header['id2semantic'].update(
                {i: link2semantics[id2name[i]] for i in id2name if id2name[i] in link2semantics})
            header['id2motion'].update(
                {i: link2motion[id2name[i]] for i in id2name if id2name[i] in link2motion})

        header.update({"link2semantic": link2semantics})
        header.update({"semantic2link": semantics2link})
        return header

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
        self.camera_frame_id.append("/base_link")
        self.camera_pose.append((camera_pose @ CAMERA_TO_LINK).astype(np.float32))

    def build_camera_mapping(self, height: int, width: int, camera_matrix: np.ndarray):
        x = np.linspace(0.5, width - 0.5, width)
        y = np.linspace(0.5, height - 0.5, height)
        x, y = np.meshgrid(x, y)
        cor = np.stack([x.flatten(), y.flatten(), np.ones([x.size])], axis=0)
        mapping = np.linalg.inv(camera_matrix[:3, :3]) @ cor
        self.mapping_list.append(np.reshape(mapping.T, [height, width, 3]).astype(np.float32))

    @property
    def mounted_camera(self):
        return self.camera_name_list.copy()

    def get_camera_id(self, name: str):
        if name in self.camera_name_list:
            return self.camera_name_list.index(name)
        else:
            warnings.warn("Camera name {} not found, valid camera names: {}".format(name, self.camera_name_list))

    def render_point_cloud(self, cam_id, rgba=True, segmentation=False):
        camera = self.cam_list[cam_id]
        camera.take_picture()
        depth = self.depth_lambda_list[cam_id](camera.get_depth())[:, :, np.newaxis].astype(np.float32)
        result = self.mapping_list[cam_id] * depth
        valid = result[:, :, 2] < RGBD_CAMERA_THRESHOLD

        if rgba:
            color = camera.get_color_rgba()
            result = np.concatenate([result, color], axis=2)

        if segmentation:
            seg = camera.get_segmentation()
            result = np.concatenate([result, seg[:, :, np.newaxis]], axis=2)

        return result, valid

    def get_robot_link_pose(self, name: str):
        links = self.robot.get_links()
        link_names = self.robot.get_link_names()
        link_index = link_names.index(name)
        return transform2mat(links[link_index].get_global_pose())

    def move_robot_to_target_place(self, target_pose):
        end_pose = np.eye(4)
        end_pose[0:2, 3] = target_pose[0:2, 3]
        end_pose[0:2, 0:2] = target_pose[0:2, 0:2]
        current_pose = np.eye(4)
        current_pose[0:2, 3] = self.root_pos
        new_theta = transforms3d.euler.mat2euler(end_pose)[2]
        x_axis = end_pose[0:3, 3] - current_pose[0:3, 3]
        x_axis /= np.linalg.norm(x_axis)
        z_axis = np.array([0, 0, 1])
        y_axis = np.cross(z_axis, x_axis)
        forward_pose = np.stack([x_axis, y_axis, z_axis], axis=1)
        relative_pose = np.linalg.inv(current_pose[0:3, 0:3]) @ forward_pose
        move_direction_theta = transforms3d.euler.mat2euler(relative_pose)[2]
        angular_velocity = 0.6
        velocity = 0.4
        for _ in range(
                np.ceil(np.abs(move_direction_theta - self.root_theta) / angular_velocity * self.simulation_hz).astype(
                        np.int)):
            self.root_theta += angular_velocity / self.simulation_hz
            current_pose[0:2, 0:2] = np.array([[np.cos(self.root_theta), -np.sin(self.root_theta)],
                                               [np.sin(self.root_theta), np.cos(self.root_theta)]])
            self.manger.move_base(mat2transform(current_pose))
            self.step()

        move_direction = end_pose[0:2, 3] - self.root_pos
        move_distance = np.linalg.norm(move_direction)
        move_direction /= move_distance

        for _ in range(np.ceil(move_distance / velocity * self.simulation_hz).astype(np.int)):
            self.root_pos += move_direction * velocity / self.simulation_hz
            current_pose[0:2, 3] = self.root_pos
            self.manger.move_base(mat2transform(current_pose))
            self.step()

        for _ in range(
                np.ceil(np.abs(new_theta - self.root_theta) / angular_velocity * self.simulation_hz).astype(np.int)):
            self.root_theta += angular_velocity / self.simulation_hz
            current_pose[0:2, 0:2] = np.array([[np.cos(self.root_theta), -np.sin(self.root_theta)],
                                               [np.sin(self.root_theta), np.cos(self.root_theta)]])
            self.manger.move_base(mat2transform(current_pose))
            self.step()
