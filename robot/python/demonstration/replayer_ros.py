import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sapyen_robot
import numpy as np
from robot.python.demonstration.replayer import Replayer as ParentModule
import copy

CAMERA_TO_LINK = np.zeros([4, 4])
CAMERA_TO_LINK[[0, 1, 2, 3], [2, 0, 1, 3]] = [1, -1, -1, 1]


class ReplayerRos(ParentModule):
    def __init__(self, partnet_id, mode="state"):
        super().__init__(partnet_id)

        # Replay mode can be state based or control based
        self.mode = mode
        if mode == "control":
            self.robot.set_drive_qpos([0.25, -1.9347, 0, -1.5318, 0, 0.9512, -2.24, 0.34, 0.64, -1.413, 0, 0, 0])
            self.robot.set_pd(5000, 1500, 50000)
            controllable_wrapper = self.sim.create_controllable_articulation(self.robot)
            self.manger = sapyen_robot.ControllerManger("movo", controllable_wrapper)
            self.ps3 = sapyen_robot.MOVOPS3(self.manger)
            self.ps3.set_replay_mode()
        elif mode == "state":
            pass
        else:
            print("Possible mode is {} and {}".format("control", "state"))
            raise Exception("Mode is invalid: {}".format(mode))

        # ROS camera specification
        self.camera_frame_id = []
        self.camera_pose = []
        self.pub_list = []
        self.__POINT_CLOUD_NAMESPACE = "sapien/replay/"
        self.init_camera_ros()

        # Add Point Cloud2 template
        self.rgb_cloud = None
        self.cloud = None
        self.add_ros_cloud_template()

        # Init ROS node
        rospy.init_node("sapien")

    def init_camera_ros(self):
        for i in range(len(self.mount_actor_list)):
            self.camera_frame_id.append("/kinect2_color_optical_frame")
            self.camera_pose.append(np.eye(4, dtype=np.float32))
            self.pub_list.append(
                rospy.Publisher(self.__POINT_CLOUD_NAMESPACE + "movo_camera{}".format(i), PointCloud2, queue_size=1))

    def add_camera(self, name, camera_pose: np.ndarray, width: int, height: int, fov=1.1, near=0.01, far=100):
        super().add_camera(name, camera_pose, width, height, fov, near, far)
        self.camera_frame_id.append("/base_link")
        self.camera_pose.append((camera_pose @ CAMERA_TO_LINK).astype(np.float32))
        self.pub_list.append(rospy.Publisher(self.__POINT_CLOUD_NAMESPACE + name, PointCloud2, queue_size=1))

    def render_point_cloud(self, cam_id, rgba=True, use_open3d=False):
        cloud_array, valid, open3d_cloud = super().render_point_cloud(cam_id, rgba, use_open3d)
        self.publish_point_cloud(cloud_array[valid], rgba, cam_id)
        return cloud_array, valid, open3d_cloud

    def publish_point_cloud(self, cloud: np.ndarray, rgb, cam_id):
        padding_cloud = np.concatenate([cloud[:, 0:3].T, np.ones([1, cloud.shape[0]], dtype=np.float32)], axis=0)
        homo_cloud = self.camera_pose[cam_id] @ padding_cloud
        if rgb:
            rgb_array = np.array(cloud[:, 3:7].copy() * 256, dtype=np.uint32)
            cloud_array = np.zeros(cloud.shape[0],
                                   dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)])
            cloud_array['x'] = homo_cloud[0, :]
            cloud_array['y'] = homo_cloud[1, :]
            cloud_array['z'] = homo_cloud[2, :]
            cloud_array['rgb'] = np.array((rgb_array[:, 0] << 16) | (rgb_array[:, 1] << 8) | (rgb_array[:, 2] << 0),
                                          dtype=np.uint32)
            cloud_msg = copy.copy(self.rgb_cloud)
            cloud_msg.data = cloud_array[:, np.newaxis].tostring()
        else:
            cloud_msg = copy.copy(self.cloud)
            cloud_msg.data = np.asarray(cloud[:, np.newaxis, :], np.float32).tostring()

        cloud_msg.header.stamp = rospy.Time.now()
        cloud_msg.width = cloud.shape[0]
        cloud_msg.row_step = cloud_msg.point_step * cloud.shape[0]
        cloud_msg.header.frame_id = self.camera_frame_id[cam_id]

        self.pub_list[cam_id].publish(cloud_msg)

    def add_ros_cloud_template(self):
        # XYZ RGB point cloud2
        self.rgb_cloud = PointCloud2()
        self.rgb_cloud.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1)
        ]
        self.rgb_cloud.point_step = 16
        self.rgb_cloud.is_bigendian = False
        self.rgb_cloud.height = 1
        self.rgb_cloud.is_dense = True

        # XYZ point cloud 2
        self.cloud = PointCloud2()
        self.cloud.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        self.cloud.point_step = 12
        self.cloud.is_bigendian = False
        self.cloud.height = 1
        self.cloud.is_dense = True

    def step(self):
        if self.mode == "state":
            if self.simulation_steps % 5 == 0:
                super().step()
            else:
                self.simulation_steps += 1
        elif self.mode == "control":
            self.sim.step()
            self.sim.update_renderer()
            self.ps3.apply_cache(self.data['control'][self.simulation_steps, :])
            self.ps3.step()
            self.simulation_steps += 1
