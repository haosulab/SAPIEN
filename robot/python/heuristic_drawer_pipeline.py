import sapyen
import sapyen_robot
import numpy as np
from robot.python.demonstration.DrawerEnv import DrawerEnv
import transforms3d

WHEEL_VELOCITY = 0.8


class DrawerTraditionalPipeline:
    def __init__(self, partnet_id: str):
        self.env = DrawerEnv(partnet_id)
        self.header = self.env.generate_header()
        self.mapping = self.env.mapping
        self.robot = self.env.robot
        self.obj = self.env.obj
        self.manger = self.env.manger

        # Add controller
        self.head_joint = ["pan_joint", "tilt_joint"]
        self.gripper_joint = ["right_gripper_finger1_joint", "right_gripper_finger2_joint",
                              "right_gripper_finger3_joint"]
        self.body_joint = ["linear_joint"]
        self.manger.add_joint_state_publisher(60, 400)
        self.head_controller = self.manger.create_joint_velocity_controller(self.head_joint, "head")
        self.gripper_controller = self.manger.create_joint_velocity_controller(self.gripper_joint, "gripper")
        self.body_controller = self.manger.create_joint_velocity_controller(self.body_joint, "body")
        self.root_pose = self.get_robot_link_pose("base_link")

        # Bind base function
        self.step = self.env.step
        self.simulation_hz = self.env.simulation_hz

    def get_robot_link_pose(self, name: str):
        links = self.robot.get_links()
        link_names = self.robot.get_link_names()
        link_index = link_names.index(name)
        return self.transform2mat(links[link_index].get_global_pose())

    def move_camera_toward_semantic(self, semantic_name="drawer"):
        links = self.obj.get_links()
        semantic_link = self.mapping['semantic2link']
        name = semantic_link[semantic_name]
        link_names = self.obj.get_link_names()
        link_index = link_names.index(name)

        # Move robot to the right place
        link_pose = self.transform2mat(links[link_index].get_global_pose())
        camera_pose = self.get_robot_link_pose("kinect2_color_optical_frame")
        relative_pose = link_pose[:3, 3] - camera_pose[:3, 3]
        self.move_robot(link_pose)

        # Move camera to the right place
        for _ in range(self.simulation_hz):
            self.head_controller.move_joint(["tilt_joint"], -0.5)

    def move_robot(self, object_pose):
        # TODO continuous control
        robot_target_pose = np.eye(4)
        robot_target_pose[0, 3] = object_pose[0, 3] - 2
        robot_target_pose[1, 3] = object_pose[1, 3]
        # robot_target_pose[0:2, 0:2] = object_pose[0:2, 0:2]
        self.manger.move_base(self.mat2transform(robot_target_pose))

    @staticmethod
    def transform2mat(trans: sapyen.Pose):
        mat = np.eye(4)
        mat[:3, :3] = transforms3d.quaternions.quat2mat(trans.q)
        mat[:3, 3] = trans.p
        return mat

    @staticmethod
    def mat2transform(mat: np.ndarray):
        pose = sapyen.Pose(mat[: 3, 3], transforms3d.quaternions.mat2quat(mat[:3, :3]))
        return pose

    def render_drawer_point_cloud(self):
        cloud, valid = self.env.render_point_cloud(0, rgba=True, segmentation=True)
        np.save("test_pc", cloud)


if __name__ == '__main__':
    import sys

    sapyen_robot.ros.init(sys.argv, "pipeline")
    partnet_id: str = "44826"
    pipeline = DrawerTraditionalPipeline(partnet_id)
    pipeline.move_camera_toward_semantic()
    pipeline.render_drawer_point_cloud()

    for i in range(1000):
        pipeline.step()
