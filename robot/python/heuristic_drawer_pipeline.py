import sapyen
import sapyen_robot
import numpy as np
from robot.python.demonstration.DrawerEnv import DrawerEnv, transform2mat, mat2transform
import transforms3d

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
        self.root_pose = self.env.get_robot_link_pose("base_link")

        # Bind base function
        self.step = self.env.step
        self.simulation_hz = self.env.simulation_hz

    def move_camera_toward_semantic(self, semantic_name="drawer"):
        links = self.obj.get_links()
        semantic_link = self.mapping['semantic2link']
        name = semantic_link[semantic_name]
        link_names = self.obj.get_link_names()
        link_index = link_names.index(name)

        # Move robot to the right place
        link_pose = transform2mat(links[link_index].get_global_pose())
        camera_pose = self.env.get_robot_link_pose("kinect2_color_optical_frame")
        relative_pose = link_pose[:3, 3] - camera_pose[:3, 3]
        self.env.move_robot_to_target_place(link_pose)

        # Move camera to the right place
        for _ in range(self.simulation_hz):
            self.head_controller.move_joint(["tilt_joint"], -0.5)

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

    for i in range(10000):
        pipeline.step()
