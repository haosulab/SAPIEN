import numpy as np
import imageio
from PIL import ImageColor

import sapien.core as sapien
import sapien.ros2 as sr
import transforms3d


# class GifRecorder:
#     def __init__(self):
#         self.images = []
#         self.color_map = np.array([ImageColor.getrgb(color) for color in ImageColor.colormap.keys()], dtype=np.uint8)
#
#     def record(self, img):
#         self.images.append((img * 256).astype(np.uint8))
#
#     def save(self, filename):
#         with imageio.get_writer(filename, mode='I', format="GIF-FI") as writer:
#             for img in self.images:
#                 writer.append_data(img)


def main():
    engine = sapien.Engine()
    renderer = sapien.OptifuserRenderer()
    renderer.enable_global_axes(True)
    engine.set_renderer(renderer)
    controller = sapien.OptifuserController(renderer)

    scene: sapien.Scene = engine.create_scene()
    scene.set_timestep(1 / 200)
    scene.add_ground(0)
    controller.set_current_scene(scene)

    controller.set_camera_position(-0.8, 0.875, 0.65)
    controller.set_camera_rotation(-1, 0)
    scene.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])
    scene.set_ambient_light((0.5, 0.5, 0.5))

    # Gif recorder
    degree = 1.0 * np.pi / 180
    # recorder = GifRecorder()
    camera_mount_actor = scene.create_actor_builder().build(is_kinematic=True)
    camera_mount_actor.set_pose(
        sapien.Pose([-0.6, 0.65, 0.95], transforms3d.euler.euler2quat(0, 20 * degree, -40 * degree, "sxyz")))
    camera = scene.add_mounted_camera('first_camera', camera_mount_actor, sapien.Pose(), 1920, 1080, np.deg2rad(35),
                                      np.deg2rad(35), 0.1, 100)

    scene_manager = sr.SceneManager(scene, "example_scene")
    loader = scene_manager.create_robot_loader()
    loader.fix_root_link = True

    descriptor = sr.RobotDescriptor(is_path=True, urdf="../../assets_local/robot/panda.urdf", srdf="")
    robot, manager = loader.load_robot_and_manager(descriptor, "panda")
    robot.set_root_pose(sapien.Pose([0, 0, 0], [1, 0, 0, 0]))
    robot.set_qpos([0, 0, 0, -1.5, 0, 1.5, 0.7, 0.4, 0.4])
    robot.set_drive_target([0, 0, 0, -1.5, 0, 1.5, 0.7, 0.4, 0.4])
    manager.set_drive_property(3000, 500, 50000, np.arange(9))

    manager.create_joint_publisher(20)
    arm_controller = manager.build_cartesian_velocity_controller("panda_arm", "arm_cartesian_velocity", 0)
    gripper_controller = manager.build_joint_velocity_controller(["panda_finger_joint1", "panda_finger_joint2"],
                                                                 "panda_gripper")
    manager.set_motion_planning_config(sr.MotionPlanningConfig())
    planner = manager.build_motion_planner("panda_arm", "asd")

    # Start
    controller.show_window()
    scene_manager.start()
    step = 0
    import time
    time.sleep(2.0)
    while not controller.should_quit:
        scene.step()
        scene.update_render()
        controller.render()
        step += 1

    # recorder.save("/home/sim/gif/no_arm_space_demo.gif")


if __name__ == '__main__':
    import sys

    sr.rclcpp_init(sys.argv)
    main()
