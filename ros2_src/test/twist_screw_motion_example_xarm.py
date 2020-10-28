import numpy as np
import imageio
from PIL import ImageColor

import sapien.core as sapien
import sapien.ros2 as sr
import transforms3d


class GifRecorder:
    def __init__(self):
        self.images = []
        self.color_map = np.array([ImageColor.getrgb(color) for color in ImageColor.colormap.keys()], dtype=np.uint8)

    def record(self, img):
        self.images.append(img)

    def save(self, filename):
        with imageio.get_writer(filename, mode='I', format="GIF-FI") as writer:
            for img in self.images:
                writer.append_data(img)


def main(twist_example, screw_example):
    engine = sapien.Engine()
    renderer = sapien.OptifuserRenderer()
    engine.set_renderer(renderer)
    controller = sapien.OptifuserController(renderer)

    scene: sapien.Scene = engine.create_scene()
    scene.set_timestep(1 / 200)
    scene.add_ground(0)
    controller.set_current_scene(scene)

    controller.set_camera_position(-1.6, 1.75, 1.3)
    controller.set_camera_rotation(-1, 0)
    scene.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])
    scene.set_ambient_light((0.5, 0.5, 0.5))

    # Gif recorder
    degree = 1.0 * np.pi / 180
    recorder = GifRecorder()
    camera_mount_actor = scene.create_actor_builder().build(is_kinematic=True)
    camera_mount_actor.set_pose(
        sapien.Pose([-1.2, 1.3, 1.1], transforms3d.euler.euler2quat(0, 20 * degree, -45 * degree, "sxyz")))
    camera = scene.add_mounted_camera('first_camera', camera_mount_actor, sapien.Pose(), 1920, 1080, np.deg2rad(35),
                                      np.deg2rad(35), 0.1, 100)

    scene_manager = sr.SceneManager(scene, "example_scene")
    loader = scene_manager.create_robot_loader()
    loader.fix_root_link = True

    init_qpos = [0, 0.5, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    descriptor = sr.RobotDescriptor("sapien_resources", "xarm6_description/urdf/xarm6.urdf",
                                    "xarm6_moveit_config/config/xarm6.srdf", True)
    robot, manager = loader.load_robot_and_manager(descriptor, "xarm")
    robot.set_root_pose(sapien.Pose([0, 0, 0.2], [1, 0, 0, 0]))
    robot.set_qpos(init_qpos)
    robot.set_drive_target(init_qpos)
    manager.set_drive_property(1000, 50, 5000, np.arange(12))

    manager.create_joint_publisher(20)
    arm_controller = manager.build_cartesian_velocity_controller("arm", "arm_cartesian_velocity", 0)
    # gripper_controller = manager.build_joint_velocity_controller(
    #     ["j2s7s300_joint_finger_1", "j2s7s300_joint_finger_1", "j2s7s300_joint_finger_1"], "gripper")

    # Start
    controller.show_window()
    scene_manager.start()
    step = 0

    while not controller.should_quit:
        scene.step()
        scene.update_render()
        controller.render()
        step += 1
        # if step % 5 == 0:
        #     camera.take_picture()
        #     recorder.record(camera.get_color_rgba())

        manager.balance_passive_force()
        # arm_controller.move_twist([-0.05, 0, 0, 0, 0, 0], sr.MoveType.BODY_TWIST)
        arm_controller.move_twist([0.0, -0.0, 0.00, 0, 0, 0.05], sr.MoveType.BODY_TWIST)
        # arm_controller.move_cartesian([0, 0, 0.05], sr.MoveType.LOCAL_TRANSLATE)


if __name__ == '__main__':
    import sys

    sr.rclcpp_init(sys.argv)
    main(True, False)
