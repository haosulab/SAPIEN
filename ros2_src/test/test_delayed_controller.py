# import pysapien as sapien
import sapien.core as sapien
import sys
import numpy as np
import pysapien_ros2
import imageio
from PIL import ImageColor


class GifRecorder:
    def __init__(self):
        self.images = []
        self.color_map = np.array([ImageColor.getrgb(color) for color in ImageColor.colormap.keys()], dtype=np.uint8)

    def record(self, img):
        self.images.append(self.color_map[img])

    def save(self, filename):
        with imageio.get_writer(filename, mode='I', format="GIF-FI") as writer:
            for img in self.images:
                writer.append_data(img)


def main():
    engine = sapien.Engine()
    renderer = sapien.OptifuserRenderer()
    engine.set_renderer(renderer)
    controller = sapien.OptifuserController(renderer)

    scene: sapien.Scene = engine.create_scene(gravity=[0, 0, 0])
    scene.set_timestep(1 / 200)
    scene.add_ground(0)
    controller.set_current_scene(scene)

    controller.set_camera_position(-0.5, 2, 0.5)
    controller.set_camera_rotation(-1, 0)
    scene.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])
    scene.set_ambient_light((0.5, 0.5, 0.5))

    # Gif recorder
    recorder = GifRecorder()
    camera_mount_actor = scene.create_actor_builder().build(is_kinematic=True)
    camera_mount_actor.set_pose(sapien.Pose([-0.5, 1.5, 0.5], [0.8775826, 0,0, -0.4794255]))
    camera = scene.add_mounted_camera('first_camera', camera_mount_actor, sapien.Pose(), 640, 480, np.deg2rad(35),
                                      np.deg2rad(35), 0.1, 100)

    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    loader.scale = 1
    robot = loader.load("example/assets/robot/xarm6.urdf")
    robot.set_root_pose(sapien.Pose([0, 0, 0.1], [1, 0, 0, 0]))

    # Manager
    scene_manager = pysapien_ros2.SceneManager(scene, "scene1")
    robot_manager: pysapien_ros2.RobotManager = scene_manager.build_robot_manager(robot, "xarm6")
    robot_manager.set_drive_property(1000, 50, 5000, np.arange(6))
    robot_manager.set_drive_property(0, 50, 100, np.arange(6, 12))

    # Controller
    arm_latency = 0.1
    gripper_joints = ["drive_joint", "left_finger_joint", "left_inner_knuckle_joint", "right_outer_knuckle_joint",
                      "right_finger_joint", "right_inner_knuckle_joint"]
    robot_manager.create_joint_publisher(20)
    gripper_controller = robot_manager.build_joint_velocity_controller(gripper_joints, "gripper_joint_velocity", 0)
    arm_controller = robot_manager.build_cartesian_velocity_controller("arm", "arm_cartesian_velocity", arm_latency)

    # Start
    controller.show_window()
    scene_manager.start()
    step = 0

    # Set moving ball
    builder: sapien.ActorBuilder = scene.create_actor_builder()
    ball_pose = sapien.Pose([0.5, 0, 0.5])
    builder.add_sphere_visual(radius=0.03)
    ball: sapien.Actor = builder.build(is_kinematic=False, name="ball")
    ball.set_pose(ball_pose)

    angular_velocity = 0.7
    filename = "gif/w_{}_latency_{}.gif".format(angular_velocity, arm_latency)
    rotation_speed = angular_velocity * 9 / 10 / np.pi
    ball_velocity = [
        (0, 0.0 + np.sin(t / 180 * rotation_speed * np.pi) * 0.6 * rotation_speed,
         -np.cos(t / 180 * rotation_speed * np.pi) * 0.6 * rotation_speed) for t
        in range(360 * 100)]
    left_finger = robot.get_links()[-1]
    right_finger = robot.get_links()[-4]

    while not controller.should_quit:
        scene.step()
        if step % 5 == 0:
            scene.update_render()
            controller.render()
            camera.take_picture()
            recorder.record(camera.get_obj_segmentation() +20)

        robot_manager.balance_passive_force()
        ball.set_velocity(ball_velocity[step % 36000])
        step += 1
        if diff_drive(ball, left_finger, right_finger, arm_controller, 4):
            print("Success in step {}".format(step / 200))
            recorder.save(filename)
            controller.hide_window()
            scene = None
            return

        if step > 10 * 200:
            print("Fail!!")
            recorder.save(filename)
            controller.hide_window()
            scene = None
            return

        gripper_controller.move_joint(np.ones(6) * 5, True)


def diff_drive(ball, left_finger, right_finger, arm_controller, scale):
    target_pos = ball.get_pose().p
    current_pos = 0.5 * left_finger.get_pose().p + 0.5 * right_finger.get_pose().p
    diff = target_pos - current_pos
    if np.linalg.norm(diff) < 0.02:
        return True
    arm_controller.move_cartesian(diff * scale, pysapien_ros2.MoveType.WORLD_TRANSLATE)


if __name__ == '__main__':
    pysapien_ros2.rclcpp_init(sys.argv)
    main()
