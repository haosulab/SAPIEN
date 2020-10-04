import sapien.core as sapien
import sapien.ros2 as sr
import sys
import numpy as np
import imageio


class GifRecorder:
    def __init__(self):
        self.images = []

    def record(self, img):
        self.images.append((img * 256).astype(np.uint8))

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
    ground_material = sapien.PxrMaterial()
    ground_color = np.array([202, 164, 114, 256]) / 256
    ground_material.set_base_color(ground_color)
    ground_material.specular = 0.5
    scene.set_timestep(1 / 200)
    scene.add_ground(0, render_material=ground_material)
    controller.set_current_scene(scene)

    controller.set_camera_position(-0.5, 2, 0.5)
    controller.set_camera_rotation(-1, 0)
    scene.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])
    scene.set_ambient_light((0.5, 0.5, 0.5))

    # Gif recorder
    recorder = GifRecorder()
    camera_mount_actor = scene.create_actor_builder().build(is_kinematic=True)
    camera_mount_actor.set_pose(sapien.Pose([-0.5, 1.5, 0.5], [0.8775826, 0, 0, -0.4794255]))
    camera = scene.add_mounted_camera('first_camera', camera_mount_actor, sapien.Pose(), 640, 480, np.deg2rad(35),
                                      np.deg2rad(35), 0.1, 100)

    # Robot Loader
    scene_manager = sr.SceneManager(scene, "scene1")
    robot_descriptor = sr.RobotDescriptor(True, "../../assets_local/robot/panda.urdf", "")
    loader = scene_manager.create_robot_loader()
    loader.fix_root_link = True
    robot, robot_manager = loader.load_robot_and_manager(robot_descriptor, "panda")
    robot.set_qpos(np.array([0, 0, 0, -1.5, 0, 1.5, 0.7, 0.4, 0.4]))
    robot.set_drive_target(robot.get_qpos())
    robot_manager.set_drive_property(1000, 400, 10000, np.arange(7))
    robot_manager.set_drive_property(50, 20, 500, np.arange(7, 9))

    # Controller
    gripper_joints = ["panda_finger_joint1", "panda_finger_joint2"]
    arm_latency = 0.05
    robot_manager.create_joint_publisher(20)
    gripper_controller = robot_manager.build_joint_velocity_controller(gripper_joints, "gripper_joint_velocity", 0)
    arm_controller = robot_manager.build_cartesian_velocity_controller("panda_arm", "cartesian_velocity", arm_latency)

    # Start
    controller.show_window()
    scene_manager.start()
    step = 0

    # Set moving ball
    builder = scene.create_actor_builder()
    ball_pose = sapien.Pose([0.5, 0, 0.3])
    builder.add_sphere_visual(radius=0.04, color=[0.1, 0.1, 0.8])
    ball = builder.build(is_kinematic=False, name="ball")
    ball.set_pose(ball_pose)

    # Set moving ball
    builder = scene.create_actor_builder()
    builder.add_sphere_visual(radius=0.02, color=[0.8, 0.1, 0.1])
    local_ball = builder.build(is_kinematic=False, name="ball")

    angular_velocity = 0.7
    ball_center = np.array([0.5, 0, 0.3])
    filename = "gif/w_{}_latency_{}.gif".format(angular_velocity, arm_latency)
    rotation_speed = angular_velocity * 9 / 10 / np.pi
    ball_velocity = [
        (0, 0.0 + np.sin(t / 180 * rotation_speed * np.pi) * 0.6 * rotation_speed,
         -np.cos(t / 180 * rotation_speed * np.pi) * 0.6 * rotation_speed) for t
        in range(360 * 100)]
    panda_hand = robot.get_links()[-3]

    while not controller.should_quit:
        scene.step()
        if step % 10 == 0:
            scene.update_render()
            controller.render()
            camera.take_picture()
            recorder.record(camera.get_color_rgba())

        robot_manager.balance_passive_force()
        ball.set_velocity(ball_velocity[step % 36000])
        step += 1
        if diff_drive(ball, panda_hand, arm_controller, local_ball, 1):
            print("Success in step {}".format(step / 200))
            return
        #
        if step > 10 * 200:
            print("Fail!!")
            return

        gripper_controller.move_joint(np.ones(2) * -5, True)

    controller.hide_window()
    scene = None
    recorder.save(filename)
    print("Save")


def diff_drive(ball, hand, arm_controller, local_ball, scale):
    target_pos = ball.get_pose().p
    hand_pos: sapien.Pose = hand.get_pose()
    current_pos = hand_pos.to_transformation_matrix()[:3, 2] * 0.11 + hand_pos.p
    local_ball.set_pose(sapien.Pose(current_pos))
    diff = target_pos - current_pos
    if np.linalg.norm(diff) < 0.02:
        return True
    arm_controller.move_cartesian(diff * scale, sr.MoveType.WORLD_TRANSLATE)


if __name__ == '__main__':
    sr.rclcpp_init(sys.argv)
    main()
