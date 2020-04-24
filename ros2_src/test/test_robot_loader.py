# import sapien.core as sapien
import sys
import pysapien_ros2
import numpy as np

sapien = pysapien_ros2.core
ros2 = pysapien_ros2.ros2


def substitute_path(string: str, path):
    return string.replace("filename=\"", "filename=\"{}/".format(path))


def main():
    engine = sapien.Engine()
    renderer = sapien.OptifuserRenderer()
    engine.set_renderer(renderer)
    controller = sapien.OptifuserController(renderer)

    scene: sapien.Scene = engine.create_scene()
    scene.set_timestep(1 / 200)
    scene.add_ground(0)
    controller.set_current_scene(scene)

    controller.set_camera_position(-0.5, 2, 0.5)
    controller.set_camera_rotation(-1, 0)
    scene.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])
    scene.set_ambient_light((0.5, 0.5, 0.5))

    # Gif recorder
    camera_mount_actor = scene.create_actor_builder().build(is_kinematic=True)
    camera_mount_actor.set_pose(sapien.Pose([-0.5, 1.5, 0.5], [0.8775826, 0, 0, -0.4794255]))
    camera = scene.add_mounted_camera('first_camera', camera_mount_actor, sapien.Pose(), 640, 480, np.deg2rad(35),
                                      np.deg2rad(35), 0.1, 100)

    l = scene.create_urdf_loader()
    with open("/home/sim/project/sapien/example/assets/robot/xarm6.urdf") as f:
        data = f.read()
        data = substitute_path(data, "/home/sim/project/sapien/example/assets/robot")
    # robot0 = l.load_from_string(data, "")

    # Manager
    scene_manager = ros2.SceneManager(scene, "scene1")
    loader = scene_manager.create_robot_loader()
    loader.fix_root_link = True

    robot_descriptor = ros2.RobotDescriptor("sapien_resources", "xarm6_description/urdf/xarm6.urdf",
                                            "xarm6_moveit_config/config/xarm6.srdf", True)
    robot, manager = loader.load_robot_and_manager(robot_descriptor, "my_xarm")
    robot.set_root_pose(sapien.Pose())
    robot.set_qpos(robot.dof * [0])
    robot.set_qf(robot.dof * [1])

    loader = scene_manager.create_robot_loader()
    loader.fix_root_link = True

    # Load second robot using
    robot_descriptor2 = ros2.RobotDescriptor(True, "assets_local/robot/panda.urdf", "")
    robot2, manager2 = loader.load_robot_and_manager(robot_descriptor2, "panda")
    robot2.set_root_pose(sapien.Pose([2, 0, 0]))

    manager.set_drive_property(1000, 50, 5000, np.arange(6))
    manager.set_drive_property(0, 50, 100, np.arange(6, 12))

    # Controller
    arm_latency = 0.1
    gripper_joints = ["drive_joint", "left_finger_joint", "left_inner_knuckle_joint", "right_outer_knuckle_joint",
                      "right_finger_joint", "right_inner_knuckle_joint"]
    manager.create_joint_publisher(20)
    gripper_controller = manager.build_joint_velocity_controller(gripper_joints, "gripper_joint_velocity", 0)
    arm_controller = manager.build_cartesian_velocity_controller("arm", "arm_cartesian_velocity", arm_latency)

    # Start
    controller.show_window()
    scene_manager.start()
    step = 0
    print("====================================================")

    while not controller.should_quit:
        scene.update_render()
        scene.step()
        controller.render()
        manager2.balance_passive_force()
        step += 1
        gripper_controller.move_joint(np.ones(6) * 5, True)


if __name__ == '__main__':
    ros2.rclcpp_init(sys.argv)
    main()
