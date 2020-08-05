import numpy as np

import pysapien_ros2.core as sapien
import pysapien_ros2.ros2 as sr
import transforms3d


def main():
    engine = sapien.Engine()
    renderer = sapien.OptifuserRenderer()
    renderer.enable_global_axes(True)
    engine.set_renderer(renderer)
    controller = sapien.OptifuserController(renderer)

    scene: sapien.Scene = engine.create_scene()
    scene.set_timestep(1 / 200)
    controller.set_current_scene(scene)

    controller.set_camera_position(-0.8, 0.875, 0.65)
    controller.set_camera_rotation(-1, 0)
    scene.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])
    scene.set_ambient_light((0.5, 0.5, 0.5))

    scene_manager = sr.SceneManager(scene, "example_scene")
    loader = scene_manager.create_robot_loader()
    loader.fix_root_link = True

    descriptor = sr.RobotDescriptor.from_path(urdf_path="assets_local/robot/panda.urdf", srdf_path="")
    robot, manager = loader.load_robot_and_manager(descriptor, "panda", config={})
    robot.set_root_pose(sapien.Pose([0, 0, 0], [1, 0, 0, 0]))
    robot.set_qpos([0, 0, 0, -1.5, 0, 1.5, 0.7, 0.4, 0.4])
    robot.set_drive_target(robot.get_qpos())
    manager.set_drive_property(3000, 500, 5000, np.arange(9))

    manager.create_joint_publisher(30)
    arm_controller = manager.build_cartesian_velocity_controller("panda_arm")
    gripper_controller = manager.build_joint_velocity_controller(["panda_finger_joint1", "panda_finger_joint2"],
                                                                 "panda_gripper")
    manager.set_motion_planning_config(sr.MotionPlanningConfig())
    planner = manager.build_motion_planner("panda_arm", "asd")

    obj_loader = scene.create_urdf_loader()
    obj_loader.scale = 0.2
    obj = obj_loader.load("assets/148/mobility.urdf")
    obj.set_root_pose(sapien.Pose([0.4, 0.2, 0.4]))

    planner.update_collision_objects(obj.get_links())

    # Start
    controller.show_window()
    scene_manager.start()
    scene.step()
    step = 0
    while step <= 900:
        manager.balance_passive_force()
        scene.step()
        scene.update_render()
        controller.render()
        step += 1

        if 500 < step < 600:
            gripper_controller.move_joint([-1, -1], False)
            arm_controller.move_cartesian([-0.2, 0, 0], sr.MoveType.WORLD_TRANSLATE)

        if 700 < step < 800:
            arm_controller.move_twist([0.0, -0.0, -0.0, 0.0, 0.3, 0.0], sr.MoveType.BODY_TWIST)

        if step == 850:
            ee_pose = robot.get_links()[9].get_pose()
            ee_pose.set_p(ee_pose.p + np.array([0, 0.3, -0.5]))
            planner.set_start_state_to_current_state()
            planner.set_goal_state(ee_pose)
            plan = planner.plan()

    if step > 850:
        now = scene_manager.now()
        for i in range(len(plan.duration)):
            finish_time = now + plan.duration[i]
            print(i)
            target_position = np.concatenate([plan.position[i, :], [0, 0]])
            robot.set_drive_target(target_position)
            while now < finish_time:
                now = scene_manager.now()
                manager.balance_passive_force()
                scene.step()
                scene.update_render()
                controller.render()
                step += 1

    while not controller.should_quit:
        manager.balance_passive_force()
        scene.step()
        scene.update_render()
        controller.render()
        step += 1



scene = None

if __name__ == '__main__':
    import sys
    import os

    sr.init_spd_logger()
    sr.set_ros2_logging_level("warning")
    resources_dir = os.path.abspath("/home/sim/project/sapien/python/py_ros2_package/ros2".rstrip('/'))
    sr.set_resources_directory(resources_dir)
    print(resources_dir)
    sr.rclcpp_init(sys.argv)
    main()
