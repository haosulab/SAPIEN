import pysapien as sapien


def robot_basic_control_demo(fix_robot_root, balance_passive_force, add_joint_damping):
    sim = sapien.Engine()
    renderer = sapien.OptifuserRenderer()
    sim.set_renderer(renderer)
    renderer_controller = sapien.OptifuserController(renderer)
    renderer_controller.set_camera_position(-2, 0, 1)
    renderer_controller.set_camera_rotation(0, -0.5)

    scene0 = sim.create_scene(gravity=[0, 0, -9.81])
    renderer_controller.set_current_scene(scene0)
    scene0.add_ground(altitude=0)
    scene0.set_timestep(1 / 240)
    scene0.set_ambient_light([0.5, 0.5, 0.5])
    scene0.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

    loader = scene0.create_urdf_loader()
    loader.fix_root_link = fix_robot_root
    robot: sapien.Articulation = loader.load("assets/robot/jaco2.urdf")

    arm_init_qpos = [4.71, 2.84, 0, 0.75, 4.62, 4.48, 4.88]
    gripper_init_qpos = [0, 0, 0, 0, 0, 0]
    init_qpos = arm_init_qpos + gripper_init_qpos
    robot.set_qpos(init_qpos)

    if add_joint_damping:
        for joint in robot.get_joints():
            joint.set_drive_property(stiffness=0, damping=10)

    steps = 0
    renderer_controller.show_window()
    while not renderer_controller.should_quit:
        scene0.update_render()
        for i in range(4):
            if balance_passive_force:
                qf = robot.compute_passive_force(gravity=True, coriolisAndCentrifugal=True, external=False)
                robot.set_qf(qf)
            scene0.step()
            steps += 1
        renderer_controller.render()


if __name__ == '__main__':
    robot_basic_control_demo(fix_robot_root=True, balance_passive_force=True, add_joint_damping=True)
