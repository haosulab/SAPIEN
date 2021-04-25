import sapien.core as sapien
from sapien.utils.viewer import Viewer


def demo(fix_root_link, balance_passive_force):
    engine = sapien.Engine()
    renderer = sapien.VulkanRenderer()
    engine.set_renderer(renderer)

    scene_config = sapien.SceneConfig()
    scene = engine.create_scene(scene_config)
    scene.set_timestep(1 / 240.0)
    scene.add_ground(0)

    rscene = scene.get_renderer_scene()
    rscene.set_ambient_light([0.5, 0.5, 0.5])
    rscene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    viewer = Viewer(renderer)
    viewer.set_scene(scene)
    viewer.set_camera_xyz(x=-2, y=0, z=1)
    viewer.set_camera_rpy(r=0, p=-0.3, y=0)

    # Load URDF
    loader: sapien.URDFLoader = scene.create_urdf_loader()
    loader.fix_root_link = fix_root_link
    robot: sapien.Articulation = loader.load("../assets/robot/jaco2/jaco2.urdf")
    robot.set_root_pose(sapien.Pose([0, 0, 0], [1, 0, 0, 0]))

    # Set initial joint positions
    arm_init_qpos = [4.71, 2.84, 0, 0.75, 4.62, 4.48, 4.88]
    gripper_init_qpos = [0, 0, 0, 0, 0, 0]
    init_qpos = arm_init_qpos + gripper_init_qpos
    robot.set_qpos(init_qpos)

    camera = create_camera(scene, [-2, 0, 1], [0, -0.3, 0])
    import imageio
    images = []

    steps = 0
    total_steps = 240 * 10 if balance_passive_force else 240
    render_freq = 4 * 5 if balance_passive_force else 4
    fps = 10 if balance_passive_force else 30
    while steps < total_steps:
        for _ in range(render_freq):  # render every 4 steps
            if balance_passive_force:
                qf = robot.compute_passive_force(
                    gravity=True, 
                    coriolis_and_centrifugal=True, 
                )
                robot.set_qf(qf)
            scene.step()
            steps += 1

        scene.update_render()
        viewer.render()

        camera.take_picture()
        rgba = camera.get_float_texture('Color')  # [H, W, 4]
        rgba_img = (rgba * 255).clip(0, 255).astype("uint8")
        images.append(rgba_img)
    
    viewer.close()
    if fix_root_link and balance_passive_force:
        filename = 'robot_fix_balance.gif'
    elif fix_root_link and not balance_passive_force:
        filename = 'robot_fix.gif'
    elif not fix_root_link and not balance_passive_force:
        filename = 'robot_fall.gif'
    else:
        raise NotImplementedError()
    imageio.mimsave(filename, images, fps=fps)
    from pygifsicle import optimize
    optimize(filename)


def create_camera(scene, xyz, rpy):
    from sapien.core import Pose
    from transforms3d.euler import euler2quat
    cam_mount_actor = scene.create_actor_builder().build_kinematic()
    cam_pose = Pose(xyz, euler2quat(rpy[0], -rpy[1], -rpy[2]))
    camera = scene.add_mounted_camera(
        'recorder',
        cam_mount_actor,
        cam_pose,
        width=1280,
        height=720,
        fovx=1.0,
        fovy=1.0,
        near=0.001,
        far=100,
    )
    return camera


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--fix-root-link', action='store_true')
    parser.add_argument('--balance-passive-force', action='store_true')
    args = parser.parse_args()

    demo(fix_root_link=args.fix_root_link,
         balance_passive_force=args.balance_passive_force)


if __name__ == '__main__':
    main()
