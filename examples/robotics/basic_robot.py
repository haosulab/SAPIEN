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
    viewer.set_camera_rpy(y=0, p=-0.3, r=0)

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

    while not viewer.closed:
        for _ in range(4):  # render every 4 steps
            if balance_passive_force:
                qf = robot.compute_passive_force(
                    gravity=True, 
                    coriolis_and_centrifugal=True, 
                )
                robot.set_qf(qf)
            scene.step()
        scene.update_render()
        viewer.render()


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
