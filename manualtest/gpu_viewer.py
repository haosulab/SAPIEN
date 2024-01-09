import sapien
import numpy as np

sapien.physx.enable_gpu()
sapien.set_cuda_tensor_backend("torch")


def main():
    scene_count = 2

    scenes = []
    for i in range(scene_count):
        scene = sapien.Scene()
        px: sapien.physx.PhysxGpuSystem = scene.physx_system
        px.set_scene_offset(scene, [i * 10, 0, 0])
        scenes.append(scene)

    px: sapien.physx.PhysxGpuSystem = scenes[0].physx_system

    urdf_loader = scenes[0].create_urdf_loader()
    builder = urdf_loader.load_file_as_articulation_builder(
        "../assets/robot/panda/panda.urdf"
    )
    robots = []
    for scene in scenes:
        scene.load_widget_from_package("demo_arena", "DemoArena")
        builder.set_scene(scene)
        robot = builder.build()
        robots.append(robot)

    all_links = [link for robot in robots for link in robot.links]

    builder = scenes[0].create_actor_builder()
    builder.add_sphere_collision(radius=0.1)
    builder.add_sphere_visual(radius=0.1)

    balls = []
    z = 5
    for scene in scenes:
        builder.set_scene(scene)
        builder.set_initial_pose(sapien.Pose([0, 0, z]))
        ball = builder.build()
        balls.append(ball)
        z += 1

    all_bodies = [
        b.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent) for b in balls
    ]

    cams = []
    for scene in scenes:
        cam = scene.add_camera("", 256, 512, 1, 0.01, 10)
        cam.entity.set_pose(sapien.Pose([-2, 0, 0.5]))
        cams.append(cam)

    px.gpu_init()

    import matplotlib.pyplot as plt

    px.gpu_fetch_articulation_link_pose()

    def fast_way():
        # tell render shapes where to look for poses
        for body in all_links + all_bodies:
            rb = body.entity.find_component_by_type(sapien.render.RenderBodyComponent)
            if rb is None:
                continue
            for s in rb.render_shapes:
                s.set_gpu_pose_batch_index(body.gpu_pose_index)

        # render system group manages batched rendering
        render_system_group = sapien.render.RenderSystemGroup(
            [s.render_system for s in scenes]
        )

        # camera group renders images in batches
        camera_group = render_system_group.create_camera_group(cams, ["Color"])
        render_system_group.set_cuda_poses(px.cuda_rigid_body_data)

        px.gpu_fetch_rigid_dynamic_data()
        px.gpu_fetch_articulation_link_pose()
        render_system_group.update_render()

        for _ in range(5):
            for _ in range(20):
                px.step()

            px.gpu_fetch_rigid_dynamic_data()
            px.gpu_fetch_articulation_link_pose()

            render_system_group.update_render()
            camera_group.take_picture()
            picture = camera_group.get_picture_cuda("Color")
            picture = picture.cpu().numpy()

            plt.subplot(1, 2, 1)
            plt.imshow(picture[0])
            plt.subplot(1, 2, 2)
            plt.imshow(picture[1])
            plt.show()

    def slow_way():
        for _ in range(10):
            for _ in range(20):
                px.step()

            px.sync_poses_gpu_to_cpu()
            for scene in scenes:
                scene.update_render()
            for cam in cams:
                cam.take_picture()

            pictures = [cam.get_picture("Color") for cam in cams]

            plt.subplot(1, 2, 1)
            plt.imshow(pictures[0])
            plt.subplot(1, 2, 2)
            plt.imshow(pictures[1])
            plt.show()

    # fast_way()

    viewer = scenes[0].create_viewer()
    viewer.render()

    while not viewer.closed:
        px.step()
        px.sync_poses_gpu_to_cpu()
        scenes[0].update_render()
        viewer.render()


main()
