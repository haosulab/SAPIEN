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
        scene.render_system.disable_auto_upload()
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
    body_data = px.gpu_create_body_data_buffer(len(all_bodies))
    body_index = px.gpu_create_body_index_buffer(all_bodies)
    body_offset = px.gpu_create_body_offset_buffer(all_bodies)

    art_link_pose = px.gpu_create_articulation_link_pose_buffer()
    art_link_vel = px.gpu_create_articulation_link_velocity_buffer()

    art_qpos = px.gpu_create_articulation_q_buffer()
    art_index = px.gpu_create_articulation_index_buffer(robots)
    art_offset = px.gpu_create_articulation_offset_buffer()

    class BodyRenderUpdater:
        def __init__(self, all_scenes, all_bodies, body_data_buffer):
            self.scenes = all_scenes
            scene_id = dict((s, i) for i, s in enumerate(all_scenes))
            self.scene_transforms = [
                scene.render_system.cuda_object_transforms for scene in all_scenes
            ]
            import torch

            self.scene_transform_pointers = torch.tensor(
                [r.__cuda_array_interface__["data"][0] for r in self.scene_transforms],
                device="cuda",
                dtype=torch.int64,
            )
            self.body_data_buffer = body_data_buffer

            scene_indices = []
            transform_indices = []
            local_poses = []
            local_scales = []
            parent_indices = []
            for body_index, body in enumerate(all_bodies):
                scene = body.entity.scene
                body = body.entity.find_component_by_type(
                    sapien.render.RenderBodyComponent
                )
                if not body:
                    continue
                for shape in body.render_shapes:
                    scene_indices.append(scene_id[scene])
                    transform_indices.append(shape.get_gpu_transform_index())
                    local_poses.append(shape.get_local_pose())
                    local_scales.append(shape.get_gpu_scale())
                    parent_indices.append(body_index)

            self.scene_indices = torch.tensor(
                scene_indices, device="cuda", dtype=torch.int32
            )
            self.transform_indices = torch.tensor(
                transform_indices, device="cuda", dtype=torch.int32
            )
            self.local_poses = torch.tensor(
                [[*p.p, *p.q] for p in local_poses], device="cuda", dtype=torch.float32
            )
            self.local_scales = torch.tensor(
                local_scales, device="cuda", dtype=torch.float32
            )
            self.parent_indices = torch.tensor(
                parent_indices, device="cuda", dtype=torch.int32
            )

        def update_render(self):
            sapien.render.gpu_transfer_poses_to_render_scenes(
                self.scene_transform_pointers,
                self.scene_indices,
                self.transform_indices,
                self.local_poses,
                self.local_scales,
                self.parent_indices,
                self.body_data_buffer,
            )
            sapien.render.gpu_notify_poses_updated()

    class ArticulationRenderUpdater:
        def __init__(
            self,
            all_scenes: list[sapien.Scene],
            all_links: list[sapien.physx.PhysxArticulationLinkComponent],
            articulation_link_pose_buffer,
        ):
            self.scenes = all_scenes
            scene_id = dict((s, i) for i, s in enumerate(all_scenes))

            self.scene_transforms = [
                scene.render_system.cuda_object_transforms for scene in all_scenes
            ]
            import torch

            self.scene_transform_pointers = torch.tensor(
                [r.__cuda_array_interface__["data"][0] for r in self.scene_transforms],
                device="cuda",
                dtype=torch.int64,
            )

            self.articulation_link_pose_buffer = articulation_link_pose_buffer

            max_link_count = articulation_link_pose_buffer.shape[1]

            scene_indices = []
            transform_indices = []
            local_poses = []
            local_scales = []
            parent_indices = []
            for link in all_links:
                scene = link.entity.scene
                body = link.entity.find_component_by_type(
                    sapien.render.RenderBodyComponent
                )
                if not body:
                    continue
                for shape in body.render_shapes:
                    scene_indices.append(scene_id[scene])
                    transform_indices.append(shape.get_gpu_transform_index())
                    local_poses.append(shape.get_local_pose())
                    local_scales.append(shape.get_gpu_scale())
                    parent_indices.append(
                        max_link_count * link.articulation.get_gpu_index() + link.index
                    )

            self.scene_indices = torch.tensor(
                scene_indices, device="cuda", dtype=torch.int32
            )
            self.transform_indices = torch.tensor(
                transform_indices, device="cuda", dtype=torch.int32
            )
            self.local_poses = torch.tensor(
                [[*p.p, *p.q] for p in local_poses], device="cuda", dtype=torch.float32
            )
            self.local_scales = torch.tensor(
                local_scales, device="cuda", dtype=torch.float32
            )
            self.parent_indices = torch.tensor(
                parent_indices, device="cuda", dtype=torch.int32
            )

        def update_render(self):
            sapien.render.gpu_transfer_poses_to_render_scenes(
                self.scene_transform_pointers,
                self.scene_indices,
                self.transform_indices,
                self.local_poses,
                self.local_scales,
                self.parent_indices,
                self.articulation_link_pose_buffer,
            )
            sapien.render.gpu_notify_poses_updated()

    # sync initial pose to objects
    for scene in scenes:
        scene.update_render()

    # initialize cameras
    for cam in cams:
        cam.gpu_init()

    np.random.shuffle(all_links)

    body_updater = BodyRenderUpdater(scenes, all_bodies, body_data)
    art_updater = ArticulationRenderUpdater(scenes, all_links, art_link_pose)

    import matplotlib.pyplot as plt

    for _ in range(20):
        for _ in range(10):
            px.step()

        px.gpu_query_body_data(body_data, body_index, body_offset)
        px.gpu_query_articulation_link_pose(art_link_pose, art_index, art_offset)

        print(cams[0].cuda_buffer)
        print(cams[1].cuda_buffer)

        body_updater.update_render()
        art_updater.update_render()

        cams[0].take_picture()
        cams[1].take_picture()

        plt.subplot(1, 2, 1)
        plt.imshow(cams[0].get_picture("Color"))
        plt.subplot(1, 2, 2)
        plt.imshow(cams[1].get_picture("Color"))
        plt.show()

    viewer = scenes[0].create_viewer()
    viewer.render()

    # NOTE: viewer can make the scene transform invalid (it adds stuff into the scene), so viewer cannot really be used for now
    # so we make new updaters
    body_updater = BodyRenderUpdater(scenes, all_bodies, body_data)
    art_updater = ArticulationRenderUpdater(scenes, all_links, art_link_pose)

    while not viewer.closed:
        px.step()

        # copy poses from physx to buffer
        px.gpu_query_body_data(body_data, body_index, body_offset)
        px.gpu_query_articulation_link_pose(art_link_pose, art_index, art_offset)

        # copy poses from buffer to renderer
        body_updater.update_render()
        art_updater.update_render()

        viewer.render()


main()
