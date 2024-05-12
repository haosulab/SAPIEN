import matplotlib.pyplot as plt
import numpy as np
import sapien
import trimesh
from sapien import ActorBuilder, Entity, Pose, Scene
from sapien.physx import PhysxCpuSystem
from sapien.render import RenderSystem
from sapien.sensor import StereoDepthSensor, StereoDepthSensorConfig


def build_scene(render_system, physx_system):
    scene = Scene([render_system, physx_system])

    ground = (
        ActorBuilder()
        .set_scene(scene)
        .add_plane_visual(
            Pose(q=[0.7071068, 0, -0.7071068, 0]),
            [10, 10, 10],
            sapien.render.RenderMaterial(
                base_color=np.array([202, 164, 114, 256]) / 256
            ),
            "",
        )
        .set_physx_body_type("static")
        .build()
    )
    ground.name = "ground"

    sphere1 = (
        ActorBuilder()
        .set_scene(scene)
        .add_sphere_visual(
            Pose(),
            0.06,
            sapien.render.RenderMaterial(base_color=[0.2, 0.2, 0.8, 1.0]),
            "",
        )
        .set_physx_body_type("kinematic")
        .build()
    )
    sphere1.name = "sphere1"
    sphere1.pose = Pose([-0.0246242, -0.138742, 0.07], [1, 0, 0, 0])

    sphere2 = (
        ActorBuilder()
        .set_scene(scene)
        .add_sphere_visual(
            Pose(),
            0.07,
            sapien.render.RenderMaterial(base_color=[1.0, 1.0, 1.0, 1.0]),
            "",
        )
        .set_physx_body_type("kinematic")
        .build()
    )
    sphere2.name = "sphere2"
    sphere2.pose = Pose([-0.0828886, 0.0949175, 0.06], [1, 0, 0, 0])

    capsule = (
        ActorBuilder()
        .set_scene(scene)
        .add_capsule_visual(
            Pose(),
            0.02,
            0.1,
            sapien.render.RenderMaterial(base_color=[0.8, 0.7, 0.1, 1.0]),
            "",
        )
        .set_physx_body_type("kinematic")
        .build()
    )
    capsule.name = "capsule"
    capsule.pose = Pose([0.15, -0.01, 0.0150469], [1, 0, 0, 0])

    box = (
        ActorBuilder()
        .set_scene(scene)
        .add_box_visual(
            Pose(),
            [0.09, 0.09, 0.09],
            sapien.render.RenderMaterial(base_color=[0.8, 0.2, 0.2, 1.0]),
            "",
        )
        .set_physx_body_type("kinematic")
        .build()
    )
    box.name = "box"
    box.pose = Pose([0.05, 0.26797, 0.09], [1, 0, 0, 0])

    scene.set_ambient_light([0.3, 0.3, 0.3])
    scene.add_directional_light([0, 0.5, -1], color=[0.5, 0.5, 0.5])

    return scene


def main(model: str = "D435"):
    render_system = RenderSystem()
    physx_system = PhysxCpuSystem()
    scene = build_scene(render_system, physx_system)

    sensor_entity = Entity()
    sensor_config = StereoDepthSensorConfig(model=model)
    sensor = StereoDepthSensor(sensor_config, sensor_entity)

    scene.add_entity(sensor_entity)
    sensor_entity.set_pose(
        Pose(
            [-0.420344, -0.218716, 0.339383], [0.927951, -0.0967264, 0.211487, 0.291245]
        )
    )
    sensor_entity.name = "sensor"

    render_system.step()
    sensor.take_picture()

    if model == "D415":
        bbox_start = (100, 100)  # Top left corner starts at (100, 100)
        bbox_size = (640, 360)  # Width 640, Height 360
    elif model == "D435":
        bbox_start = (150, 100)  # Top left corner starts at (150, 100)
        bbox_size = (360, 240)  # Width 360, Height 240
    else:
        raise NotImplementedError(f"Unsupported stereo depth sensor model: {model}")
    sensor.compute_depth(
        bbox_start, bbox_size
    )  # Passing these two tuples to compute_depth will only do stereo matching in this region
    # More specifically, it will clip the both left and right ir images to this region before stereo matching
    # Since left and right ir images should be of same resolution, one should use the larger bbox size of the two
    # Also spare some space in the bbox as part of the depth on boundary will be invalid due to stereo matching's nature
    # For this example, using a bbox of 640*360 instead of the full 1280*720 provides about 2.7x speedup

    rgb = sensor.get_rgb()
    ir_l, ir_r = sensor.get_ir()
    depth = sensor.get_depth()

    plt.subplot(221)
    plt.title("RGB Image")
    plt.imshow((rgb * 255).astype(np.uint8))
    plt.subplot(222)
    plt.title("Left Infrared Image")
    plt.imshow((ir_l * 255).astype(np.uint8), cmap="gray")
    plt.subplot(223)
    plt.title("Right Infrared Image")
    plt.imshow((ir_r * 255).astype(np.uint8), cmap="gray")
    plt.subplot(224)
    plt.title("Depth Map")
    plt.imshow(depth)
    plt.show()

    pc = sensor.get_pointcloud(
        with_rgb=True
    )  # From RGB camera's view with x rightward, y downward, z forward
    pcd = trimesh.PointCloud(
        pc[..., :3] * np.array([1, -1, -1]), pc[..., 3:]
    )  # Change axis direction for easier view
    pcd.show()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Test stereo depth sensor")
    parser.add_argument(
        "model",
        type=str,
        nargs="?",
        default="D435",
        choices=["D415", "D435"],
        help="Stereo depth sensor model",
    )
    args = parser.parse_args()
    print(f"Testing Stereo Depth Sensor: {args.model}")

    main(args.model)
