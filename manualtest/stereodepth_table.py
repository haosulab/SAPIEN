from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import sapien
import trimesh
from sapien import ActorBuilder, Entity, Pose, Scene
from sapien.physx import PhysxCpuSystem
from sapien.render import RenderMaterial, RenderSystem, RenderTexture2D
from sapien.sensor import StereoDepthSensor, StereoDepthSensorConfig


def build_scene(
    render_system,
    physx_system,
    assets_dir=Path(__file__).resolve().parents[1] / "assets",
):
    assert assets_dir.is_dir(), f"assets_dir not exists: {assets_dir}"

    scene = Scene([render_system, physx_system])

    # Beer can
    material = RenderMaterial(
        base_color=[1.0, 1.0, 1.0, 1.0], metallic=0.001, roughness=0.4
    )
    material.base_color_texture = RenderTexture2D(
        str(assets_dir / "aligned/beer_can/texture.png")
    )
    beer_can = (
        ActorBuilder()
        .set_scene(scene)
        .add_visual_from_file(
            str(assets_dir / "aligned/beer_can/visual_mesh.obj"), material=material
        )
        .set_name("bear_can")
        .build_kinematic()
    )
    beer_can.pose = Pose(
        [0.370301, -0.246856, 0.0738802], [0.922673, -0.00379302, -0.00852731, 0.385469]
    )

    # Champagne
    material = RenderMaterial(
        base_color=[1.0, 1.0, 1.0, 1.0], metallic=0.01, roughness=0.2
    )
    material.base_color_texture = RenderTexture2D(
        str(assets_dir / "aligned/champagne/texture.png")
    )
    champagne = (
        ActorBuilder()
        .set_scene(scene)
        .add_visual_from_file(
            str(assets_dir / "aligned/champagne/visual_mesh.obj"), material=material
        )
        .set_name("champagne")
        .build_kinematic()
    )
    champagne.pose = Pose(
        [0.182963, -0.277838, 0.0873777], [0.723872, 0.00616071, -0.00678847, -0.689874]
    )

    # pepsi_bottle
    material = RenderMaterial(
        base_color=[1.0, 1.0, 1.0, 1.0], metallic=0.001, roughness=0.6
    )
    material.base_color_texture = RenderTexture2D(
        str(assets_dir / "aligned/pepsi_bottle/texture.png")
    )
    pepsi_bottle = (
        ActorBuilder()
        .set_scene(scene)
        .add_visual_from_file(
            str(assets_dir / "aligned/pepsi_bottle/visual_mesh.obj"), material=material
        )
        .set_name("pepsi_bottle")
        .build_kinematic()
    )
    pepsi_bottle.pose = Pose(
        [0.392403, 0.0504232, 0.116739], [0.991259, -0.00145631, -0.00922613, 0.131601]
    )

    # steel_ball
    steel_ball = (
        ActorBuilder()
        .set_scene(scene)
        .add_visual_from_file(
            str(assets_dir / "aligned/steel_ball/visual_mesh.obj"),
            material=RenderMaterial(
                base_color=[0.6, 0.6, 0.6, 1.0], metallic=0.8, roughness=0.2
            ),
        )
        .set_name("steel_ball")
        .build_kinematic()
    )
    steel_ball.pose = Pose(
        [0.192034, 0.131187, 0.0170772], [0.949057, -0.0375225, 0.0676584, -0.305458]
    )

    # water_bottle
    water_bottle = (
        ActorBuilder()
        .set_scene(scene)
        .add_visual_from_file(str(assets_dir / "aligned/water_bottle/water_bottle.glb"))
        .set_name("water_bottle")
        .build_kinematic()
    )
    water_bottle.pose = Pose(
        [0.256327, -0.0162116, -0.01], [0.00627772, -0.0093401, 0.000145366, 1.0008]
    )

    # table
    material = RenderMaterial(
        base_color=[1.0, 1.0, 1.0, 1.0], metallic=0.1, roughness=0.3
    )
    material.base_color_texture = RenderTexture2D(
        str(assets_dir / "aligned/table/texture.png")
    )
    table = (
        ActorBuilder()
        .set_scene(scene)
        .add_visual_from_file(
            str(assets_dir / "aligned/table/visual_mesh.obj"), material=material
        )
        .set_name("table")
        .build_kinematic()
    )
    table.pose = Pose(
        [0.405808, 0.022201, -0.043524],
        [0.999921, -0.000290915, -0.00932814, 0.00842011],
    )

    scene.set_ambient_light([0.0, 0.0, 0.0])
    scene.add_point_light([1.0, 0.2, 2.5], [10, 10, 10])
    scene.add_point_light([1.0, -2.7, 2.5], [10, 10, 10])
    scene.add_point_light([1.0, -5.6, 2.5], [10, 10, 10])
    scene.add_point_light([1.0, 3.1, 2.5], [10, 10, 10])
    scene.add_point_light([1.0, 6.0, 2.5], [10, 10, 10])

    return scene


def build_wall_scene(
    render_system,
    physx_system,
    no_texture=False,
    assets_dir=Path(__file__).resolve().parents[1] / "assets",
):
    assert assets_dir.is_dir(), f"assets_dir not exists: {assets_dir}"

    scene = Scene([render_system, physx_system])

    # table
    material = RenderMaterial(
        base_color=[1.0, 1.0, 1.0, 1.0], metallic=0.1, roughness=0.3
    )
    if not no_texture:
        material.base_color_texture = RenderTexture2D(
            str(assets_dir / "aligned/table/texture.png")
        )
    table = (
        ActorBuilder()
        .set_scene(scene)
        .add_visual_from_file(
            str(assets_dir / "aligned/table/visual_mesh.obj"), material=material
        )
        .set_name("table")
        .build_kinematic()
    )
    table.pose = Pose()

    scene.set_ambient_light([0.0, 0.0, 0.0])
    scene.add_point_light([1.0, 0.2, 2.5], [10, 10, 10])
    scene.add_point_light([1.0, -2.7, 2.5], [10, 10, 10])
    scene.add_point_light([1.0, -5.6, 2.5], [10, 10, 10])
    scene.add_point_light([1.0, 3.1, 2.5], [10, 10, 10])
    scene.add_point_light([1.0, 6.0, 2.5], [10, 10, 10])

    return scene


def main(model: str = "D435", wall_only: bool = False, no_texture: bool = False):
    sapien.render.set_camera_shader_dir("rt")
    sapien.render.set_ray_tracing_denoiser("optix")
    sapien.render.set_ray_tracing_samples_per_pixel(4)

    render_system = RenderSystem()
    physx_system = PhysxCpuSystem()
    if wall_only:
        scene = build_wall_scene(render_system, physx_system, no_texture)
    else:
        scene = build_scene(render_system, physx_system)

    sensor_entity = Entity()
    sensor_config = StereoDepthSensorConfig(model=model)
    sensor = StereoDepthSensor(sensor_config, sensor_entity)

    scene.add_entity(sensor_entity)
    if wall_only:
        sensor_entity.pose = Pose([-0.1, 0, 0.5], [0.5, -0.5, 0.5, 0.5])
    else:
        sensor_entity.pose = Pose(
            [-0.13732728, 0.13584249, 0.82386769],
            [0.84667092, -0.01287458, 0.53195302, -0.00292851],
        )
    sensor_entity.name = "sensor"

    render_system.step()
    sensor.take_picture()
    sensor.compute_depth()

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
    parser.add_argument(
        "--wall",
        action="store_true",
        help="Use a wall-only scene to visualize IR pattern and test surface flatness",
    )
    parser.add_argument(
        "--no-texture",
        action="store_true",
        help="Whether to load texture for the wall in the wall-only scene",
    )
    args = parser.parse_args()
    print(f"Testing Stereo Depth Sensor: {args.model}")

    main(args.model, args.wall, args.no_texture)
