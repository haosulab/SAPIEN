# A minimal example of using KuafuRenderer
#
# By Jet <i@jetd.me>
#
import sapien.core as sapien
from sapien.core import Pose


def main():
    sim = sapien.Engine()

    sapien.KuafuRenderer.set_log_level("debug")

    config = sapien.KuafuConfig()
    config.use_viewer = True
    config.spp = 1
    config.max_materials = 32
    config.max_textures = 32
    config.max_geometries = 256
    config.max_geometry_instances = 256

    renderer = sapien.KuafuRenderer(config)

    sim.set_renderer(renderer)

    config = sapien.SceneConfig()
    scene = sim.create_scene(config)

    scene.add_ground(0)
    scene.set_timestep(1 / 60)
    prefix = '/home/jet/Downloads/fbtest/objects/'

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(prefix + "scene5.gltf")
    builder.add_nonconvex_collision_from_file(prefix + "col_scene.stl")
    room = builder.build_static()

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(prefix + "cup.gltf")
    builder.add_multiple_collisions_from_file(prefix + "cup.stl")
    cup = builder.build()
    cup.set_pose(Pose([-1.273, -0.781, 0.809], [0.901, 0, 0, 0.433]))

    cup = builder.build()
    cup.set_pose(Pose([-1.273, -0.881, 0.809], [0.964, 0, 0, 0.-265]))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(prefix + "glass.gltf")
    builder.add_multiple_collisions_from_file(prefix + "glass.stl")
    builder.set_mass_and_inertia(0.1, Pose([0, 0, 0.08]), [0.001, 0.001, 0.0002])
    cup = builder.build()
    cup.set_pose(Pose([-1.51, -1.05, 0.810], [1, 0, 0, 0]))
    cup = builder.build()
    cup.set_pose(Pose([-1.49, -0.95, 0.810], [1, 0, 0, 0]))
    cup = builder.build()
    cup.set_pose(Pose([-1.5, -0.85, 0.810], [1, 0, 0, 0]))
    cup = builder.build()
    cup.set_pose(Pose([-1.45, -0.75, 0.810], [1, 0, 0, 0]))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(prefix + "rack.gltf")
    builder.add_multiple_collisions_from_file(prefix + "rack.stl")
    rack = builder.build()
    rack.set_pose(Pose([-1.368, -1.387, 0.822], [0.779, 0, 0, 0.627]))

    builder = scene.create_actor_builder()
    builder.add_visual_from_file(prefix + "plate.gltf")
    builder.add_multiple_collisions_from_file(prefix + "plate.stl")
    plate = builder.build()
    plate.set_pose(Pose([-1.343, -1.293, 0.940], [-0.651, -0.654, -0.181, 0.340]))
    plate = builder.build()
    plate.set_pose(Pose([-1.351, -1.327, 0.940], [-0.651, -0.654, -0.181, 0.340]))
    plate = builder.build()
    plate.set_pose(Pose([-1.359, -1.361, 0.940], [-0.651, -0.654, -0.181, 0.340]))

    scene.set_ambient_light([1.0, 1.0, 1.0, 1.0])

    mount = scene.create_actor_builder().build_kinematic()
    mount.set_pose(Pose([-3, -3, 1.5]))
    cam = scene.add_mounted_camera("cam", mount, Pose([0, 0, 0]), 400, 400, 0, 1, 0.1, 100)

    while renderer.is_running:
        scene.step()
        scene.update_render()
        cam.take_picture()


main()
