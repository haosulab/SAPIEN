import sapien
from sapien import (
    Scene,
    Entity,
    Pose,
)
from sapien.physx import PhysxSystem
from sapien.render import (
    SapienRenderer,
    RenderSystem,
    RenderBodyComponent,
    RenderMaterial,
    RenderCameraComponent,
    RenderPointLightComponent,
)

from sapien.utils.viewer import Viewer
from sapien import ActorBuilder, VisualMaterialRecord, PhysicalMaterialRecord
from sapien import ArticulationBuilder
from transforms3d.quaternions import axangle2quat as aa
import numpy as np

sapien.render.set_log_level("info")
renderer = SapienRenderer()

render_system = RenderSystem()
physx_system = PhysxSystem()
scene = Scene([render_system, physx_system])
render_system.set_ambient_light([0.1, 0.1, 0.1])


def create_ant_builder(scene):
    builder = ArticulationBuilder()
    builder.set_scene(scene)

    copper = VisualMaterialRecord(
        base_color=[0.875, 0.553, 0.221, 1], metallic=1, roughness=0.2
    )

    body = builder.create_link_builder()
    body.add_sphere_collision(Pose(), 0.25)
    body.add_sphere_visual(Pose(), 0.25, copper)
    body.add_capsule_collision(Pose([0.141, 0, 0]), 0.08, 0.141)
    body.add_capsule_visual(Pose([0.141, 0, 0]), 0.08, 0.141, copper)
    body.add_capsule_collision(Pose([-0.141, 0, 0]), 0.08, 0.141)
    body.add_capsule_visual(Pose([-0.141, 0, 0]), 0.08, 0.141, copper)
    body.add_capsule_collision(
        Pose([0, 0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141
    )
    body.add_capsule_visual(
        Pose([0, 0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141, copper
    )
    body.add_capsule_collision(
        Pose([0, -0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141
    )
    body.add_capsule_visual(
        Pose([0, -0.141, 0], aa([0, 0, 1], np.pi / 2)), 0.08, 0.141, copper
    )
    body.set_name("body")

    l1 = builder.create_link_builder(body)
    l1.set_name("l1")
    l1.set_joint_name("j1")
    l1.set_joint_properties(
        "revolute",
        [[-0.5236, 0.5236]],
        Pose([0.282, 0, 0], [0.7071068, 0, 0.7071068, 0]),
        Pose([0.141, 0, 0], [-0.7071068, 0, 0.7071068, 0]),
        0.1,
    )
    l1.add_capsule_collision(Pose(), 0.08, 0.141)
    l1.add_capsule_visual(Pose(), 0.08, 0.141, copper)

    l2 = builder.create_link_builder(body)
    l2.set_name("l2")
    l2.set_joint_name("j2")
    l2.set_joint_properties(
        "revolute",
        [[-0.5236, 0.5236]],
        Pose([-0.282, 0, 0], [0, -0.7071068, 0, 0.7071068]),
        Pose([0.141, 0, 0], [-0.7071068, 0, 0.7071068, 0]),
        0.1,
    )
    l2.add_capsule_collision(Pose(), 0.08, 0.141)
    l2.add_capsule_visual(Pose(), 0.08, 0.141, copper)

    l3 = builder.create_link_builder(body)
    l3.set_name("l3")
    l3.set_joint_name("j3")
    l3.set_joint_properties(
        "revolute",
        [[-0.5236, 0.5236]],
        Pose([0, 0.282, 0], [0.5, -0.5, 0.5, 0.5]),
        Pose([0.141, 0, 0], [0.7071068, 0, -0.7071068, 0]),
        0.1,
    )
    l3.add_capsule_collision(Pose(), 0.08, 0.141)
    l3.add_capsule_visual(Pose(), 0.08, 0.141, copper)

    l4 = builder.create_link_builder(body)
    l4.set_name("l4")
    l4.set_joint_name("j4")
    l4.set_joint_properties(
        "revolute",
        [[-0.5236, 0.5236]],
        Pose([0, -0.282, 0], [0.5, 0.5, 0.5, -0.5]),
        Pose([0.141, 0, 0], [0.7071068, 0, -0.7071068, 0]),
        0.1,
    )
    l4.add_capsule_collision(Pose(), 0.08, 0.141)
    l4.add_capsule_visual(Pose(), 0.08, 0.141, copper)

    f1 = builder.create_link_builder(l1)
    f1.set_name("f1")
    f1.set_joint_name("j11")
    f1.set_joint_properties(
        "revolute",
        [[0.5236, 1.222]],
        Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
        Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]),
        0.1,
    )
    f1.add_capsule_collision(Pose(), 0.08, 0.282)
    f1.add_capsule_visual(Pose(), 0.08, 0.282, copper)

    f2 = builder.create_link_builder(l2)
    f2.set_name("f2")
    f2.set_joint_name("j21")
    f2.set_joint_properties(
        "revolute",
        [[0.5236, 1.222]],
        Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
        Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]),
        0.1,
    )
    f2.add_capsule_collision(Pose(), 0.08, 0.282)
    f2.add_capsule_visual(Pose(), 0.08, 0.282, copper)

    f3 = builder.create_link_builder(l3)
    f3.set_name("f3")
    f3.set_joint_name("j31")
    f3.set_joint_properties(
        "revolute",
        [[0.5236, 1.222]],
        Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
        Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]),
        0.1,
    )
    f3.add_capsule_collision(Pose(), 0.08, 0.282)
    f3.add_capsule_visual(Pose(), 0.08, 0.282, copper)

    f4 = builder.create_link_builder(l4)
    f4.set_name("f4")
    f4.set_joint_name("j41")
    f4.set_joint_properties(
        "revolute",
        [[0.5236, 1.222]],
        Pose([-0.141, 0, 0], [0, 0.7071068, 0.7071068, 0]),
        Pose([0.282, 0, 0], [0, 0.7071068, 0.7071068, 0]),
        0.1,
    )
    f4.add_capsule_collision(Pose(), 0.08, 0.282)
    f4.add_capsule_visual(Pose(), 0.08, 0.282, copper)

    return builder


physical_material = PhysicalMaterialRecord(0.3, 0.35, 0.1)
ground = (
    ActorBuilder()
    .set_scene(scene)
    .add_plane_visual(
        Pose(q=[0.7071068, 0, -0.7071068, 0]),
        [10, 10, 10],
        VisualMaterialRecord(base_color=[1, 1, 1, 1]),
        "",
    )
    .add_plane_collision(Pose(q=[0.7071068, 0, -0.7071068, 0]), physical_material)
    .set_physx_body_type("static")
    .build()
)
ground.name = "ground"

# box = (
#     ActorBuilder()
#     .set_scene(scene)
#     .add_box_visual(
#         Pose([0, 0, 0.1]),
#         [0.05, 0.05, 0.1],
#         VisualMaterialRecord(base_color=[1, 0, 0, 1]),
#         "",
#     )
#     .add_box_collision(Pose([0, 0, 0.1]), [0.05, 0.05, 0.1], physical_material)
#     .build()
# )
# box.name = "box"
# box.pose = Pose([0, 0, 0.1])

# sphere = (
#     ActorBuilder()
#     .set_scene(scene)
#     .add_sphere_visual(
#         Pose([0, 0, 0.1]),
#         0.1,
#         VisualMaterialRecord(base_color=[0, 1, 0, 1]),
#         "",
#     )
#     .add_sphere_collision(
#         Pose([0, 0, 0.1]),
#         0.1,
#         physical_material,
#     )
#     .build()
# )
# sphere.name = "sphere"
# sphere.pose = Pose([0, 0.3, 0.2])

# capsule = (
#     ActorBuilder()
#     .set_scene(scene)
#     .add_capsule_visual(
#         Pose([0, 0, 0.1]),
#         0.1,
#         0.2,
#         VisualMaterialRecord(base_color=[0, 0, 1, 1]),
#         "",
#     )
#     .add_capsule_collision(Pose([0, 0, 0.1]), 0.1, 0.2, physical_material)
#     .build()
# )
# capsule.name = "capsule"
# capsule.pose = Pose([0, -0.3, 0.5])


# camera_mount = Entity()
# camera = RenderCameraComponent(512, 512, "../vulkan_shader/ibl")
# camera_mount.add_component(camera)
# scene.add_entity(camera_mount)
# camera_mount.pose = Pose([-1, 0, 0.5])
# camera_mount.name = "camera"






light_mount = Entity()
light = RenderPointLightComponent()
light.color = [0.1, 0.1, 0.1]
light_mount.add_component(light)
scene.add_entity(light_mount)
light_mount.pose = Pose([0, 0, 0.3])
light_mount.name = "light"

builder = create_ant_builder(scene)
ant = builder.build()
ant.root.pose = Pose([0, 0, 1])

viewer = Viewer(resolutions=(1920, 1080), shader_dir="../vulkan_shader/ibl")
viewer.set_scene(scene)
viewer.set_camera_pose(sapien.Pose([-2, 0, 0.5]))

count = 0

while not viewer.closed:
    physx_system.step()
    render_system.step()

    # if count == 60:
    #     scene.remove_articulation(ant)
    print(scene.get_contacts())

    viewer.render()

    count += 1

viewer.close()
