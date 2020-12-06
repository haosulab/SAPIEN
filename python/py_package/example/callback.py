import sapien.core as sapien
from sapien.core import Pose

engine = sapien.Engine()
renderer = sapien.VulkanRenderer()
engine.set_renderer(renderer)
controller = sapien.VulkanController(renderer)

scene = engine.create_scene()
controller.set_current_scene(scene)
scene.add_ground(-1)
scene.set_timestep(1 / 60)

scene.set_ambient_light([0.5, 0.5, 0.5])
scene.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])


def contact_callback(self, other, contact):
    pass


def add_box(z):
    builder = scene.create_actor_builder()
    builder.add_box_shape()
    builder.add_box_visual()
    box = builder.build()
    box.set_name("box")
    box.set_pose(Pose([0, 0, z]))
    return box


def add_ball(z):
    builder = scene.create_actor_builder()
    builder.add_sphere_shape()
    builder.add_sphere_visual()
    ball = builder.build()
    ball.set_name("ball")
    ball.set_pose(Pose([0, 0, z]))
    return ball


add_box(0)
add_box(2)
add_box(4)
ball = add_ball(8)


def contact(self: sapien.ActorBase, other: sapien.ActorBase, contact: sapien.Contact):
    if other.name == "box" and any(
        [abs(x) > 1e-6 for p in contact.points for x in p.impulse]
    ):
        scene.remove_actor(other)


ball.on_contact(contact)


controller.set_free_camera_position(-10, 0, 0)

while not controller.is_closed:
    scene.update_render()
    scene.step()
    controller.render()
