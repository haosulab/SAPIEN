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
    builder.add_box_shape(is_trigger=True)
    builder.add_box_visual()
    box = builder.build(True)
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


b1 = add_box(0)
b2 = add_box(2)
b3 = add_box(4)
ball = add_ball(8)


def step(self, dt):
    print(type(self))


def trigger(self, other, trigger):
    if other.name == "ball":
        scene.remove_actor(self)


b1.on_trigger(trigger)
b2.on_trigger(trigger)
b3.on_trigger(trigger)


controller.set_free_camera_position(-10, 0, 0)

while not controller.is_closed:
    scene.update_render()
    scene.step()
    controller.render()
