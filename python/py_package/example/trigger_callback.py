import sapien.core as sapien
from sapien.core import Pose
from sapien.utils import Viewer

engine = sapien.Engine()
renderer = sapien.VulkanRenderer()
engine.set_renderer(renderer)
viewer = Viewer(renderer)

scene = engine.create_scene()
viewer.set_scene(scene)
scene.add_ground(-1)
scene.set_timestep(1 / 60)

scene.set_ambient_light([0.5, 0.5, 0.5])

rs: sapien.VulkanScene = scene.get_render_scene()
light = rs.add_shadow_directional_light([0, 1, -1], [0.5, 0.5, 0.5])


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


def trigger(self, other, trigger):
    if other.name == "ball":
        scene.remove_actor(self)


b1.on_trigger(trigger)
b2.on_trigger(trigger)
b3.on_trigger(trigger)


viewer.set_camera_xyz(-10, 0, 0)
viewer.window.set_camera_parameters(0.1, 1000, 1)

scene.update_render()
scene.step()
viewer.render()

while not viewer.closed:
    scene.update_render()
    scene.step()
    viewer.render()
