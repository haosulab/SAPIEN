import sapien.core as sapien
from sapien.core import Pose

renderer = sapien.OptifuserRenderer()  # default renderer for sapien
render_control = sapien.OptifuserController(renderer)  # used to control the renderer

sim = sapien.Engine()
sim.set_renderer(renderer)
scene = sim.create_scene()
scene.set_timestep(1 / 60)

scene.add_ground(-1)
scene.set_ambient_light([0.2, 0.2, 0.2])
scene.set_shadow_light([1, -1, -1], [.5, .5, .5])

box_material = sim.create_physical_material(0.5, 0.5, 1)


def build_box(pos, size, color):
    box_builder = scene.create_actor_builder()
    # add physics for box
    box_builder.add_box_shape(Pose(pos), size, box_material, 1000)  # 1 is the density
    # add visual display for box
    box_builder.add_box_visual(Pose(pos), size, color=color)
    b = box_builder.build()
    b.set_damping(0, 2)
    return b


box1 = build_box([0, 2, 0], [.5] * 3, [1, 0, 0])
box2 = build_box([0, -2, 0], [.5] * 3, [0, 1, 0])

# render
render_control.set_current_scene(scene)
render_control.set_camera_position(-15, 0, 8)
render_control.set_camera_rotation(0, -0.5)
render_control.show_window()

print(box1.mass)
print(box2.mass)

d1 = box1.pack()
d2 = box2.pack()

loop = 0
count = 0
# simulation loop
while not render_control.should_quit:
    count += 1

    if count > 60:
        box1.add_force_torque([0, 0, 0], [0, 0, 10000])
        box2.add_force_torque([0, 0, 0], [0, 0, 0])
    if count > 360:
        f = 10000 if loop == 0 else 6000
        box1.add_force_torque([f, 0, 0], [0, 0, 0])
        box2.add_force_torque([f, 0, 0], [0, 0, 0])

    if count > 600:
        count = 0
        loop = 1 - loop
        box1.unpack(d1)
        box2.unpack(d2)

    # simulate physics
    scene.step()
    # update render
    scene.update_render()
    # render the frame
    render_control.render()

scene = None
