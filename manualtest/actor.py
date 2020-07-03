import sapien.core as sapien
from sapien.core import Pose

sim = sapien.Engine()
renderer = sapien.OptifuserRenderer()
sim.set_renderer(renderer)
controller = sapien.OptifuserController(renderer)

controller.show_window()

s0 = sim.create_scene()
s0.add_ground(-1)
s0.set_timestep(1 / 60)

s0.set_ambient_light([0.5, 0.5, 0.5])
s0.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

builder = s0.create_actor_builder()
# builder.add_box_shape()
# builder.add_box_visual()
builder.add_multiple_convex_shapes_from_file('/home/fx/source/sapien/assets/bottle/model.obj',
                                             scale=[0.1,0.1,0.1])
builder.add_visual_from_file('/home/fx/source/sapien/assets/bottle/model.obj', scale=[0.1,0.1,0.1])
actor = builder.build()
for s in actor.get_collision_shapes():
    print(s.convex_mesh_geometry.scale)

actor.set_pose(Pose([0, 0, 2]))

controller.set_current_scene(s0)

while not controller.should_quit:
    s0.update_render()
    s0.step()
    controller.render()
