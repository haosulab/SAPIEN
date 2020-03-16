import sapien.core as sapien

engine = sapien.Engine()
renderer = sapien.OptifuserRenderer()
engine.set_renderer(renderer)

scene0 = engine.create_scene(gravity=[0, 0, -9.81])
scene0.set_timestep(1 / 240)

renderer_controller = sapien.OptifuserController(renderer)
renderer_controller.set_current_scene(scene0)
renderer_controller.show_window()

renderer_controller.set_camera_position(-4, 0, 2)
renderer_controller.set_camera_rotation(0, -0.5)
scene0.set_ambient_light([0.5, 0.5, 0.5])
scene0.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

loader = scene0.create_urdf_loader()
articulated_object = loader.load("assets/179/mobility.urdf")

while not renderer_controller.should_quit:
    scene0.update_render()
    scene0.step()
    renderer_controller.render()

scene0 = None