import pysapien as sapien

engine = sapien.Engine()
scene0 = engine.create_scene(gravity=[0, 0, -9.81])
scene0.set_timestep(1 / 240)

renderer = sapien.OptifuserRenderer()
renderer.enable_global_axes(False)
scene0.set_ambient_light([0.5, 0.5, 0.5])
scene0.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])
engine.set_renderer(renderer)

renderer_controller = sapien.OptifuserController(renderer)
renderer_controller.set_current_scene(scene0)
renderer_controller.set_camera_position(-2, 0, 1)
renderer_controller.set_camera_rotation(0, -0.5)
renderer_controller.show_window()

loader = scene0.create_urdf_loader()
articulated_object = loader.load("assets/179/mobility.urdf")

while not renderer_controller.should_quit:
    scene0.update_render()
    scene0.step()
    renderer_controller.render()
