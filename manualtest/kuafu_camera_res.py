import sapien.core as sapien

# renderer = sapien.VulkanRenderer()
config = sapien.KuafuConfig()
config.use_denoiser = True
renderer = sapien.KuafuRenderer(config)
sapien.KuafuRenderer.set_log_level('debug')

engine = sapien.Engine()
engine.set_renderer(renderer)

config = sapien.SceneConfig()
scene = engine.create_scene(config)

for i in range(100000):
    print(i)
    cam = scene.add_camera("cam", 512, 512, 1, 0.1, 10)
    scene.update_render()
    cam.take_picture()
    # cam.get_color_rgba()
    scene.remove_camera(cam)
