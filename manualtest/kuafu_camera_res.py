import sapien.core as sapien

# renderer = sapien.VulkanRenderer()
renderer = sapien.KuafuRenderer()
sapien.KuafuRenderer.set_log_level('debug')

engine = sapien.Engine()
engine.set_renderer(renderer)

config = sapien.SceneConfig()
scene = engine.create_scene(config)

for i in range(1000):
    print(i)
    cam = scene.add_camera("cam", 512, 512, 1, 0.1, 10)
    scene.update_render()
    cam.take_picture()
    scene.remove_camera(cam)
