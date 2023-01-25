import sapien.core as sapien
from sapien.utils import Viewer
import numpy as np


def main():
    engine = sapien.Engine()  # Create a physical simulation engine
    renderer = sapien.SapienRenderer()  # Create a Vulkan renderer

    renderer.set_log_level("info")

    # renderer.set_default_texture_format({
    #     "texture_format": {
    #         "Color": "4u1"
    #     }
    # })

    engine.set_renderer(renderer)  # Bind the renderer and the engine

    scene = engine.create_scene()  # Create an instance of simulation world (aka scene)
    scene.set_timestep(1 / 100.0)  # Set the simulation frequency

    # NOTE: How to build actors (rigid bodies) is elaborated in create_actors.py
    scene.add_ground(altitude=0)  # Add a ground
    actor_builder = scene.create_actor_builder()
    actor_builder.add_box_collision(half_size=[0.5, 0.5, 0.5])
    actor_builder.add_box_visual(half_size=[0.5, 0.5, 0.5], color=[1.0, 0.0, 0.0])
    box = actor_builder.build(name="box")  # Add a box
    box.set_pose(sapien.Pose(p=[0, 0, 0.5]))
    box.set_pose(sapien.Pose(p=[0, 0, 2.0]))

    # Add some lights so that you can observe the scene
    scene.set_ambient_light([0.5, 0.5, 0.5])
    # scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [1, 1, 1])

    viewer = Viewer(renderer)
    viewer.set_scene(scene)  # Bind the viewer and the scene
    # viewer.window.set_camera_property("spp", 32)

    # The coordinate frame in Sapien is: x(forward), y(left), z(upward)
    # The principle axis of the camera is the x-axis
    viewer.set_camera_xyz(x=-4, y=0, z=2)
    # The rotation of the free camera is represented as [roll(x), pitch(-y), yaw(-z)]
    # The camera now looks at the origin
    viewer.set_camera_rpy(r=0, p=-np.arctan2(2, 4), y=0)
    viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)

    camera = scene.add_camera("", 512, 512, 1, 0.01, 10)
    camera.set_local_pose(sapien.Pose([-10, 0, 2]))
    scene.step()
    scene.update_render()
    camera.take_picture()

    from torch.utils.dlpack import from_dlpack

    color = from_dlpack(camera.get_dl_tensor("Color")).cpu().numpy()

    import matplotlib.pyplot as plt

    plt.imshow(color)
    plt.show()

    while not viewer.closed:  # Press key q to quit
        scene.step()  # Simulate the world
        scene.update_render()  # Update the world to the renderer
        viewer.render()


if __name__ == "__main__":
    main()
