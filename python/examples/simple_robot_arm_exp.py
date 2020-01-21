import sapien
from sapien.env.robot_arm_env import RobotArmEnv


# Uncomment below to enable OpenGL 4
# sapien.core.enable_gl4()


def main():
    on_screen = True
    sim = sapien.core.Simulation()
    renderer = sapien.core.OptifuserRenderer()
    sim.set_renderer(renderer)

    env1 = RobotArmEnv(sim)
    if on_screen:
        on_screen_rendering_controller = sapien.core.OptifuserController(renderer)
        on_screen_rendering_controller.show_window()
        on_screen_rendering_controller.set_current_scene(env1.scene)
        on_screen_rendering_controller.camera.set_position([-3, 0, 2])

    while not on_screen_rendering_controller.should_quit:
        for i in range(1000):
            action = env1.action_space.sample()
            obs = env1.step(action)
            env1.update_renderer()
            if on_screen:
                on_screen_rendering_controller.render()
        env1.reset()


if __name__ == '__main__':
    main()
