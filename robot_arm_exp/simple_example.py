from env.robot_arm_env import RobotArmEnv, sapyen


def main():
    env = RobotArmEnv(on_screen_render=True)
    for j in range(3):
        for i in range(1000):
            action = env.action_space.sample()
            env.step(action)
            env.renderer.render()
        env.reset()


if __name__ == '__main__':
    main()
