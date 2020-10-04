from gym.envs.registration import register

register(
    id='SapyenAnt-v0',
    entry_point='gym_sapyen.envs:AntEnv',
    max_episode_steps=1000,
    reward_threshold=6000.0,
)

register(
    id='SapyenHalfCheetah-v0',
    entry_point='gym_sapyen.envs:HalfCheetahEnv',
    max_episode_steps=1000,
    reward_threshold=4800.0,
)