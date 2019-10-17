from gym.envs.registration import register

register(
    id='SapyenAnt-v0',
    entry_point='gym_sapyen.envs:AntEnv',
)

register(
    id='SapyenHalfCheetah-v0',
    entry_point='gym_sapyen.envs:HalfCheetahEnv',
)