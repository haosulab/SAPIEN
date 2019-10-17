import gym
import gym_sapyen
import time
import numpy as np
Sapyen = False

if Sapyen:
    env = gym.make('SapyenHalfCheetah-v0')
else:
    env = gym.make('HalfCheetah-v2')

sum_reward = 0
if Sapyen:
    env.renderer.show_window()
actions = np.load('actions.npy')
env.seed(0)
start_time = time.perf_counter()
for i_episode in range(100):
    observation = env.reset()
    for t in range(100):
        env.render()
        action = actions[i_episode, t] #env.action_space.sample() 
        if Sapyen:
            action = action * [120, 90, 60, 120, 60, 30]
        observation, reward, done, info = env.step(action)
        sum_reward += info['reward_run']
        #print(observation.shape)
        #if done:
        #    print("Episode finished after {} timesteps".format(t+1))
        #    break
    print(sum_reward)
    sum_reward = 0
end_time = time.perf_counter()
#print(sum_reward)
#print(end_time - start_time)
env.close()