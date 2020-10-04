import gym
import gym_sapyen
import time
import numpy as np
env = gym.make('SapyenAnt-v0')
start_time = time.perf_counter()
sum_reward = 0
env.renderer.show_window()
for i_episode in range(100):
    observation = env.reset()
    for t in range(100):
        env.render()
        action = env.action_space.sample() 
        observation, reward, done, info = env.step(action)
        sum_reward += info['reward_forward']
        #print(observation.shape)
        #if done:
        #    print("Episode finished after {} timesteps".format(t+1))
        #    break
end_time = time.perf_counter()
#print(sum_reward)
#print(end_time - start_time)
env.close()