import gymnasium as gym
from gymnasium.wrappers import TimeLimit
from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env
import numpy as np


model = SAC.load("/home/jang/unitree_mujoco/simulate/src/RL/policies/sac_cartpole_dx(9)")
env = gym.make('CartPole-v1', max_episode_steps=1500, render_mode="human")

# obs = np.array([0,0,12,0,0])
# env.reset_state(obs)
obs,info = env.reset()
print("state",obs)
max_pos = 0
max_vel = 0
init = 1
i = 100
while i<10000:
    action, _states = model.predict(obs)        
    res = env.step(action)
    obs, rewards, dones, truncated, info = res
      #print("action: ",action)
    print("obs shape: ",obs.shape)
    print("obs type: ",type(obs))
    print("mass pole: ",env.get_wrapper_attr('masspole'))
    print("state: ",env.get_wrapper_attr("state"))
    print("max_pos: ",max_pos)
    print("max_Vel: ",max_vel)
    if abs(obs[0]) >= abs(max_pos):
        max_pos  = obs[0]    
    if abs(obs[1]) >= abs(max_vel):
        max_vel  = obs[1]    
    
    if dones or init:
        print("cartPole policy_dx(10)")
        print(f"Episode terminated: {truncated}")
        env.reset()
        
        print("mass pole: ",env.get_wrapper_attr('masspole'))
        print("state: ",env.get_wrapper_attr("state"))
        init = 0
        # if dones:
        #     break
    i = i + 1    
    env.render() 
  
