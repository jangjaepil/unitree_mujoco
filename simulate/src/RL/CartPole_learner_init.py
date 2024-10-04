import gymnasium as gym
from gymnasium.wrappers import TimeLimit
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

def make_env():
    env = gym.make('CartPole-v1', max_episode_steps=1500)
    return env

#Create the vectorized environment with the custom time limit applied
vec_env = make_vec_env(make_env, n_envs=10)
   
model = PPO("MlpPolicy", vec_env, verbose=1)
model.set_parameters("/home/jang/unitree_mujoco/simulate/src/RL/policies/ppo_cartpole_dx(18)")

i = 4# 1: 10^4 total steps, 2: 10^5, 3: 10^6, 4: 10^7   5: 10^7 + 10^4 , 6: 10^7 + 10^5, 7: 10^7 + 10^5 + 10^7 , 8: 10^7 + 10^5 + 10^8   total steps  
# 9: add ternimation - error limit( file(8) + 10^5) 10: file(9)-> change error 0.001-> 0.1(file(9) + 10^5), 11: (file(10) + 10^5), 12: (file(10) + 10^6)
# 13 : (file(12) + 10^5) , 14 : (file(12) + 10^6), 15 : (file(14) + 10^6), 16 : (file(14) + 10^7), 17 : add goal reward (file(16) + 10^6), 18 : add goal reward (file(16) + 10^7)
while i < 6:
     
    model.learn(total_timesteps=1000*pow(10,i),progress_bar=1)
    model.save("/home/jang/unitree_mujoco/simulate/src/RL/policies/ppo_cartpole_dx(%d)"%(i+15))
    i = i +1

del model # remove to demonstrate saving and loading

model = PPO.load("/home/jang/unitree_mujoco/simulate/src/RL/policies/ppo_cartpole_dx(19)")
env = gym.make('CartPole-v1', max_episode_steps=1500, render_mode="human")

obs,info = env.reset()
init = 1
while True:
    action, _states = model.predict(obs)        
    res = env.step(action)
    obs, rewards, dones, truncated, info = res
    
    if dones or init:
        print("cartPole policy(%d)"%(14))
        print(f"Episode terminated: {truncated}")
        env.reset()
        print("mass pole: ",env.get_wrapper_attr('masspole'))
        print("state: ",env.get_wrapper_attr("state"))
        init = 0
    env.render()  
  
