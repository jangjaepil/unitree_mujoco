import gymnasium as gym
from gymnasium.wrappers import TimeLimit
from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env

def make_env():
    env = gym.make('CartPole-v1', max_episode_steps=1500)
    return env

#Create the vectorized environment with the custom time limit applied
vec_env = make_vec_env(make_env, n_envs=10)
   
model = SAC("MlpPolicy", vec_env,verbose=1)
model.set_parameters("/home/jang/unitree_mujoco/simulate/src/RL/policies/sac_cartpole_dx(3)")

# 1: ent_auto 0.1, step : 10^6(30m), 2: 10^7, 3: revise x position limit 10^7,  4: add x_dot_limit : 1
     
model.learn(total_timesteps=0.3*pow(10,7),progress_bar=1)
model.save("/home/jang/unitree_mujoco/simulate/src/RL/policies/sac_cartpole_dx(5)")
# i = i +1

del model # remove to demonstrate saving and loading

model = SAC.load("/home/jang/unitree_mujoco/simulate/src/RL/policies/sac_cartpole_dx(5)")
env = gym.make('CartPole-v1', max_episode_steps=1500, render_mode="human")

obs,info = env.reset()
init = 1
while True:
    action, _states = model.predict(obs)        
    res = env.step(action)
    obs, rewards, dones, truncated, info = res
    
    if dones or init:
        print("cartPole policy sac ")
        print(f"Episode terminated: {truncated}")
        env.reset()
        print("mass pole: ",env.get_wrapper_attr('masspole'))
        print("total pole length: ",2*env.get_wrapper_attr('length'))
        print("state: ",env.get_wrapper_attr("state"))
        init = 0
    env.render()  
  
