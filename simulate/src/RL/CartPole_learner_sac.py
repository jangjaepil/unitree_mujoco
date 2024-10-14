import gymnasium as gym
from gymnasium.wrappers import TimeLimit
from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env

def make_env():
    env = gym.make('CartPole-v1', max_episode_steps=1500)
    return env

#Create the vectorized environment with the custom time limit applied
vec_env = make_vec_env(make_env, n_envs=1000)
   
model = SAC("MlpPolicy", vec_env,verbose=1)
model.set_parameters("/home/jang/unitree_mujoco/simulate/src/RL/policies/sac_cartpole_dx(9)")

# 1: ent_auto 0.1, step : 10^6(30m), 2: 10^7, 3: revise x position limit 0.4 10^7,  4: add x_dot_limit : 1, 5: (3)+ 3*10^7 with adding - 0.0001 x_dot
# 6: revise x_dot limit 0.8, x position limit 0.35, -0.001 x_dot, 7: expand init theta -1.5pi 1.5pi    8: add random cart position, revise pole_x_error 2*self.length+2*self.x_threshold 
# 9: add cart_vel_limit 1m/s  
model.learn(total_timesteps=pow(10,9),progress_bar=1)
model.save("/home/jang/unitree_mujoco/simulate/src/RL/policies/sac_cartpole_dx(10)")
# i = i +1

# del model # remove to demonstrate saving and loading

# model = SAC.load("/home/jang/unitree_mujoco/simulate/src/RL/policies/sac_cartpole_dx(8)")
# env = gym.make('CartPole-v1', max_episode_steps=1500, render_mode="human")

# obs,info = env.reset()
# init = 1
# while True:
#     action, _states = model.predict(obs)        
#     res = env.step(action)
#     obs, rewards, dones, truncated, info = res
    
#     if dones or init:
#         print("cartPole policy sac ")
#         print(f"Episode terminated: {truncated}")
#         env.reset()
#         print("mass pole: ",env.get_wrapper_attr('masspole'))
#         print("total pole length: ",2*env.get_wrapper_attr('length'))
#         print("state: ",env.get_wrapper_attr("state"))
#         init = 0
        
#     env.render()  
  
