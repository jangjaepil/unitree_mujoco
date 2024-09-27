import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
import warnings
warnings.filterwarnings("ignore")

vec_env = make_vec_env("CartPole-v1", n_envs=1)
model = PPO.load("/home/jang/unitree_mujoco/simulate/src/RL/ppo_cartpole")

fake_obs = vec_env.reset()
print("fake_obs type: ",type(fake_obs))
def get_action(obs):
    print("obs: ",obs)
    print("obs type: ",type(obs))
    vec_env.__setattr__("state",obs)
    action, _states = model.predict(obs)
    print("action: ",action)
    print("action shape: ",action.shape)
    print("action type: ",type(action))
   
    action2D = action.reshape(1,1)
    print("actiont2D: ",action2D)
    print("actiont2D shape: ",action2D.shape)
   
    state, rewards, dones, info = vec_env.step(action2D)
    
    return state

