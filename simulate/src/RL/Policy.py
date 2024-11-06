# import gymnasium as gym
# import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env



model = SAC.load("/home/jang/unitree_mujoco/simulate/src/RL/policies/mujoco_cartPole(23)")
        
def get_action(obs):
    action, _states = model.predict(obs)  
    return action

