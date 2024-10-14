import gymnasium as gym
import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env
import warnings
warnings.filterwarnings("ignore")

#vec_env = make_vec_env("CartPole-v1", n_envs=1)
env = gym.make('CartPole-v1')
model = SAC.load("/home/jang/unitree_mujoco/simulate/src/RL/policies/sac_cartpole_dx(9)")
predition = 1


#print("fake_obs type: ",type(fake_obs))
def get_action(obs):
    #print("current_state: ",obs)
    env.reset_state(obs)
    obs,info = env.reset()
    
    i = 0
    cart_vel = []
    cart_pos = []
    
    while (i < predition):
        #print("obs: ",obs)
        
        action, _states = model.predict(obs)  
        #print("action: ",action)
        #print("action shape: ",action.shape)
        #print("action type: ",type(action))
        obs, rewards, dones,truncated, info = env.step(action)
        i =i + 1
        cart_pos.append(obs[0])
        cart_vel.append(obs[1])
    #print("cart_pos: ",cart_pos)
    #print("cart_vel: ",cart_vel)
    trajectory = np.array(cart_pos + cart_vel) 
    #print("trajector: ",trajectory)
    return trajectory

