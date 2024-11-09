import gymnasium as gym
from gymnasium.wrappers import TimeLimit
from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env
import numpy as np
import matplotlib.pyplot as plt
import time

model = SAC.load("/home/jang/unitree_mujoco/simulate/src/RL/policies/mujoco_cartPole(23)")
env = gym.make('InvertedPendulum-v4', max_episode_steps=1500, render_mode="human")
#print('model and env imported')sdfsd
#obs = np.array([0,0,0,0,0])
#env.reset_state(obs)
obs,info = env.reset()
#print("state",obs)
max_pos = 0
max_vel = 0
init = 1
i = 1
action_pos = []
action_vel = []
cart_pos = []

cart_vel = []
pole_angle_vel = []
filterd_action_pos = []
filterd_action_vel = []
pole_angle = []

frequency = 1  # Frequency in Hz (one cycle per second)
 

while i<200:
    action, _states = model.predict(obs)        
    # print("action: ",action)
    cart_pos.append(obs[0])
    cart_vel.append(obs[2])
    pole_angle.append(obs[1])
    pole_angle_vel.append(obs[3])
    action_pos.append(action[0])
    action_vel.append(action[1])
    
    
    res = env.step(action)
    
    filterd_action_pos.append(env.output[0,0])
    filterd_action_vel.append(env.output[1,0])
    
    obs, rewards, dones, truncated, info = res
    # if dones:
    #     break
    env.render()
    i = i + 1 

action_pos = np.array(action_pos)
actions = np.array(action_vel)

cart_pos = np.array(cart_pos)
cart_vel = np.array(cart_vel)
pole_angle = np.asarray(pole_angle)
pole_angle_vel = np.asarray(pole_angle_vel)
time_steps = np.arange(len(actions))

plt.figure(figsize=(10, 5))

# Plot actions
# plt.plot(time_steps, action_pos, label='Action(desired position)', color='green')
# plt.plot(time_steps, filterd_action_pos, label='filtered desired position', color='blue')
# plt.plot(time_steps, cart_pos, label='Cart Position', color='red')


# plt.title('Action and filtered action Over Time')
# plt.xlabel('Time Steps(1 step = 0.02s)')
# plt.ylabel('Value[m]')
# plt.grid(True)
# plt.legend()

# # Show the plot
# plt.show()   
    
# plt.plot(time_steps, action_vel, label='Action(desired velocity)', color='red')
# plt.plot(time_steps, filterd_action_vel, label='filtered desired velocity', color='blue')
# plt.plot(time_steps, cart_vel, label='Cart Velocity', color='green')



# plt.title('Action and filtered action Over Time')
# plt.xlabel('Time Steps(1 step = 0.02s)')
# plt.ylabel('Value[m/s]')
# plt.grid(True)
# plt.legend()
 
# plt.show()  


plt.plot(time_steps, cart_pos, label='Cart Position', color='red')
plt.plot(time_steps, cart_vel, label='Cart Velocity', color='green')
plt.plot(time_steps, pole_angle, label='Pole Angle', color='blue')
plt.plot(time_steps, pole_angle_vel, label='Pole Angular Velocity', color='orange')


plt.title('Trajectory of Cart and Pole Over Time', fontsize = 30)
plt.xlabel('Time Steps(1 step = 0.02s)', fontsize = 30)
plt.ylabel('Values[m,m/s,rad,rad/s]', fontsize = 30)
plt.grid(True)
plt.legend(prop={'size': 25})
plt.xticks(fontsize=30)
plt.yticks(fontsize=30) 
plt.show()  
