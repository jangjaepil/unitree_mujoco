import gymnasium as gym
from gymnasium.wrappers import TimeLimit
from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env



model = SAC.load("/home/jang/unitree_mujoco/simulate/src/RL/policies/sac_cartpole_dx(5)")
env = gym.make('CartPole-v1', max_episode_steps=1500, render_mode="human")

obs,info = env.reset()
init = 1
while True:
    action, _states = model.predict(obs)        
    res = env.step(action)
    obs, rewards, dones, truncated, info = res
    print("mass pole: ",env.get_wrapper_attr('masspole'))
    print("state: ",env.get_wrapper_attr("state"))
        
    if dones or init:
        print("cartPole policy_dx(10)")
        print(f"Episode terminated: {truncated}")
        env.reset()
        print("mass pole: ",env.get_wrapper_attr('masspole'))
        print("state: ",env.get_wrapper_attr("state"))
        init = 0
    env.render() 
  
