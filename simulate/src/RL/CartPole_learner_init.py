import gymnasium as gym

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

# Parallel environments
vec_env = make_vec_env("CartPole-v1", n_envs=1)

model = PPO("MlpPolicy", vec_env, verbose=1)
model.learn(total_timesteps=10)
model.save("ppo_cartpole")

del model # remove to demonstrate saving and loading
#vec_env = make_vec_env("CartPole-v1", n_envs=1)
model = PPO.load("ppo_cartpole")

obs = vec_env.reset()
while True:
    action, _states = model.predict(obs)
    print("action.shape: ",action.shape)
    print("action: ",action)
    obs, rewards, dones, info = vec_env.step(action)
    print("obs.shape:", obs.shape)
    print("obs: ",obs[0])
   
    vec_env.render("human")
