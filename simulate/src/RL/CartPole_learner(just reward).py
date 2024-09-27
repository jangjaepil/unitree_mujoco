import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from gymnasium import Wrapper

# Custom wrapper to modify reward and termination condition
class CustomEnvWrapper(Wrapper):
    def __init__(self, env):
        super(CustomEnvWrapper, self).__init__(env)
    
    def step(self, action):
        # Take a step in the environment
        obs, reward, done, info = self.env.step(action)
        
        # Modify the reward (example: double the reward)
        modified_reward = reward * 2
        
        # Modify the termination condition (example: force end after 200 steps)
        if 'num_steps' not in info:
            info['num_steps'] = 0
        info['num_steps'] += 1
        
        if info['num_steps'] >= 200:  # Custom termination condition
            done = True
        
        return obs, modified_reward, done, info

# Create a vectorized environment with a seed
env = make_vec_env("CartPole-v1", n_envs=1, seed=42)

# Wrap the environment with CustomEnvWrapper
modified_env = CustomEnvWrapper(env)

# Create and train the model
model = PPO("MlpPolicy", modified_env, verbose=1)
model.learn(total_timesteps=25000)
model.save("ppo_cartpole")

del model  # remove to demonstrate saving and loading

# Load the trained model
model = PPO.load("ppo_cartpole")

# Reset without the seed argument (vectorized envs don't use it in reset)
obs = modified_env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = modified_env.step(action)
    modified_env.render("human")
    
    # Exit the loop if the episode is done
    if dones:
        break
