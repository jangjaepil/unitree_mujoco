import math
from typing import Optional, Tuple
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

class customCartPoleEnv(gym.Env):
    def __init__(self, sutton_barto_reward: bool = False, render_mode: Optional[str] = None):
        super(customCartPoleEnv, self).__init__()
        self.env = gym.make('CartPole-v1', render_mode=render_mode)
        self.render_mode = render_mode
        self._sutton_barto_reward = sutton_barto_reward
        
        # Custom dynamics settings
        self.env.__setattr__("gravity", 9.8)
        self.env.__setattr__("masspole", 0.1)
        self.env.__setattr__("masscart", 1000.0)
        self.env.__setattr__("total_mass", self.env.__getattribute__("masspole") + self.env.__getattribute__("masscart"))
        self.env.__setattr__("length", 0.5)  # half the pole's length
        self.env.__setattr__("polemass_length", self.env.__getattribute__("masspole") * self.env.__getattribute__("length"))
        self.env.__setattr__("kinematics_integrator", "euler")
        
        # Action and observation spaces
        self.min_action = -10
        self.max_action = 10
        self.max_obs = np.array([5, 3.4028235e+38, 1.57, 3.4028235e+38])
        self.min_obs = -self.max_obs
        self.env.__setattr__("tau", 0.02)  # seconds between state updates
        self.env.__setattr__("theta_threshold_radians", 12 * 2 * math.pi / 360)
        self.x_threshold = 2.4
        
        self.action_space = spaces.Box(low=self.min_action, high=self.max_action, shape=(1,), dtype=np.float32)
        self.observation_space = spaces.Box(low=self.min_obs, high=self.max_obs, shape=(4,), dtype=np.float32)

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, dict]:
        assert self.action_space.contains(action), f"{action!r} ({type(action)}) invalid"
        assert self.env.__getattribute__("state") is not None, "Call reset before using step method."

        x, x_dot, theta, theta_dot = self.env.__getattribute__("state")
        
        # Force calculations
        force = action
        costheta = np.cos(theta)
        sintheta = np.sin(theta)
        temp = (force + self.env.__getattribute__("polemass_length") * np.square(theta_dot) * sintheta) / self.env.__getattribute__("total_mass")
        thetaacc = (self.env.__getattribute__("gravity") * sintheta - costheta * temp) / (
            self.env.__getattribute__("length") * (4.0 / 3.0 - self.env.__getattribute__("masspole") * np.square(costheta) / self.env.__getattribute__("total_mass"))
        )
        xacc = temp - self.env.__getattribute__("polemass_length") * thetaacc * costheta / self.env.__getattribute__("total_mass")

        # Euler's method for updating states
        if self.env.__getattribute__("kinematics_integrator") == "euler":
            x = x + self.env.__getattribute__("tau") * x_dot
            x_dot = x_dot + self.env.__getattribute__("tau") * xacc
            theta = theta + self.env.__getattribute__("tau") * theta_dot
            theta_dot = theta_dot + self.env.__getattribute__("tau") * thetaacc

        states = np.array([float(x), float(x_dot), float(theta), float(theta_dot)], dtype=np.float64)
        self.env.__setattr__("state", states)
        
        # Termination conditions
        terminated = bool(
            x < -self.x_threshold
            or x > self.x_threshold
            or theta < -self.env.__getattribute__("theta_threshold_radians")
            or theta > self.env.__getattribute__("theta_threshold_radians")
        )

        # Reward system
        reward = 1.0 if not terminated else -1.0 if self._sutton_barto_reward else 0.0
        
        if self.render_mode == "human":
            self.render()

        return np.array(self.env.__getattribute__("state"), dtype=np.float32), reward, terminated, False, {}

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        self.env.reset()
        low, high = -0.05, 0.05
        random_state = self.env.np_random.uniform(low=low, high=high, size=(4,))
        self.env.__setattr__("state", random_state)
        self.env.__setattr__("steps_beyond_terminated", None)
        if self.render_mode == "human":
            self.render()
        return np.array(self.env.__getattribute__("state"), dtype=np.float32), {}

    def render(self):
        return self.env.render()

    def close(self):
        self.env.close()

# Create the environment
env = customCartPoleEnv(render_mode="human")

# Create a PPO model for the environment
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=5000)
model.save("ppo_cartpole")

# Load the model and use it for inference
model = PPO.load("ppo_cartpole")

# Run the environment with the trained model
obs, _ = env.reset()
while True:
    print("obs: ",obs)
    
    action, _ = model.predict(obs)
    print("action: ",action)
    obs, rewards, dones, _, _ = env.step(action)
    env.render()
    if dones:
        obs, _ = env.reset()
