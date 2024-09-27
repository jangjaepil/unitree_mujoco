import math
from typing import Optional, Tuple, Union

import gymnasium as gym
from gymnasium import spaces
from gymnasium.envs.classic_control import utils

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

class customCartPoleEnv(gym.Env):
    def __init__(self,sutton_barto_reward: bool = False, render_mode: Optional[str] = None):
        super(customCartPoleEnv, self).__init__()
        self.env = gym.make('CartPole-v1',render_mode= "human")
        self._sutton_barto_reward = sutton_barto_reward
        self.gravity = 9.8
        self.masspole = 0.1
        self.masscart = 1.0
        self.total_mass = self.masspole + self.masscart
        self.length = 0.5  # actually half the pole's length
        self.polemass_length = self.masspole * self.length
        self.kinematics_integrator = "euler"
        self.min_action = -0.01
        self.max_action = 0.01
        self.max_obs = np.array([5, 3.4028235e+38, 1.57, 3.4028235e+38])
        self.min_obs = -self.max_obs
        self.tau = 0.02  # seconds between state updates
        # Angle at which to fail the episode
        self.theta_threshold_radians = 50 * 2 * math.pi / 360
        self.x_threshold = 50
        self.state: np.ndarray | None = None
        
        self.action_space = spaces.Box(
            low=self.min_action, high=self.max_action, shape=(1,), dtype=np.float32
        )
        
        self.observation_space = spaces.Box(
            low=self.min_obs, high=self.max_obs, shape=(4,), dtype=np.float32
        )
        
    def reset(self, *, seed=None, options=None):
        # Reset the environment, passing the seed if provided
        return self.env.reset(seed=seed, options=options)

    def step(self, action):
        
      
        x, x_dot, theta, theta_dot = self.state
        force = action
        costheta = np.cos(theta)
        sintheta = np.sin(theta)
        
        temp = (
            force + self.polemass_length * np.square(theta_dot) * sintheta
        ) / self.total_mass
        thetaacc = (self.gravity * sintheta - costheta * temp) / (
            self.length
            * (4.0 / 3.0 - self.masspole * np.square(costheta) / self.total_mass)
        )
        xacc = temp - self.polemass_length * thetaacc * costheta / self.total_mass

        if self.kinematics_integrator == "euler":
            x = x + self.tau * x_dot
            x_dot = x_dot + self.tau * xacc
            theta = theta + self.tau * theta_dot
            theta_dot = theta_dot + self.tau * thetaacc
        else:  # semi-implicit euler
            x_dot = x_dot + self.tau * xacc
            x = x + self.tau * x_dot
            theta_dot = theta_dot + self.tau * thetaacc
            theta = theta + self.tau * theta_dot

        self.state = np.array((float(x), float(x_dot), float(theta), float(theta_dot)), dtype=np.float64)

        terminated = bool(
            x < -self.x_threshold
            or x > self.x_threshold
            or theta < -self.theta_threshold_radians
            or theta > self.theta_threshold_radians
        )

        if not terminated:
            reward = 0.0 if self._sutton_barto_reward else 1.0
        elif self.steps_beyond_terminated is None:
            # Pole just fell!
            self.steps_beyond_terminated = 0

            reward = -1.0 if self._sutton_barto_reward else 1.0
        else:
            if self.steps_beyond_terminated == 0:
                logger.warn(
                    "You are calling 'step()' even though this environment has already returned terminated = True. "
                    "You should always call 'reset()' once you receive 'terminated = True' -- any further steps are undefined behavior."
                )
            self.steps_beyond_terminated += 1

            reward = -1.0 if self._sutton_barto_reward else 0.0
        
        # pole_angle_penalty = -10*abs(theta)  # Penalize larger angles

        # cart_position_penalty = -1*abs(x)  # Penalize for being far from the center
        # cart_vel_penalty = 1000*abs(x_dot)
        # time_step_reward = 1.0  # Survive bonus per step

        # # Total custom reward
        custom_reward = reward #+ cart_vel_penalty 
        if self.render_mode == "human":
            self.render()

        # truncation=False as the time limit is handled by the `TimeLimit` wrapper added during `make`
        return np.array(self.state, dtype=np.float32), custom_reward, terminated, False, {}
       
    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ): 
        self.env.reset()
        super().reset(seed=seed)
        # Note that if you use custom reset bounds, it may lead to out-of-bound
        # state/observations.
        low, high = utils.maybe_parse_reset_bounds(
            options, -0.05, 0.05  # default low
        )  # default high
        self.state = self.np_random.uniform(low=low, high=high, size=(4,))
        self.steps_beyond_terminated = None

        if self.render_mode == "human":
            self.env.render()
        return np.array(self.state, dtype=np.float32), {}

    def render(self, mode = "human"):
        # self.env.reset()
        self.env.render()

    def close(self):
        self.env.close()

# Create vectorized version of the custom continuous action environment
def make_custom_env():
    return customCartPoleEnv()

# Create vectorized environment for Stable Baselines
vec_env = make_vec_env(make_custom_env, n_envs=1)
vec_env.render_mode='human'

# Parallel environments
#vec_env = make_vec_env("CartPole-v1", n_envs=1)

model = PPO("MlpPolicy", vec_env, verbose=1)
model.learn(total_timesteps=5000)
model.save("ppo_cartpole")


del model # remove to demonstrate saving and loading


model = PPO.load("ppo_cartpole")

obs = vec_env.reset()
while True:
    action, _states = model.predict(obs)
    print("action = ",action)
    obs,rewards,dones,info = vec_env.step(action)
    vec_env.render()

