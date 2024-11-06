import gymnasium as gym
from gymnasium.wrappers import TimeLimit
from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env
import torch

def make_env():
    env = gym.make('InvertedPendulum-v4', max_episode_steps=1500)
    return env 

#Create the vectorized environment with the custom time limit applied
vec_env = make_vec_env(make_env, n_envs=100)
   

model = SAC(
    "MlpPolicy",
    vec_env,
    use_sde=True,                      # Enable gSDE
    sde_sample_freq=8,                 # gSDE sample frequency as specified
    learning_rate=7.3e-4,              # Learning rate
    buffer_size=int(3e5),              # Replay buffer size
    learning_starts=10000,             # Warm-up steps
    batch_size=256,                    # Number of samples per minibatch    
    
    tau=0.02,                          # Target smoothing coefficient
    gamma=0.98,                        # Discount factor
    # train_freq=1,                      # Train every episode
    # gradient_steps=-1,                 # Train after each episode
    policy_kwargs=dict(
        net_arch=[400, 300],           # Number of hidden units per layer
        log_std_init=-3,               # Initial log σ for gSDE
        activation_fn=torch.nn.ReLU,   # Non-linearity
    ),
    ent_coef="auto",                   # Entropy coefficient (α) set to auto
    target_entropy="auto",             # Target entropy (-dim(A)), auto handles this based on action space
    verbose=1
)
model.set_parameters("/home/jang/unitree_mujoco/simulate/src/RL/policies/mujoco_cartPoleDOWN(1)")

# 1: ent_auto 0.1, step : 10^6(30m), 2: 10^7, 3: revise x position limit 0.4 10^7,  4: add x_dot_limit : 1, 5: (3)+ 3*10^7 with adding - 0.0001 x_dot
# 6: revise x_dot limit 0.8, x position limit 0.35, -0.001 x_dot, 7: expand init theta -1.5pi 1.5pi    8: add random cart position, revise pole_x_error 2*self.length+2*self.x_threshold 
# 9: add cart_vel_limit 1m/s  


# 20 : success!!!
# 21 : success just in RL env 7*pow(10,7)
# 22 : 3*pow(10,7)  env(100)
# 23 : 3*pow(10,7)  env(100) armature 0.01 continuity 0.01
# 24 : 3*pow(10,7)  env(100) armature 0.01 friction 0.01, fail
# 25 : 3*pow(10,7)  env(100) armature 0.01 friction 0.01, 

model.learn(total_timesteps=2*pow(10,6),progress_bar=1)
model.save("/home/jang/unitree_mujoco/simulate/src/RL/policies/mujoco_cartPoleDOWN(2)") #17:5, 18:8


del model # remove to demonstrate saving and loading
