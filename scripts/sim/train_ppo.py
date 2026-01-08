from stable_baselines3 import PPO
from sim.quadson_env import QuadsonEnv, PlottingCallback
from stable_baselines3.common.env_util import make_vec_env

# env = QuadsonEnv()
# model = PPO("MlpPolicy", env, verbose=1)

# Multi-agent training, need to disable GUI
# env = make_vec_env(QuadsonEnv, n_envs=4)

env = QuadsonEnv()

model = PPO(
    'MlpPolicy', 
    env, 
    verbose=0,
    learning_rate=5e-5,  # Current rate seems low
    n_steps=2048, 
    batch_size=256,
    device='cpu',
    gamma=0.995,  # Add discount factor
    gae_lambda=0.95,  # Add GAE parameter
    ent_coef=0.01,  # Add entropy coefficient for exploration
    n_epochs=10,  # Number of epochs for each update
    clip_range=0.2  # PPO clip range
)

plotting_callback = PlottingCallback()
model.learn(total_timesteps=5000000, callback=plotting_callback)
model.save("quadson_ppo")
