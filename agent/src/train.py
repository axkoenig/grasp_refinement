import numpy as np
import gym

from stable_baselines3 import TD3
from stable_baselines3.td3.policies import MlpPolicy
from stable_baselines3.common.noise import NormalActionNoise

from envs.sanity_env_one_joint import GazeboEnv

env = GazeboEnv()
log_name = "one_joint"
parent_dir = "/output/"

n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

model = TD3(MlpPolicy, env, action_noise=action_noise, verbose=1, tensorboard_log=parent_dir + "logs")
model.learn(total_timesteps=10000, tb_log_name=log_name)
model.save(parent_dir + "models/" + log_name)

print("Evaluating model...")

for episode in range(20):
    obs = env.reset()
    for t in range(10000):
        action, _state = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        if done:
            print(f"Episode finished after {t+1} timesteps")
            break
