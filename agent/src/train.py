import rospy
import gym
from stable_baselines3 import A2C
from stable_baselines3.common.env_checker import check_env

from env import GazeboEnv

env = GazeboEnv()
#check_env(env)

model = A2C("MlpPolicy", env, verbose=1, tensorboard_log="./logs/")
model.learn(total_timesteps=30000)

model.save("./a2c_model")
del model 
model = A2C.load("./a2c_model")

# I think this is inference
# for episode in range(20):
#     obs = env.reset()
#     for t in range(10000):
#         action, _state = model.predict(obs, deterministic=True)
#         obs, reward, done, info = env.step(action)
#         if done:
#             model.save("./a2c_model")
#             print(f"Episode finished after {t+1} timesteps")
#             break
