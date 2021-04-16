from argparse import ArgumentParser
import os

import numpy as np
import gym

from stable_baselines3 import TD3
from stable_baselines3.td3.policies import MlpPolicy
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.env_checker import check_env


def main(args):

    log_path = os.path.join(args.output_dir, "logs", args.environment)
    ckpt_path =  os.path.join(args.output_dir, "models", args.environment, args.log_name)
    model_path = os.path.join(ckpt_path, "final_model")

    print("Loading environment...")
    if args.environment == "refinement":
        from envs.refinement import GazeboEnv, TensorboardCallback
        env = GazeboEnv(args.exec_secs, args.max_ep_len, args.joint_lim, args.obj_shift_tol, args.reward_weight, [args.x_error, args.y_error, args.z_error])
    else:
        raise ValueError("Invalid environment name.")

    # set random seeds
    # env.seed(args.seed)
    # env.action_space.seed(args.seed)
    # set_random_seed(args.seed)

    # import torch
    # torch.manual_seed(0)
    # import random
    # random.seed(0)
    # np.random.seed(0)

    # prepare model
    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
    # model = TD3(MlpPolicy, env, action_noise=action_noise, verbose=1, tensorboard_log=log_path, seed=args.seed)
    model = TD3(MlpPolicy, env, action_noise=action_noise, verbose=1, tensorboard_log=log_path)
    
    if args.train:

        checkpoint_callback = CheckpointCallback(save_freq=args.chkpt_freq, save_path=ckpt_path, name_prefix="chkpt")
        rewards_callback = TensorboardCallback()

        print("Training model...")
        model.learn(total_timesteps=args.time_steps, tb_log_name=args.log_name, callback=[checkpoint_callback, rewards_callback])
        model.save(model_path)
        print("Saved model under: " + model_path)

    else:
        print("Loading model from: " + model_path)
        model.load(model_path)
        print("Evaluating model...")

        for episode in range(20):
            obs = env.reset()
            env.seed(args.seed)
            for t in range(1000):
                action, _state = model.predict(obs, deterministic=True)
                obs, reward, done, info = env.step(action)
                env.seed(args.seed)
                if done:
                    print(f"Episode finished after {t+1} timesteps.")
                    break

    print("Done! Have a nice day.")


if __name__ == "__main__":
    parser = ArgumentParser("Trains an RL algorithm for autonomous grasping.")

    parser.add_argument("--environment", type=str, default="refinement", help="Environment to load.")
    parser.add_argument("--train", type=int, default=1, help="Whether to train or evaluate the model.")
    parser.add_argument("--seed", type=int, default=0, help="Seed for random number generators.")
    parser.add_argument("--max_ep_len", type=float, default=15, help="Maximum time steps in one episode.")
    parser.add_argument("--joint_lim", type=float, default=2, help="End episode if joint limit reached.")
    parser.add_argument("--exec_secs", type=float, default=0.3, help="How long to execute same command on hand.")
    parser.add_argument("--obj_shift_tol", type=float, default=0.03, help="How far object is allowed to shift.")
    parser.add_argument("--time_steps", type=float, default=2000, help="How many time steps to train.")
    parser.add_argument("--reward_weight", type=float, default=1, help="Weight in reward function.")
    parser.add_argument("--log_name", type=str, default="test", help="Name for log.")
    parser.add_argument("--output_dir", type=str, default="./", help="Path of output directory.")
    parser.add_argument("--chkpt_freq", type=int, default=300, help="Save model every n training steps.")
    parser.add_argument("--x_error", type=float, default=0, help="Positional error along x direction.")
    parser.add_argument("--y_error", type=float, default=0, help="Positional error along y direction.")
    parser.add_argument("--z_error", type=float, default=0, help="Positional error along z direction.")

    args = parser.parse_args()

    main(args)
