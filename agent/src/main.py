#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
    ckpt_path = os.path.join(args.output_dir, "models", args.environment, args.log_name)
    model_path = os.path.join(ckpt_path, "final_model")

    print("Log path \t", log_path)
    print("Ckpt path \t", ckpt_path)
    print("Model path \t", model_path)

    print("Loading environment...")
    if args.environment == "refinement":
        from envs.refinement import GazeboEnv
        from envs.callbacks import TensorboardCallback

        env = GazeboEnv(
            args.exec_secs,
            args.max_ep_len,
            args.joint_lim,
            args.obj_shift_tol,
            args.reward_weight,
            [
                [args.x_error_min, args.x_error_max],
                [args.y_error_min, args.y_error_max],
                [args.z_error_min, args.z_error_max],
            ],
            args.framework,
        )
    else:
        raise ValueError("Invalid environment name.")

    # prepare model
    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
    model = TD3(MlpPolicy, env, action_noise=action_noise, verbose=1, tensorboard_log=log_path)

    if args.train:
        callbacks = [CheckpointCallback(save_freq=args.chkpt_freq, save_path=ckpt_path, name_prefix="chkpt"), TensorboardCallback()]
        print("Training model...")
        model.learn(total_timesteps=args.time_steps, tb_log_name=args.log_name, callback=callbacks)
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
    parser.add_argument("--x_error_min", type=float, default=0, help="Positional error along x direction")
    parser.add_argument("--y_error_min", type=float, default=0, help="Positional error along y direction")
    parser.add_argument("--z_error_min", type=float, default=0, help="Positional error along z direction")
    parser.add_argument("--x_error_max", type=float, default=0, help="Positional error along x direction")
    parser.add_argument("--y_error_max", type=float, default=0, help="Positional error along y direction")
    parser.add_argument("--z_error_max", type=float, default=0, help="Positional error along z direction")
    parser.add_argument("--framework", type=int, default=1, help="Which reward framework to train with (1 or 2).")
    
    args, unknown = parser.parse_known_args()
    
    print("==================")
    print("received arguments")
    print("==================")
    print("--environment \t", args.environment)
    print("--train \t", args.train)
    print("--seed \t\t", args.seed)
    print("--max_ep_len \t", args.max_ep_len)
    print("--joint_lim \t", args.joint_lim)
    print("--exec_secs \t", args.exec_secs)
    print("--obj_shift_tol ", args.obj_shift_tol)
    print("--time_steps \t", args.time_steps)
    print("--reward_weight ", args.reward_weight)
    print("--log_name \t", args.log_name)
    print("--output_dir \t", args.output_dir)
    print("--chkpt_freq \t", args.chkpt_freq)
    print("--x_error_min \t", args.x_error_min)
    print("--y_error_min \t", args.y_error_min)
    print("--z_error_min \t", args.z_error_min)
    print("--x_error_max \t", args.x_error_max)
    print("--y_error_max \t", args.y_error_max)
    print("--z_error_max \t", args.z_error_max)
    print("--framework \t", args.framework)
    print("==================")

    main(args)
