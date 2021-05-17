#!/usr/bin/env python
# -*- coding: utf-8 -*-

from argparse import ArgumentParser
import os

import numpy as np
import rospy

from stable_baselines3 import TD3
from stable_baselines3.td3.policies import MlpPolicy
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_checker import check_env

from envs.refinement import GazeboEnv
from envs.tensorboard import TensorboardCallback
from envs.helpers import deg2rad
from envs.eval import EvalCallbackWithInfo


def main(args):
    rospy.init_node("agent", anonymous=True)

    log_path = os.path.join(args.output_dir, "logs", args.environment)
    ckpt_path = os.path.join(args.output_dir, "models", args.environment, args.log_name)
    model_path = os.path.join(ckpt_path, "final_model")
    best_model_path = os.path.join(ckpt_path, "best_model")

    rospy.loginfo("Log path \t%s", log_path)
    rospy.loginfo("Ckpt path \t%s", ckpt_path)
    rospy.loginfo("Model path \t%s", model_path)
    rospy.loginfo("Best model path \t%s", best_model_path)

    rospy.loginfo("Training with arguments")
    hparams = vars(args)

    # convert degrees to radians
    names = ["roll_error_min", "roll_error_max", "pitch_error_min", "pitch_error_max", "yaw_error_min", "yaw_error_max"]
    for name in names:
        hparams[name] = deg2rad(hparams[name])

    for key, value in hparams.items():
        rospy.loginfo(f"- {key:20}{value}")

    rospy.loginfo("Loading environment...")
    env = GazeboEnv(hparams, "TRAIN")
    if args.check_env:
        check_env(env)

    rospy.loginfo("Preparing model...")
    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
    model = TD3(MlpPolicy, env, action_noise=action_noise, verbose=1, tensorboard_log=log_path)

    if args.train:

        eval_env = GazeboEnv(hparams, "EVAL")

        callbacks = [
            CheckpointCallback(save_freq=args.chkpt_freq, save_path=ckpt_path, name_prefix="chkpt"),
            TensorboardCallback(hparams),
            EvalCallbackWithInfo(
                eval_env,
                best_model_save_path=best_model_path,
                eval_freq=args.eval_freq,
                n_eval_episodes=args.n_eval_episodes,
                eval_at_init=args.eval_at_init,
                deterministic=True,
            ),
        ]
        rospy.loginfo("Training model...")
        model.learn(total_timesteps=args.time_steps, tb_log_name=args.log_name, callback=callbacks, log_interval=args.log_interval)
        model.save(model_path)
        rospy.loginfo("Saved final model under: " + model_path)

    else:
        rospy.loginfo("Loading model from: " + model_path)
        model.load(model_path)
        rospy.loginfo("Evaluating model...")

        for episode in range(20):
            obs = env.reset()
            env.seed(args.seed)
            for t in range(1000):
                action, _state = model.predict(obs, deterministic=True)
                obs, reward, done, info = env.step(action)
                env.seed(args.seed)
                if done:
                    rospy.loginfo(f"Episode finished after {t+1} timesteps.")
                    break

    rospy.loginfo("Done! Have a nice day.")


if __name__ == "__main__":
    parser = ArgumentParser("Trains an RL algorithm for autonomous grasping.")

    parser.add_argument("--environment", type=str, default="refinement", help="Environment to load.")
    parser.add_argument("--train", type=int, default=1, help="Whether to train or evaluate the model.")
    parser.add_argument("--seed", type=int, default=0, help="Seed for random number generators.")
    parser.add_argument("--refine_steps", type=float, default=15, help="Time steps to refine grasp.")
    parser.add_argument("--max_refine_rate", type=float, default=3, help="Max rate of refinment control.")
    parser.add_argument("--lift_steps", type=float, default=6, help="Time steps to lift object.")
    parser.add_argument("--hold_steps", type=float, default=6, help="Time steps to hold object.")
    parser.add_argument("--secs_to_lift", type=float, default=2, help="For how many seconds to lift object.")
    parser.add_argument("--secs_to_hold", type=float, default=2, help="For how many seconds to hold object.")
    parser.add_argument("--lift_dist", type=float, default=0.15, help="How far to lift object.")
    parser.add_argument("--joint_lim", type=float, default=3, help="End episode if joint limit reached.")
    parser.add_argument("--exec_secs", type=float, default=0.1, help="How long to wait after executing each command.")
    parser.add_argument("--obj_shift_tol", type=float, default=0.03, help="How far object is allowed to shift.")
    parser.add_argument("--time_steps", type=float, default=2000, help="How many time steps to train.")
    parser.add_argument("--log_name", type=str, default="test", help="Name for log.")
    parser.add_argument("--output_dir", type=str, default="./", help="Path of output directory.")
    parser.add_argument("--chkpt_freq", type=int, default=300, help="Save model every n training steps.")
    parser.add_argument("--x_error_min", type=float, default=0, help="Positional error along x direction [m]")
    parser.add_argument("--y_error_min", type=float, default=0, help="Positional error along y direction [m]")
    parser.add_argument("--z_error_min", type=float, default=0, help="Positional error along z direction [m]")
    parser.add_argument("--x_error_max", type=float, default=0, help="Positional error along x direction [m]")
    parser.add_argument("--y_error_max", type=float, default=0, help="Positional error along y direction [m]")
    parser.add_argument("--z_error_max", type=float, default=0, help="Positional error along z direction [m]")
    parser.add_argument("--roll_error_min", type=float, default=0, help="Orientational error along x direction [deg]")
    parser.add_argument("--pitch_error_min", type=float, default=0, help="Orientational error along y direction [deg]")
    parser.add_argument("--yaw_error_min", type=float, default=0, help="Orientational error along z direction [deg]")
    parser.add_argument("--roll_error_max", type=float, default=0, help="Orientational error along x direction [deg]")
    parser.add_argument("--pitch_error_max", type=float, default=0, help="Orientational error along y direction [deg]")
    parser.add_argument("--yaw_error_max", type=float, default=0, help="Orientational error along z direction [deg]")
    parser.add_argument("--framework", type=int, default=1, help="Which reward framework to train with (1 or 2).")
    parser.add_argument("--log_interval", type=int, default=1, help="After how many episodes to log.")
    parser.add_argument("--eval_freq", type=int, default=200, help="After how many time steps to evaluate.")
    parser.add_argument("--n_eval_episodes", type=int, default=10, help="How many episodes to run when evaluating.")
    parser.add_argument("--eval_at_init", type=bool, default=False, help="Whether to evaluate environment at step = 0.")
    parser.add_argument("--check_env", type=bool, default=False, help="Whether to check environment or not.")
    parser.add_argument("--w_binary_rew", type=float, default=2, help="Weight for binary reward in framework 3")
    parser.add_argument("--w_eps_torque", type=float, default=10, help="Weight for epsilon torque in framework 1 and 3")
    parser.add_argument("--w_delta", type=float, default=0.1, help="Weight for delta in framework 1 and 3")
    
    args, unknown = parser.parse_known_args()

    main(args)
