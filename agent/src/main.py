#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

import numpy as np
import rospy

from stable_baselines3 import TD3, SAC, PPO, A2C
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.td3.policies import MlpPolicy
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.utils import set_random_seed

from callbacks.tensorboard import TensorboardCallback
from callbacks.eval import EvalCallbackWithInfo
from controller.controller import Controller
from controller.helpers.transforms import deg2rad
from controller.tests import test, generate_test_cases
from args import parse_args


def make_env(hparams, name):
    env = Controller(hparams, name)
    env.seed(hparams["seed"])
    set_random_seed(hparams["seed"])
    return env


def make_model_train(algorithm, env, log_path, policy_delay):
    if algorithm == "td3":
        n_actions = env.action_space.shape[-1]
        action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
        return TD3(MlpPolicy, env, action_noise=action_noise, verbose=1, tensorboard_log=log_path, policy_delay=policy_delay)
    elif algorithm == "sac":
        return SAC("MlpPolicy", env, verbose=1, tensorboard_log=log_path, train_freq=(2, "episode"))
    elif algorithm == "ppo":
        return PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_path)
    elif algorithm == "a2c":
        return A2C("MlpPolicy", env, verbose=1, tensorboard_log=log_path)
    else:
        rospy.logerr("Unrecognized algorithm: " + algorithm)
        return


def make_model_test(algorithm, test_model_path):
    if algorithm == "td3":
        return TD3.load(test_model_path)
    elif algorithm == "sac":
        return SAC.load(test_model_path)
    elif algorithm == "ppo":
        return PPO.load(test_model_path)
    elif algorithm == "a2c":
        return A2C.load(test_model_path)
    else:
        rospy.logerr("Unrecognized algorithm: " + algorithm)
        return


def main(args):
    rospy.init_node("agent")

    log_path = os.path.join(args.output_dir, "logs", args.environment)
    ckpt_path = os.path.join(args.output_dir, "models", args.environment, args.log_name)
    model_path = os.path.join(ckpt_path, "final_model")
    best_model_path = os.path.join(ckpt_path, "best_model")

    hparams = vars(args)
    hparams.update(
        {
            "log_path": log_path,
            "ckpt_path": ckpt_path,
            "model_path": model_path,
            "best_model_path": best_model_path,
        }
    )

    # convert degrees to radians
    names = ["roll_error_min", "roll_error_max", "pitch_error_min", "pitch_error_max", "yaw_error_min", "yaw_error_max"]
    for name in names:
        hparams[name] = deg2rad(hparams[name])

    rospy.loginfo("Running with arguments")
    for key, value in hparams.items():
        rospy.loginfo(f"- {key:20}{value}")

    if args.gen_new_test_cases:
        rospy.loginfo("Generating new test cases...")
        generate_test_cases()
    
    env_name = "TRAIN" if args.train else "TEST"
    rospy.loginfo(f"Making {env_name} environment...")
    env = make_env(hparams, env_name)

    if args.train:
        model = make_model_train(args.algorithm, env, log_path, args.policy_delay)

        if args.check_env:
            rospy.loginfo("Checking environment...")
            check_env(env)

        callbacks = [
            CheckpointCallback(save_freq=args.chkpt_freq, save_path=ckpt_path, name_prefix="chkpt"),
            TensorboardCallback(hparams),
        ]
        if args.eval_during_training:
            rospy.loginfo("Loading evaluation environment ...")
            eval_env = make_env(hparams, "EVAL")
            callbacks.append(
                EvalCallbackWithInfo(
                    eval_env,
                    best_model_save_path=best_model_path,
                    eval_freq=args.eval_freq,
                    n_eval_episodes=args.n_eval_episodes,
                    eval_at_init=args.eval_at_init,
                    deterministic=True,
                ),
            )

        rospy.loginfo("Training model...")
        model.learn(total_timesteps=args.time_steps, tb_log_name=args.log_name, callback=callbacks, log_interval=args.log_interval)
        model.save(model_path)
        rospy.loginfo("Saved final model under: " + model_path)

        if args.eval_after_training:
            rospy.loginfo("Testing final model")
            test(model, env, log_path, args.log_name)

    else:
        rospy.loginfo("Loading model from: " + args.test_model_path)
        model = make_model_test(args.algorithm, args.test_model_path)
        rospy.loginfo("Testing model...")
        test(model, env, log_path, "test_" + args.log_name)

    rospy.loginfo("Done! Have a nice day.")


if __name__ == "__main__":
    args = parse_args()
    main(args)
