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

from callbacks.tensorboard import TensorboardCallback
from callbacks.eval import EvalCallbackWithInfo
from controller.gazebo_env import GazeboEnv
from controller.helpers.transforms import deg2rad
from args import parse_args


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

    rospy.loginfo("Loading training environment...")
    env = GazeboEnv(hparams, "TRAIN")
    if args.check_env:
        check_env(env)

    rospy.loginfo("Preparing model...")

    if args.algorithm == "td3":
        n_actions = env.action_space.shape[-1]
        action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
        model = TD3(MlpPolicy, env, action_noise=action_noise, verbose=1, tensorboard_log=log_path)
    elif args.algorithm == "sac":
        model = SAC("MlpPolicy", env, verbose=1, tensorboard_log=log_path)
    elif args.algorithm == "ppo":
        model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_path)
    elif args.algorithm == "a2c":
        model = A2C("MlpPolicy", env, verbose=1, tensorboard_log=log_path)
    else:
        rospy.logerr("Unrecognized algorithm: " + args.algorithm)
        return

    if args.train:

        rospy.loginfo("Loading evaluation environment ...")
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
        rospy.loginfo("Loading model from: " + args.eval_model_path)
        model.load(args.eval_model_path)
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
    args = parse_args()
    main(args)
