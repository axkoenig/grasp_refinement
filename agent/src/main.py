from argparse import ArgumentParser
import os

import numpy as np
import gym

from stable_baselines3 import TD3
from stable_baselines3.td3.policies import MlpPolicy
from stable_baselines3.common.noise import NormalActionNoise


def main(args):

    log_path = os.path.join(args.output_dir, "logs", args.environment)
    model_path = os.path.join(args.output_dir, "models", args.environment, args.log_name)

    print("Loading environment...")
    if args.environment == "sanity_one_joint":
        from envs.sanity_one_joint import GazeboEnv
        env = GazeboEnv(args.exec_secs, args.max_ep_len, args.joint_lim)
    elif args.environment == "sanity_three_joints":
        from envs.sanity_three_joints import GazeboEnv
        env = GazeboEnv(args.exec_secs, args.max_ep_len, args.joint_lim)
    elif args.environment == "refinement_4_dof":
        from envs.refinement_4_dof import GazeboEnv
        env = GazeboEnv(args.exec_secs, args.max_ep_len, args.joint_lim, args.obj_shift_tol)
    else:
        raise ValueError("Invalid environment name.")

    # prepare model
    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
    model = TD3(MlpPolicy, env, action_noise=action_noise, verbose=1, tensorboard_log=log_path)

    if args.train:
        print("Training model...")
        model.learn(total_timesteps=args.time_steps, tb_log_name=args.log_name)
        model.save(model_path)
        print("Saved model under: " + model_path)

    else:
        print("Loading model from: " + model_path)
        model.load(model_path)
        print("Evaluating model...")

        for episode in range(20):
            obs = env.reset()
            for t in range(1000):
                action, _state = model.predict(obs, deterministic=True)
                obs, reward, done, info = env.step(action)
                if done:
                    print(f"Episode finished after {t+1} timesteps.")
                    break

    print("Done! Have a nice day.")


if __name__ == "__main__":
    parser = ArgumentParser("Trains an RL algorithm for autonomous grasping.")

    parser.add_argument("--environment", type=str, default="refinement_4_dof", help="Environment to load.")
    parser.add_argument("--train", type=int, default=1, help="Whether to train or evaluate the model.")
    parser.add_argument("--max_ep_len", type=float, default=8, help="Maximum episode length in secs sim time.")
    parser.add_argument("--joint_lim", type=float, default=2, help="End episode if joint limit reached.")
    parser.add_argument("--exec_secs", type=float, default=0.3, help="How long to execute same command on hand.")
    parser.add_argument("--obj_shift_tol", type=float, default=0.03, help="How far object is allowed to shift.")
    parser.add_argument("--time_steps", type=float, default=4000, help="How many time steps to train.")
    parser.add_argument("--log_name", type=str, default="test", help="Name for log.")
    parser.add_argument("--output_dir", type=str, default="./", help="Path of output directory.")

    args = parser.parse_args()

    main(args)
