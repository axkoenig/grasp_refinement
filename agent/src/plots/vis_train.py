import os
from argparse import ArgumentParser

import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

from vis_tests import get_framework_name

sns.set(style="darkgrid", font_scale=1.5)
OUTPUT_DIR = "./output"


def get_scalar(
    tb_log,
    scalar_name,
    window_size=10,
    cap_len=99999,
    every_10_step=False,
    smooth=False,
):

    event_acc = EventAccumulator(tb_log)
    event_acc.Reload()

    try:
        _, time_steps, vals = zip(*event_acc.Scalars(scalar_name))
    except KeyError:
        print("Could not find scalar " + scalar_name)
        return None

    # convert time steps to episodes
    time_steps = np.arange(1, len(time_steps) + 1)
    time_steps = time_steps[:cap_len]
    vals = vals[:cap_len]

    # removes all data except multiples of 10, for faster plotting
    if every_10_step:
        new_time_steps = []
        new_vals = []
        for i in range(0, len(time_steps)):
            if i % 10 == 0:
                new_time_steps.append(i)
                new_vals.append(vals[i])
        time_steps = new_time_steps
        vals = new_vals

    # smooth data with moving average filter
    if smooth or "sustained" in scalar_name:
        vals = np.convolve(vals, np.ones(window_size), "valid") / window_size
        time_steps = np.convolve(time_steps, np.ones(window_size), "valid") / window_size

    # create pd dataframe
    data = {"steps": time_steps, scalar_name: vals}
    return pd.DataFrame(data)


def get_experiment_data(tb_log, scalar_name, plot_objs_individually):

    event_acc = EventAccumulator(tb_log)
    event_acc.Reload()

    # the variable that you want to keep fixed
    try:
        # _, _, x = zip(*event_acc.Scalars("hparams/learning_rate"))
        # keep_fixed = 0.00019999999494757503

        # if x[0] != keep_fixed:
        #     print("none")
        #     return None

        # # the variable that you want to keep fixed
        _, _, x = zip(*event_acc.Scalars("hparams/batch_size"))
        keep_fixed = 256

        if x[0] != keep_fixed:
            print("none")
            return None
    except Exception as e:
        print(e)
        return None

    df = pd.DataFrame()
    if plot_objs_individually:
        objs = [scalar_name + "_box", scalar_name + "_cylinder", scalar_name + "_sphere"]

        for o in objs:
            df = df.append(get_scalar(tb_log, o))
    else:
        if len(args.scalar_name) == 1:
            df = get_scalar(tb_log, scalar_name)
        else:
            for s in args.scalar_name:
                df = df.append(get_scalar(tb_log, s))

    # get some interesting hyperparameters
    des_hparams = ["reward_framework", "force_framework", "w_eps_torque", "w_delta", "gradient_steps", "learning_rate", "batch_size", "tau"]

    for hparam in des_hparams:
        try:
            _, _, x = zip(*event_acc.Scalars("hparams/" + hparam))
        except KeyError as e:
            print(e)
            continue
        if "framework" in hparam:
            force_framework = True if hparam == "force_framework" else False
            df[hparam] = get_framework_name(force_framework, x[0])
        else:
            df[hparam] = x[0]
    return df


def get_all_data(args):
    # get any directories that match the starting string
    dirs = [os.path.join(args.log_path, dir) for dir in os.listdir(args.log_path) if dir.startswith(args.prefix)]

    df = pd.DataFrame()
    for tb_log in dirs:
        df = df.append(get_experiment_data(tb_log, args.scalar_name, args.plot_objs_individually))

    print(df)
    return df


def get_y_label(scalar):

    if "sustained" in scalar:
        return "Success Rate"
    elif "ep_rew" in scalar:
        return "Mean Episode Reward"
    elif "actor" in scalar:
        return "Actor Loss"
    elif "critic" in scalar:
        return "Critic Loss"
    else:
        return scalar


def plot(args, df):

    palette = sns.color_palette("tab10", len(df[args.compare].unique()))

    if args.plot_objs_individually:
        object_types = ["box", "cylinder", "sphere"]
        num_object_types = len(object_types)
        fig, ax = plt.subplots(1, num_object_types, figsize=(15, 5), sharey=True)

        for i in range(num_object_types):

            ax = sns.lineplot(data=df, x="steps", y=args.scalar_name + "_" + object_types[i], palette=palette, hue=args.compare, legend="full")
            ax[i].set_xlabel("Number of Episodes")
            ax[i].set_ylabel(None)
            if not i:
                ax[i].set_ylabel(get_y_label(args.scalar_name))
            ax[i].set_title(object_types[i].capitalize())
            if "sustained" in args.scalar_name:
                ax[i].set_ylim((0, 1))
    else:

        if len(args.scalar_name) == 1:
            fig = plt.figure()
            ax = sns.lineplot(data=df, x="steps", y=args.scalar_name, palette=palette, hue=args.compare, legend="full")
            ax.set_title(f"Resuts over 4 seeds")
            ax.set_xlabel("Number of Episodes")
            ax.set_ylabel(args.scalar_name)
        else:
            
            fig, ax = plt.subplots(
                1,
                len(args.scalar_name),
                figsize=(15, 5),
            )
            for i in range(len(args.scalar_name)):
                sns.lineplot(data=df, x="steps", y=args.scalar_name[i], palette=palette, hue=args.compare, legend="full", ax=ax[i])
                ax[i].set_xlabel("Number of Episodes")
                ax[i].set_ylabel(get_y_label(args.scalar_name[i]))
                if "sustained" in args.scalar_name[i]:
                    ax[i].set_ylim((0, 1))

    fig.tight_layout(pad=0.5)
    fig.show()
    args.scalar_name = "_".join(args.scalar_name)
    args.scalar_name = args.scalar_name.replace("/", "_")
    fig.savefig(f"{OUTPUT_DIR}/{args.compare}_{args.scalar_name}_{args.prefix}.png")


if __name__ == "__main__":
    parser = ArgumentParser("Plots results of RL algorithm training.")
    parser.add_argument(
        "--log_path",
        type=str,
        default="/home/parallels/cluster_logs_sd3",
        help="Path to tensorboard log.",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="21Jul_LearnRateB",
        help="Prefix comment of your experiment.",
    )
    parser.add_argument(
        "--scalar_name",
        type=str,
        default="rollout/ep_rew_mean",
        help="Which metric to plot.",
    )
    parser.add_argument(
        "--compare",
        type=str,
        default="learning_rate",
        help="Which metric to compare.",
    )
    parser.add_argument(
        "--plot_objs_individually",
        type=int,
        default=0,
        help="Whether to plot objects individually.",
    )

    args = parser.parse_args()

    args.scalar_name = ["train/actor_loss", "train/critic_loss", "rollout/ep_rew_mean"]
    df = get_all_data(args)
    plot(args, df)
