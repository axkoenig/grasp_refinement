import os
from argparse import ArgumentParser

import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


def get_experiment_data(
    tb_log,
    scalar_name,
    window_size=1,
    cap_len=10000,
    verbose=False,
    every_10_step=True,
    interpolate=False,
    smooth=False,
):

    event_acc = EventAccumulator(tb_log)
    event_acc.Reload()

    try:
        _, time_steps, vals = zip(*event_acc.Scalars(scalar_name))
    except KeyError:
        print("Could not find scalar " + scalar_name)
        return None

    # we want episode outcomes in this case
    if "sustained" in scalar_name:
        time_steps = np.arange(1, len(time_steps) + 1)
        window_size = 10
        smooth = True
        every_10_step = False

    # removes all data except multiples of 1k, for faster plotting
    if every_10_step:
        new_time_steps = []
        new_vals = []
        for i in range(0, len(time_steps)):
            if i % 10 == 0:
                new_time_steps.append(time_steps[i])
                new_vals.append(vals[i])
        time_steps = new_time_steps
        vals = new_vals

    # smooth data with moving average filter
    if smooth:
        vals = np.convolve(vals, np.ones(window_size), "valid") / window_size
        time_steps = np.convolve(time_steps, np.ones(window_size), "valid") / window_size

    # linearly interpolate the missing time steps
    if interpolate:
        dense_time_steps = np.arange(int(vals[0]), int(time_steps[-1]) + 1)
        dense_vals = np.interp(dense_time_steps, time_steps, vals)

        if len(dense_vals) > cap_len:
            dense_time_steps = dense_time_steps[:cap_len]
            dense_vals = dense_vals[:cap_len]
        time_steps = dense_time_steps
        vals = dense_vals

    # create pd dataframe
    data = {"steps": time_steps, scalar_name: vals}
    df = pd.DataFrame(data)

    # get some interesting hyperparameters
    # des_hparams = ["reward_framework", "force_framework", "w_eps_torque", "w_delta", "gradient_steps"]
    des_hparams = [ "w_eps_torque", "w_delta", "gradient_steps"]
    for hparam in des_hparams:
        _, _, x = zip(*event_acc.Scalars("hparams/" + hparam))
        df[hparam] = x[0]

    # # the variable that you want to keep fixed
    # _, _, x = zip(*event_acc.Scalars("hparams/reward_framework"))
    # keep_fixed = 1

    # # TODO delete 
    # if int(x[0]) != keep_fixed:
    #     print("none")
    #     return None

    if verbose:
        print("Tags in this tensorboard log are: \n", str(event_acc.Tags()))
        print("Smoothed vals are: \n", vals)
        print(df)
    return df


def get_all_data(args, verbose=True):
    # get any directories that match the starting string
    dirs = [os.path.join(args.log_path, dir) for dir in os.listdir(args.log_path) if dir.startswith(args.prefix)]

    df = pd.DataFrame()
    for tb_log in dirs:
        df = df.append(get_experiment_data(tb_log, args.scalar_name))
    if verbose:
        print(df)
    return df


def plot(args, df):

    num_items = len(pd.unique(df[args.compare]))
    palette = sns.color_palette("tab10", num_items)
    sns.set(style="darkgrid", font_scale=1.5)

    ax = sns.lineplot(data=df, x="steps", y=args.scalar_name, palette=palette, hue=args.compare)
    ax.set_title(f"10 Trainings Runs")
    ax.set_xlabel("Number of Episodes")
    ax.set_ylabel(args.scalar_name)

    plt.tight_layout(pad=0.5)
    plt.show()


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
        default="20Ju",
        help="Prefix comment of your experiment.",
    )
    parser.add_argument(
        "--scalar_name",
        type=str,
        default="train/critic_loss",
        help="Which metric to plot.",
    )
    parser.add_argument(
        "--compare",
        type=str,
        default="gradient_steps",
        help="Which metric to compare.",
    )

    args = parser.parse_args()
    df = get_all_data(args)
    plot(args, df)
