import os
from argparse import ArgumentParser

import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


def get_experiment_data(log_path, prefix, framework, try_id, scalar_name, window_size=4, verbose=False):

    # load tensorboard logs
    experiment_path = os.path.join(log_path, f"{prefix}_f{framework}_id{try_id}_1")
    event_acc = EventAccumulator(experiment_path)
    event_acc.Reload()
    world_times, time_steps, vals = zip(*event_acc.Scalars(scalar_name))

    # smooth data with moving average filter
    vals_smooth = np.convolve(vals, np.ones(window_size), "valid") / window_size
    time_steps_smooth = np.convolve(time_steps, np.ones(window_size), "valid") / window_size

    # linearly interpolate the missing time steps
    dense_time_steps = np.arange(int(time_steps_smooth[0]), int(time_steps_smooth[-1]) + 1)
    dense_vals = np.interp(dense_time_steps, time_steps_smooth, vals_smooth)

    # create pd dataframe
    data = {"dense_time_steps": dense_time_steps, scalar_name: dense_vals}
    df = pd.DataFrame(data)
    df["prefix"] = prefix
    df["framework"] = framework
    df["try_id"] = try_id

    if verbose:
        print("Tags in this tensorboard log are: \n", str(event_acc.Tags()))
        print("Smoothed vals are: \n", vals_smooth)
        print(df)
    return df


def add_to_df(df, log_path, prefix, framework, end_try_id, scalar_name):
    try_ids = list(range(1, end_try_id + 1))
    for try_id in try_ids:
        df = df.append(get_experiment_data(log_path, prefix, framework, try_id, scalar_name))
    return df


def get_all_data(args, verbose=True):
    # loads data from tensorboard logfiles
    df = pd.DataFrame()
    for framework in range(1, 4):
        df = add_to_df(df, args.log_path, args.prefix, framework, args.max_num_trials, args.scalar_name)
    if verbose: 
        print(df)
    return df


def plot(args, df, num_frameworks=3):
    palette = sns.color_palette("tab10", num_frameworks)
    sns.set(style="darkgrid", font_scale=1.5)
    
    ax = sns.lineplot(data=df, x="dense_time_steps", y=args.scalar_name, palette=palette, hue="framework")
    ax.set_title(f"{args.max_num_trials} Trainings Runs")
    ax.set_xlabel("Number of Time Steps")
    ax.set_ylabel(args.scalar_name)
    
    plt.tight_layout(pad=0.5)
    plt.ylim(0, 1.2)
    plt.show()


if __name__ == "__main__":
    parser = ArgumentParser("Plots results of RL algorithm training.")
    parser.add_argument(
        "--log_path",
        type=str,
        default="/home/parallels/cluster_logs",
        help="Path to tensorboard log.",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="05May_MultObjWRot",
        help="Prefix comment of your experiment.",
    )
    parser.add_argument(
        "--scalar_name",
        type=str,
        default="drop_test/sustained_holding",
        help="Which metric to plot.",
    )
    parser.add_argument(
        "--max_num_trials",
        type=int,
        default=5,
        help="How many trials were in your experiment.",
    )
    args = parser.parse_args()

    df = get_all_data(args)
    plot(args, df)