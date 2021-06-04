import os
from argparse import ArgumentParser

import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
from tensorboard.backend.event_processing.directory_watcher import DirectoryDeletedError


def get_experiment_data(log_path, prefix, framework, try_id, scalar_name, new_name=False, log_id=1, window_size=10, cap_len=10000, verbose=False):

    # load tensorboard logs
    if not new_name:
        experiment_path = os.path.join(log_path, f"{prefix}_f{framework}_id{try_id}_{log_id}")
    else:
        experiment_path = os.path.join(log_path, f"{prefix}_f{framework}_id{try_id}_algotd3_{log_id}")

    try:
        event_acc = EventAccumulator(experiment_path)
        event_acc.Reload()
        try:
            world_times, time_steps, vals = zip(*event_acc.Scalars(scalar_name))
        except KeyError:
            print("Could not find scalar " + scalar_name)
            return None
    except DirectoryDeletedError:
        print(experiment_path + " does not exist. Returning None.")
        return None

    # smooth data with moving average filter
    vals_smooth = np.convolve(vals, np.ones(window_size), "valid") / window_size
    time_steps_smooth = np.convolve(time_steps, np.ones(window_size), "valid") / window_size

    # linearly interpolate the missing time steps
    dense_time_steps = np.arange(int(time_steps_smooth[0]), int(time_steps_smooth[-1]) + 1)
    dense_vals = np.interp(dense_time_steps, time_steps_smooth, vals_smooth)

    if len(dense_vals) > cap_len:
        dense_time_steps = dense_time_steps[:cap_len]
        dense_vals = dense_vals[:cap_len]

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


def add_to_df(df, log_path, prefix, framework, end_try_id, scalar_name, new_name=False, end_log_id=5):
    try_ids = list(range(1, end_try_id + 1))
    log_ids = list(range(1, end_log_id + 1))
    for try_id in try_ids:
        for log_id in log_ids:
            df = df.append(get_experiment_data(log_path, prefix, framework, try_id, scalar_name, new_name, log_id))
    return df


def get_all_data(args, new_name=False, verbose=True, framework_list=[1, 2, 4, 5, 6]):
    # loads data from tensorboard logfiles
    df = pd.DataFrame()
    for framework in framework_list:
        df = add_to_df(df, args.log_path, args.prefix, framework, args.max_num_trials, args.scalar_name, new_name)
    if verbose:
        print(df)
    return df


def plot(args, df, num_items=5, hue="framework"):
    palette = sns.color_palette("tab10", num_items)
    sns.set(style="darkgrid", font_scale=1.5)

    ax = sns.lineplot(data=df, x="dense_time_steps", y=args.scalar_name, palette=palette, hue=hue)
    ax.set_title(f'{df["try_id"].max()} Trainings Runs')
    ax.set_xlabel("Number of Time Steps")
    ax.set_ylabel(args.scalar_name)

    plt.tight_layout(pad=0.5)
    plt.ylim(0, 1.2)
    plt.xlim(0, 1950)
    plt.show()


def plot_percentiles(args, df, framework=1, num_groups=2):
    # filter out desired framework
    df = df[df["framework"] == framework]
    scores = np.empty(0)

    min_try_id = df["try_id"].min()
    max_try_id = df["try_id"].max()

    for i in range(min_try_id, max_try_id + 1):
        df_try = df[df["try_id"] == i]
        # score is average performance over time steps
        score = df_try[args.scalar_name].sum() / df_try.shape[0]
        scores = np.append(scores, score)

    # get decision boundaries
    boundaries = np.linspace(scores.min(), scores.max(), num_groups)

    # assemble new data frame with a new column that represents in which bin try_id is
    new_df = pd.DataFrame()
    for i in range(min_try_id, max_try_id + 1):
        df_try = df[df["try_id"] == i]
        bin = np.digitize(scores[i - 1], boundaries)
        df_try["bin"] = bin
        new_df = pd.concat([new_df, df_try])

    print("new dataframe is")
    print(new_df)
    plot(args, new_df, num_groups, "bin")
    return new_df


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
        default="26May_NoBernoulliAndDenserTactileObsFixed",
        help="Prefix comment of your experiment.",
    )
    parser.add_argument(
        "--scalar_name",
        type=str,
        default="step/sustained_holding",
        help="Which metric to plot.",
    )
    parser.add_argument(
        "--max_num_trials",
        type=int,
        default=14,
        help="How many trials were in your experiment.",
    )
    args = parser.parse_args()

    df = get_all_data(args, new_name=True)

    # args.prefix="19May_NewActionSpaceDelta0.05"
    # df = pd.concat([df, get_all_data(args, new_name=True)])
    plot(args, df)

    # plot_percentiles(args, df)
