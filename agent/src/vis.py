import os
from argparse import ArgumentParser

import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


def get_experiment_data(log_path, date, framework, misc, try_id, scalar_name, window_size=5, verbose=False, log_interval=4):

    # load tensorboard logs
    experiment_path = os.path.join(log_path, f"{date}_F{framework}_{misc}_try{try_id}_1")
    event_acc = EventAccumulator(experiment_path)
    event_acc.Reload()
    world_times, time_step, vals = zip(*event_acc.Scalars(scalar_name))

    # smooth data with moving average filter
    vals_smooth = np.convolve(vals, np.ones(window_size), "valid") / window_size
    num_episodes = np.arange(log_interval, len(vals_smooth) * log_interval + log_interval, log_interval)

    # create pd dataframe
    data = {"num_episodes": num_episodes, scalar_name: vals_smooth}
    df = pd.DataFrame(data)
    df["date"] = date
    df["framework"] = framework
    df["try_id"] = try_id

    if verbose:
        print("Tags in this tensorboard log are: \n", str(event_acc.Tags()))
        print("Smoothed vals are: \n", vals_smooth)
        print(df)
    return df


def add_to_df(df, log_path, date, framework, misc, end_try_id, scalar_name):
    try_ids = list(range(0, end_try_id + 1))
    for try_id in try_ids:
        df = df.append(get_experiment_data(log_path, date, framework, misc, try_id, scalar_name))
    return df


if __name__ == "__main__":
    parser = ArgumentParser("Plots results of RL algorithm training.")
    parser.add_argument(
        "--log_path",
        type=str,
        default="/home/parallels/catkin_ws/src/grasp_refinement/agent/src/logs/refinement",
        help="Path to tensorboard log.",
    )
    args = parser.parse_args()

    df = pd.DataFrame()
    misc = "Lim3_X0.03_Z0.02"
    scalar_name = "drop_test/sustained_holding"
    df = add_to_df(df, args.log_path, "29Apr", 1, misc, 5, scalar_name)
    df = add_to_df(df, args.log_path, "30Apr", 1, misc, 8, scalar_name)
    df = add_to_df(df, args.log_path, "29Apr", 2, misc, 5, scalar_name)
    df = add_to_df(df, args.log_path, "30Apr", 2, misc, 8, scalar_name)
    df = add_to_df(df, args.log_path, "29Apr", 3, misc, 4, scalar_name)
    df = add_to_df(df, args.log_path, "30Apr", 3, misc, 8, scalar_name)

    print(df)

    sns.set(style="darkgrid", font_scale=1.5)
    ax = sns.lineplot(data=df, x="num_episodes", y=scalar_name, hue="framework")
    ax.set_title("Avg. of [F1: 13], [F2: 13], [F3: 12] runs")
    ax.set_xlabel("Number of Episodes")
    ax.set_ylabel(scalar_name)
    plt.tight_layout(pad=0.5)
    plt.ylim(0, 1.2)
    plt.show()