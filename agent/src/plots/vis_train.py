import os
from argparse import ArgumentParser

import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import pickle
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

from vis_thesis_tests import get_framework_name



seven_tab10_colors = [
    (0.12156862745098039, 0.4666666666666667, 0.7058823529411765),
    (1.0, 0.4980392156862745, 0.054901960784313725),
    (0.17254901960784313, 0.6274509803921569, 0.17254901960784313),
    (0.8392156862745098, 0.15294117647058825, 0.1568627450980392),
    (0.5803921568627451, 0.403921568627451, 0.7411764705882353),
    (0.5490196078431373, 0.33725490196078434, 0.29411764705882354),
    (0.8901960784313725, 0.4666666666666667, 0.7607843137254902),
    (0.4980392156862745, 0.4980392156862745, 0.4980392156862745),
    (0.7372549019607844, 0.7411764705882353, 0.13333333333333333),
    (0.09019607843137255, 0.7450980392156863, 0.8117647058823529),
]
plots = 2
palette = sns.color_palette(seven_tab10_colors[:4])

if plots == 2:
    first_col = [seven_tab10_colors[0]]
    seven_tab10_colors = first_col + seven_tab10_colors[-3:]
    palette = sns.color_palette(seven_tab10_colors)

sns.set(style="darkgrid", font_scale=1.1)
OUTPUT_DIR = "./output"


def get_scalars(
    tb_log,
    scalar_names,
    window_size=50,
):

    event_acc = EventAccumulator(tb_log)
    event_acc.Reload()

    try:
        _, time_steps, vals = zip(*event_acc.Scalars(scalar_names))
    except KeyError:
        print("Could not find scalar " + scalar_names)
        return None

    # convert time steps to episodes
    episodes = np.arange(1, len(time_steps) + 1)

    # smooth data with moving average filter
    vals = np.convolve(vals, np.ones(window_size), "valid") / window_size
    episodes = np.convolve(episodes, np.ones(window_size), "valid") / window_size

    # create pd dataframe
    data = {"episodes": episodes, scalar_names: vals}
    return pd.DataFrame(data)


def get_experiment_data(tb_log, args):

    event_acc = EventAccumulator(tb_log)
    event_acc.Reload()

    # the variable that you want to keep fixed
    # try:
    #     _, _, x = zip(*event_acc.Scalars(f"hparams/{args.keep_fixed}"))

    #     if x[0] != args.keep_fixed_value:
    #         print("none")
    #         return None

    # except Exception as e:
    #     print(e)
    #     return None

    df = pd.DataFrame()

    for s in args.scalar_names:
        df = df.append(get_scalars(tb_log, s[0]))

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
            df[hparam] = f"{x[0]:.5f}"

    # extract experiment id from log string
    id = tb_log.split("id")[1].split("_")[0]
    df["try_id"] = id
    df["tb_log"] = tb_log
    return df


def get_all_data(args, valid_tb_logs):
    print("Getting all data...")
    # get any directories
    all_dirs = [os.path.join(args.log_path, dir) for dir in os.listdir(args.log_path) if dir.endswith("_1")]

    # filter dirs for valid ones
    dirs = []
    for i in range(len(valid_tb_logs)):
        for j in range(len(all_dirs)):
            if all_dirs[j].__contains__(valid_tb_logs[i]):
                dirs.append(all_dirs[j])

    df = pd.DataFrame()

    for tb_log in dirs:
        print("Getting " + tb_log)
        df = df.append(get_experiment_data(tb_log, args))
        # break 

    # for each experiment (via unique id) get maximum episode number and from those select the min
    results = []
    experiments = np.unique(df["tb_log"].values)
    for exp in experiments:
        max_ep = df.loc[df["tb_log"] == exp, "episodes"].max()
        results.append((exp, max_ep))
    sorted_results = sorted(results, key=lambda t: t[1])
    print(sorted_results)
    min_ep_over_all_exp = sorted_results[0][1]
    print("cutting all experiments at " + str(min_ep_over_all_exp))
    # TODO
    df = df[df.episodes <= 900]

    # very infrequently we get data points with very high forces values, we treat them as outliers and remove them
    # TODO put back in
    # df_sum = df[df["step/sum_contact_forces"] <= 200]
    # df = df[np.isnan(df["step/sum_contact_forces"])]
    # df = df.append(df_sum)
    print(df)
    return df


def plot(args, df, title):
    hue_order = (
        ["Full", "Normal", "Binary", "None"] if args.compare == "force_framework" else [r"$\epsilon$ and $\delta$", r"$\delta$", r"$\epsilon$", r"$\beta$"]
    )
    num_frameworks = 4

    num_plots = len(args.scalar_names)
    fig = plt.figure(figsize=(10, 6))
    num_tries_per_exp = len(np.unique(df["tb_log"].values)) / num_frameworks
    print("Num tries per exp " + str(num_tries_per_exp))
    # if num_tries_per_exp % 1 != 0:
    #     print("Num tries per exp should be an integer. it is " + str(num_tries_per_exp))
    #     import pdb; pdb.set_trace()
    fig.suptitle(f"Training Results.")

    for i in range(num_plots):
        ax = fig.add_subplot(2, 3, i + 1)
        sns.lineplot(data=df, x="episodes", y=args.scalar_names[i][0], palette=palette, hue=args.compare, hue_order=hue_order, ax=ax)

        lines, labels = ax.get_legend_handles_labels()
        ax.get_legend().remove()
        ax.set_xlabel("Num. of Episodes")
        ax.set_xlim((0, 900))
        if "box" in args.scalar_names[i][0]:
            ax.set_xlabel("Num. of Box-Episodes")
            # todo remove
            ax.set_xlim((0, 300))
        elif "cylinder" in args.scalar_names[i][0]:
            ax.set_xlabel("Num. of Cylinder-Episodes")
            # todo remove
            ax.set_xlim((0, 300))
        elif "sphere" in args.scalar_names[i][0]:
            ax.set_xlabel("Num. of Sphere-Episodes")
            # todo remove
            ax.set_xlim((0, 300))
        ax.set_ylabel(args.scalar_names[i][1])
        if "sustained" in args.scalar_names[i][0]:
            ax.set_ylim((0, 1))

    fig.tight_layout(rect=[0, 0.06, 1, 0.95], pad=0.5)
    fig.legend(lines, labels, loc="lower center", ncol=4)
    fig.show()
    fig.savefig(f"{OUTPUT_DIR}/paper_train_{args.prefix}_{args.compare}.png")
    fig.savefig(f"{OUTPUT_DIR}/paper_train_{args.prefix}_{args.compare}.pdf")


if __name__ == "__main__":
    parser = ArgumentParser("Plots results of RL algorithm training.")
    parser.add_argument(
        "--log_path",
        type=str,
        default="/home/parallels/cluster_logs_sd6",
        help="Path to tensorboard log.",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="23Aug_SmallTau25k_f",
        help="Prefix comment of your experiment.",
    )
    parser.add_argument(
        "--scalar_names",
        type=str,
        default="step/sustained_holding",
        help="Which metric to plot.",
    )
    parser.add_argument(
        "--compare",
        type=str,
        default="reward_framework",
        help="Which metric to compare.",
    )
    args = parser.parse_args()

    args.scalar_names = [
        ("step/sustained_holding", "Holding Success"),
        ("step/sustained_lifting", "Lifting Success"),
        ("rollout/ep_rew_mean", "Mean Episode Reward"),
        ("step/sustained_holding_box", "Holding Success Box"),
        ("step/sustained_holding_cylinder", "Holding Success Cylinder"),
        ("step/sustained_holding_sphere", "Holding Success Sphere"),
    ]

    if plots == 1:
        args.compare = "reward_framework"
        frameworks = [r"$\epsilon$ and $\delta$", r"$\delta$", r"$\epsilon$", r"$\beta$"]
    elif plots == 2:
        args.compare = "force_framework"
        frameworks = ["Full", "Normal", "Binary", "None"]

    # we want to filter the frameworks that were successfully completeted
    done_frameworks_path = os.path.join(".", f"done_{args.compare}.pkl")
    with open(done_frameworks_path, "rb") as file:
        valid_tb_logs = pickle.load(file)

    title_path = os.path.join(".", f"title_{args.compare}.pkl")
    with open(title_path, "rb") as file:
        title = pickle.load(file)

    # get all data that we have a test result for
    df = get_all_data(args, valid_tb_logs)

    plot(args, df, title)
