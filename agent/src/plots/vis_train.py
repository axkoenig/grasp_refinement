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


def get_scalars(
    tb_log,
    scalar_names,
    window_size=25,
    smooth=False,
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
    if smooth or "sustained" in scalar_names:
        vals = np.convolve(vals, np.ones(window_size), "valid") / window_size
        episodes = np.convolve(episodes, np.ones(window_size), "valid") / window_size

    # create pd dataframe
    data = {"episodes": episodes, scalar_names: vals}
    return pd.DataFrame(data)


def get_experiment_data(tb_log, scalar_names):

    event_acc = EventAccumulator(tb_log)
    event_acc.Reload()

    # the variable that you want to keep fixed
    # try:
    #     _, _, x = zip(*event_acc.Scalars("hparams/reward_framework"))
    #     keep_fixed = 1

    #     if x[0] != keep_fixed:
    #         print("none")
    #         return None

    # except Exception as e:
    #     print(e)
    #     return None

    df = pd.DataFrame()

    for s in scalar_names:
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


def get_all_data(args):
    print("Getting all data...")
    # get any directories that match the starting string
    dirs = [os.path.join(args.log_path, dir) for dir in os.listdir(args.log_path) if dir.startswith(args.prefix)]

    df = pd.DataFrame()

    for tb_log in dirs:
        print("Getting " + tb_log)
        df = df.append(get_experiment_data(tb_log, args.scalar_names))

    # for each experiment (via unique id) get maximum episode number and from those select the min
    # results = []
    # experiments = np.unique(df["tb_log"].values)
    # for exp in experiments:
    #     max_ep = df.loc[df["tb_log"] == exp, "episodes"].max()
    #     results.append((exp, max_ep))
    # sorted_results = sorted(results, key=lambda t: t[1])
    # print(sorted_results)
    # min_ep_over_all_exp = sorted_results[0][1]
    # print("cutting all experiments at " + str(min_ep_over_all_exp))
    # df = df[df.episodes <= min_ep_over_all_exp]

    print(df)
    return df


def plot(args, df):
    num_frameworks = len(df[args.compare].unique())
    palette = sns.color_palette("tab10", num_frameworks)

    num_plots = len(args.scalar_names)
    fig = plt.figure(figsize=(12, 17))
    num_tries_per_exp = len(np.unique(df["tb_log"].values)) / num_frameworks
    print("Num tries per exp " + str(num_tries_per_exp))
    # if num_tries_per_exp % 1 != 0:
    #     print("Num tries per exp should be an integer. it is " + str(num_tries_per_exp))
    #     import pdb; pdb.set_trace()
    fig.suptitle(f"Results averaged over {int(num_tries_per_exp)} seeds.")

    for i in range(num_plots):
        ax = fig.add_subplot(4, 3, i + 1)
        sns.lineplot(data=df, x="episodes", y=args.scalar_names[i][0], palette=palette, hue=args.compare, ax=ax)

        lines, labels = ax.get_legend_handles_labels()
        ax.get_legend().remove()
        ax.set_xlabel("Num. of Episodes")
        if "box" in args.scalar_names[i][0]:
            ax.set_xlabel("Num. of Box-Episodes")
        elif "cylinder" in args.scalar_names[i][0]:
            ax.set_xlabel("Num. of Cylinder-Episodes")
        elif "sphere" in args.scalar_names[i][0]:
            ax.set_xlabel("Num. of Sphere-Episodes")
        ax.set_ylabel(args.scalar_names[i][1])
        if "sustained" in args.scalar_names[i][0]:
            ax.set_ylim((0, 1))

    fig.tight_layout(rect=[0, 0, 1, 0.95], pad=0.5)
    fig.legend(lines, labels, loc="upper right", ncol=2)
    fig.show()
    fig.savefig(f"{OUTPUT_DIR}/final_{args.prefix}_{args.compare}.png")
    fig.savefig(f"{OUTPUT_DIR}/final_{args.prefix}_{args.compare}.pdf")


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
        default="16Aug_Final",
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
        default="force_framework",
        help="Which metric to compare.",
    )
    args = parser.parse_args()

    args.scalar_names = [
        ("step/sustained_holding", "Holding Success"),
        ("step/sustained_lifting", "Lifting Success"),
        ("rollout/ep_rew_mean", "Mean Episonde Reward"),
        ("step/sustained_holding_box", "Holding Success Box"),
        ("step/sustained_holding_cylinder", "Holding Success Cylinder"),
        ("step/sustained_holding_sphere", "Holding Success Sphere"),
        ("step/a_delta_cur", r"$\delta_{cur}$"),
        ("step/a_delta_task", r"$\delta_{task}$"),
        ("step/sum_contact_forces", "Sum of Contact Force Magn."),
        ("step/a_epsilon_force", r"$\epsilon_{f}$"),
        ("step/a_epsilon_torque", r"$\epsilon_{\tau}$"),
        ("step/num_contacts", "Num. of Contacts"),
    ]
    df = get_all_data(args)
    plot(args, df)
