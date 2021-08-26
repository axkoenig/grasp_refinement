import os
from argparse import ArgumentParser

import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd


palette = sns.color_palette("tab10", 4)
sns.set(style="darkgrid", font_scale=1.5)
OUTPUT_DIR = "./output"


def get_framework_title(framework):
    if framework == "force_framework":
        return "Contact Sensing"
    elif framework == "reward_framework":
        return "Reward Function"


def get_framework_name(force_framework, framework):
    if force_framework:
        if framework == 1:
            return "Full"
        elif framework == 2:
            return "Normal"
        elif framework == 3:
            return "Binary"
        elif framework == 4:
            return "None"
    else:
        if framework == 1:
            return r"$\epsilon + \delta$"
        elif framework == 2:
            return r"\delta$"
        elif framework == 3:
            return r"$\epsilon$"
        elif framework == 4:
            return r"$\beta$"


def get_csv_data(args, verbose=True):
    df = pd.DataFrame()

    dirs = [os.path.join(args.log_path, dir) for dir in os.listdir(args.log_path) if dir.startswith(args.prefix) and dir.endswith("_wdelta0.5.csv")]

    for dir in dirs:
        data = pd.read_csv(dir)

        # check if valid num experiments
        num_exp_per_obj = 10
        num_objs = 3
        num_wrist_errors = 8
        num_finished = num_exp_per_obj * num_objs * num_wrist_errors
        num_rows = data.shape[0]
        if num_finished != num_rows:
            print(f"Num rows is {num_rows} but should be {num_finished}. Name: {dir}")

        data["try_id"] = dir.split("_id")[1].split("_")[0]
        data["force_framework"] = get_framework_name(True, int(dir.split("_s")[2].split("_")[0]))
        data["reward_framework"] = get_framework_name(False, int(dir.split("_f")[1].split("_")[0]))
        data["tb_log"] = dir
        wrist_error_names = ["A", "B", "C", "D", "E", "F", "G", "H"]
        for i in range(len(wrist_error_names)):
            data.loc[data["trans_l2_error"]==i/100, "trans_l2_error"] = wrist_error_names[i]
            
        df = df.append(data)

    if verbose:
        print(df)
    return df


def plot(args, df, object_types=["cylinder", "box", "sphere"]):
    hue_order = ["Full", "Normal", "Binary", "None"] if args.compare == "force_framework" else [r"$\epsilon + \delta$", r"\delta$", r"$\epsilon$", r"$\beta$"]
    fig = plt.figure(figsize=(12, 17))
    num_frameworks = len(df[args.compare].unique())
    num_tries_per_exp = len(np.unique(df["tb_log"].values)) / num_frameworks
    fig.suptitle(f"Test results of {int(num_tries_per_exp)} models per framework. {df.shape[0]} data points in total.")

    # one plot for each object, success over wrist error
    for i in range(len(object_types)):
        ax = fig.add_subplot(4, 3, i + 1)
        sns.barplot(data=df.loc[df["object_type"] == object_types[i]], palette=palette, x="trans_l2_error", y="sustained_holding", hue=args.compare,  hue_order=hue_order,ax=ax)
        ax.set_xlabel("L2 Wrist Error")
        ax.set_ylabel("Holding Success")
        ax.set_title(object_types[i].capitalize())
        ax.set_ylim((0, 1))
        ax.get_legend().remove()

    for i in range(len(object_types)):
        ax = fig.add_subplot(4, 3, i + 4)
        cur_df = df.loc[df["object_type"] == object_types[i]]
        for j in range(len(hue_order)):
            sns.regplot(data=cur_df[cur_df[args.compare] ==  hue_order[j]], x="mass", y="sustained_holding", ax=ax, marker="", label=hue_order[j])
        ax.set_xlabel("Object Mass")
        ax.set_ylabel("Holding Success")
        ax.set_title(object_types[i].capitalize())
        ax.set_ylim((0, 1))

    ax = fig.add_subplot(4, 3, 7)

    for j in range(len(hue_order)):
        sns.regplot(data=df[df[args.compare] == hue_order[j]], x="mass", y="sustained_holding", ax=ax, marker="", label=hue_order[j])
    ax.set_xlabel("Object Mass")
    ax.set_ylabel("Holding Success")
    ax.set_title("All Objects")

    ax.set_ylim((0, 1))

    ax = fig.add_subplot(4, 3, 8)
    sns.barplot(data=df, palette=palette, x="trans_l2_error", y="sustained_holding", hue=args.compare, hue_order=hue_order, ax=ax)
    ax.set_xlabel("L2 Wrist Error")
    ax.set_ylabel("Holding Success")
    ax.set_title("All Objects")
    ax.set_ylim((0, 1))
    lines, labels = ax.get_legend_handles_labels()
    ax.get_legend().remove()

    ax = fig.add_subplot(4, 3, 9)
    sns.barplot(data=df, palette=palette, x=args.compare, order=hue_order, y="sustained_holding", ax=ax)
    ax.set_xlabel(get_framework_title(args.compare))
    ax.set_ylabel("Holding Success")
    ax.set_title("All Objects")
    ax.set_ylim((0, 1))

    ax = fig.add_subplot(4, 3, 10)
    sns.barplot(data=df, palette=palette, x="object_type", hue=args.compare, hue_order=hue_order, y="sustained_holding", ax=ax)
    ax.set_xlabel(get_framework_title(args.compare))
    ax.set_ylabel("Holding Success")
    ax.set_title("All Objects")
    ax.set_ylim((0, 1))
    ax.get_legend().remove()

    fig.tight_layout(rect=[0, 0.06, 1, 0.95], pad=0.5)
    fig.legend(lines, labels, loc="lower center", ncol=4)
    fig.show()
    fig.savefig(f"{OUTPUT_DIR}/test_{args.prefix}_{args.compare}.png")
    fig.savefig(f"{OUTPUT_DIR}/test_{args.prefix}_{args.compare}.pdf")


if __name__ == "__main__":
    parser = ArgumentParser("Plots results of RL algorithm testing.")
    parser.add_argument(
        "--log_path",
        type=str,
        default="/home/parallels/cluster_logs_sd6",
        help="Path to tensorboard log.",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="test_23Aug_SmallTau25k",
        help="Prefix comment of your experiment.",
    )
    parser.add_argument(
        "--compare",
        type=str,
        default="force_framework",
    )

    args = parser.parse_args()
    df = get_csv_data(args)
    plot(args, df)
