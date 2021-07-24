import os
from argparse import ArgumentParser

import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd


palette = sns.color_palette("tab10", 4)    
sns.set(style="darkgrid", font_scale=1.5)
OUTPUT_DIR = "./output"

def get_framework_name(force_framework, framework):
    if force_framework:
        if framework == 1:
            return "full"
        elif framework == 2:
            return "normal"
        elif framework == 3:
            return "binary"
        elif framework == 4:
            return "none"
    else:
        if framework == 1:
            return "epsilon + delta"
        elif framework == 2:
            return "delta"
        elif framework == 3:
            return "epsilon"
        elif framework == 4:
            return "binary"


def get_csv_data(args, verbose=True, force_framework=True, framework_list=[1, 2, 3, 4]):
    df = pd.DataFrame()
    for framework in framework_list:
        for try_id in range(1, args.max_num_trials + 1):
            if force_framework:
                path = os.path.join(args.log_path, f"{args.prefix}_f1_s{framework}_id{try_id}_algosac_lr0.0003_bs256.csv")
            else:

                path = os.path.join(args.log_path, f"{args.prefix}_f{framework}_s1_id{try_id}_algosac_lr0.0003_bs256.csv")
            try:
                data = pd.read_csv(path)
                print("Got data from path " + path)
                framework_type = "force_framework" if force_framework else "reward_framework"
                data[framework_type] = get_framework_name(force_framework, framework)
                data["try_id"] = try_id
                data["trans_l2_error"] *= 100  # convert error to cm
                data["trans_l2_error"] = data["trans_l2_error"].astype(int)
                df = df.append(data)
            except Exception as e:
                print(e)
    if verbose:
        print(df)
    return df


def plot(df, hue="framework", prefix="", object_types=["box", "cylinder", "sphere"]):
    num_object_types = len(object_types)
    fig, ax = plt.subplots(1, num_object_types, figsize=(15, 5), sharey=True)

    for i in range(num_object_types):
        sns.barplot(data=df.loc[df["object_type"] == object_types[i]], palette=palette, x="trans_l2_error", y="sustained_holding", hue=hue, ax=ax[i])
        ax[i].set_xlabel("L2 Wrist Error")
        ax[i].set_ylabel(None)
        if not i:
            ax[i].set_ylabel("Success Rate")
        ax[i].set_title(object_types[i].capitalize())
        ax[i].set_ylim((0, 1))

    # fig.set_title("One bar includes results of 10 different objects")
    fig.tight_layout(pad=0.5)
    fig.show()
    fig.savefig(f"{OUTPUT_DIR}/{hue}_object_{prefix}.png")

def plot_all_objects(df, hue="framework", prefix=""):
    fig = plt.figure()
    ax = sns.barplot(data=df, x="trans_l2_error", y="sustained_holding", palette=palette, hue=hue)
    ax.set_title("All Objects")
    ax.set_xlabel("L2 Wrist Error")
    ax.set_ylabel("Success Rate")
    ax.set_ylim((0, 1))

    # fig.set_title("One bar includes results of 10 different objects")
    fig.tight_layout(pad=0.5)
    fig.show()
    fig.savefig(f"{OUTPUT_DIR}/{hue}_objects_{prefix}.png")

def plot_all_objects_all_errors(df, hue="framework", prefix=""):
    fig = plt.figure()
    ax = sns.barplot(data=df, x=hue, y="sustained_holding", palette=palette)
    ax.set_title("All Objects, All Wrist Errors")
    ax.set_xlabel(hue)
    ax.set_ylabel("Success Rate")
    ax.set_ylim((0, 1))

    fig.tight_layout(pad=0.5)
    fig.show()
    fig.savefig(f"{OUTPUT_DIR}/{hue}_objects_errors_{prefix}.png")


if __name__ == "__main__":
    parser = ArgumentParser("Plots results of RL algorithm testing.")
    parser.add_argument(
        "--log_path",
        type=str,
        default="/home/parallels/cluster_logs_sd3",
        help="Path to tensorboard log.",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="20Jul_FinalAndLearnRate",
        help="Prefix comment of your experiment.",
    )
    parser.add_argument(
        "--max_num_trials",
        type=int,
        default=11,
        help="How many trials were in your experiment.",
    )

    args = parser.parse_args()
    force_framework = False
    framework_list = [1, 2, 3, 4]
    df = get_csv_data(args, framework_list=framework_list, force_framework=force_framework)

    framework_name = "force_framework" if force_framework else "reward_framework"
    plot(df, framework_name, prefix=args.prefix)
    plot_all_objects(df, framework_name, prefix=args.prefix)
    plot_all_objects_all_errors(df, framework_name, prefix=args.prefix)