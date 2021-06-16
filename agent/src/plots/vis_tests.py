import os
from argparse import ArgumentParser

import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd


def get_csv_data(args, verbose=True, framework_list=[1, 2, 3, 4]):
    df = pd.DataFrame()
    for framework in framework_list:
        for try_id in range(1, args.max_num_trials + 1):
            path = os.path.join(args.log_path, f"{args.prefix}_f{framework}_id{try_id}_algotd3.csv")
            try:
                data = pd.read_csv(path)
                print("Got data from path " + path)
                data["framework"] = framework
                data["try_id"] = try_id
                df = df.append(data)
            except Exception:
                pass
    if verbose:
        print(df)
    return df


def plot(df, num_items=3, hue="framework"):
    palette = sns.color_palette("tab10", num_items)
    sns.set(style="darkgrid", font_scale=1.5)

    ax = sns.barplot(data=df, x="wrist_l2_error", y="sustained_holding", palette=palette, hue=hue)
    ax.set_title("Test results on 20 objects")
    ax.set_xlabel("L2 Wrist Error")
    ax.set_ylabel("Holding success")

    plt.tight_layout(pad=0.5)
    plt.show()


if __name__ == "__main__":
    parser = ArgumentParser("Plots results of RL algorithm testing.")
    parser.add_argument(
        "--log_path",
        type=str,
        default="/home/parallels/cluster_logs",
        help="Path to tensorboard log.",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="12Jun_EvalAfterTrain_AvgDelta",
        help="Prefix comment of your experiment.",
    )
    parser.add_argument(
        "--max_num_trials",
        type=int,
        default=20,
        help="How many trials were in your experiment.",
    )

    args = parser.parse_args()
    framework_list = [1, 2, 3]

    df = get_csv_data(args, framework_list=framework_list)
    plot(df, len(framework_list))
