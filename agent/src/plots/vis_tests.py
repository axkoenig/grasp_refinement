import os
from argparse import ArgumentParser

import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd


def get_csv_data(args, verbose=True, sensor_framework=True, framework_list=[1, 2, 3, 4]):
    df = pd.DataFrame()
    for framework in framework_list:
        for try_id in range(1, args.max_num_trials + 1):
            if sensor_framework:
                path = os.path.join(args.log_path, f"{args.prefix}_f1_s{framework}_id{try_id}_algotd3.csv")
            else:

                path = os.path.join(args.log_path, f"{args.prefix}_f{framework}_s1_id{try_id}_algotd3.csv")
            try:
                data = pd.read_csv(path)
                print("Got data from path " + path)
                data["framework"] = framework
                data["try_id"] = try_id
                data["trans_l2_error"] *= 100  # convert error to meters
                df = df.append(data)
            except Exception:
                pass
    if verbose:
        print(df)
    return df


def plot(df, num_items=3, hue="framework"):
    palette = sns.color_palette("tab10", num_items)
    sns.set(style="darkgrid", font_scale=1.5)

    ax = sns.barplot(data=df, x="trans_l2_error", y="sustained_holding", palette=palette, hue=hue)
    ax.set_title("Test results on 30 objects")
    ax.set_xlabel("L2 Wrist Error")
    ax.set_ylabel("Holding success")

    plt.tight_layout(pad=0.5)
    plt.show()


if __name__ == "__main__":
    parser = ArgumentParser("Plots results of RL algorithm testing.")
    parser.add_argument(
        "--log_path",
        type=str,
        default="/home/parallels/cluster_logs_final",
        help="Path to tensorboard log.",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="23Jun_SpheresAnd4thDOFSeedsWeight",
        help="Prefix comment of your experiment.",
    )
    parser.add_argument(
        "--max_num_trials",
        type=int,
        default=51,
        help="How many trials were in your experiment.",
    )

    args = parser.parse_args()

    # THIS IS FOR REWARD FRAMEWORKS
    framework_list = [1, 2, 3, 4]
    df = get_csv_data(args, framework_list=framework_list, sensor_framework=False)

    # THIS IS FOR FORCE FRAMEWORK
    # framework_list = [1]
    # df = get_csv_data(args, framework_list=framework_list, sensor_framework=False)
    # args.prefix ="23Jun_SpheresAnd4thDOFSeedsWeightForceFrameworks"
    # framework_list = [2, 3, 4]
    # df = pd.concat([df, get_csv_data(args, framework_list=framework_list, sensor_framework=True)])
    # framework_list = [1, 2, 3,4]

    df_sph = df.loc[df["object_type"] == "sphere"]
    print("sph ", df_sph.shape[0])
    df_cyl = df.loc[df["object_type"] == "cylinder"]
    print("cyl ", df_cyl.shape[0])
    df_box = df.loc[df["object_type"] == "box"]
    print("box ", df_box.shape[0])


    plot(df_sph, len(framework_list))

    plot(df_cyl, len(framework_list))

    plot(df_box, len(framework_list))
