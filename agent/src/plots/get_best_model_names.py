import os
from argparse import ArgumentParser

import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import pickle

# palette = sns.color_palette("tab10", 7) # we have 7 frameworks overall
# import pdb

# pdb.set_trace()
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
# import pdb

# pdb.set_trace()
plots = 2
palette = sns.color_palette(seven_tab10_colors[:4])

if plots == 2:
    first_col = [seven_tab10_colors[0]]
    seven_tab10_colors = first_col + seven_tab10_colors[-3:]
    palette = sns.color_palette(seven_tab10_colors)
sns.set(style="darkgrid", font_scale=1)
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
            return r"$\epsilon$ and $\delta$"
        elif framework == 2:
            return r"$\delta$"
        elif framework == 3:
            return r"$\epsilon$"
        elif framework == 4:
            return r"$\beta$"


def get_csv_data(args, prefixes, num_seeds, verbose=True):
    df = pd.DataFrame()
    framework_seeds = [0, 0, 0, 0]
    for prefix in prefixes:
        dirs = [os.path.join(args.log_path, dir) for dir in os.listdir(args.log_path) if dir.startswith(prefix) and dir.endswith("_wdelta0.5.csv")]

        for dir in dirs:

            data = pd.read_csv(dir)

            # convert bool to int
            data["sustained_holding"] = data["sustained_holding"].astype(np.int)
            data["sustained_lifting"] = data["sustained_lifting"].astype(np.int)

            # check if valid num experiments
            num_exp_per_obj = 10
            num_objs = 3
            num_wrist_errors = 8
            num_finished = num_exp_per_obj * num_objs * num_wrist_errors
            num_rows = data.shape[0]
            if num_finished != num_rows:
                print(f"Num rows is {num_rows} but should be {num_finished}. Name: {dir}. Skipping!")
                continue

            data["try_id"] = dir.split("_id")[1].split("_")[0]

            # TODO
            if int(dir.split("_id")[1].split("_")[0]) < 800:
                continue

            force_framework_number = int(dir.split("_s")[2].split("_")[0])
            reward_framework_number = int(dir.split("_f")[1].split("_")[0])
            force_framework = get_framework_name(True, force_framework_number)
            reward_framework = get_framework_name(False, reward_framework_number)

            if args.keep_fixed == "reward_framework" and reward_framework != args.keep_fixed_value:
                print(f"Skipping data because reward framework is {reward_framework} and we want {args.keep_fixed_value}.")
                continue
            elif args.keep_fixed == "force_framework" and force_framework != args.keep_fixed_value:
                print(f"Skipping data because force framework is {force_framework} and we want {args.keep_fixed_value}.")
                continue

            # looking at force frameworks
            if args.keep_fixed == "reward_framework":
                framework_seeds[force_framework_number - 1] += 1
                if framework_seeds[force_framework_number - 1] >= num_seeds:
                    continue
            else:
                framework_seeds[reward_framework_number - 1] += 1
                if framework_seeds[reward_framework_number - 1] >= num_seeds:
                    continue

            data["force_framework"] = force_framework
            data["reward_framework"] = reward_framework
            data["tb_log"] = dir
            data["object_type"] = data["object_type"].str.capitalize()
            data.loc[(data.object_type == "Box"), "object_type"] = "Cuboid"
            wrist_error_names = ["A", "B", "C", "D", "E", "F", "G", "H"]
            for i in range(len(wrist_error_names)):
                data.loc[data["trans_l2_error"] == i / 100, "trans_l2_error"] = wrist_error_names[i]

            ### ##############################
            df = df.append(pd.DataFrame({"sustained_holding_mean": data["sustained_holding"].mean(), "tb_log": dir}, index=[0]))
            #######

            # df = df.append(data)

    if verbose:
        print(df)
    return df


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
        default="test_23Aug_SmallTau25k_f",
        help="Prefix comment of your experiment.",
    )
    parser.add_argument(
        "--compare",
        type=str,
        default="force_framework",
    )
    parser.add_argument(
        "--keep_fixed",
        type=str,
        default="reward_framework",
    )
    parser.add_argument(
        "--keep_fixed_value",
        type=str,
        default=r"$\epsilon$ and $\delta$",
    )

    args = parser.parse_args()

    if plots == 1:
        args.compare = "reward_framework"
        args.keep_fixed = "force_framework"
        args.keep_fixed_value = "Full"
        frameworks = [r"$\epsilon$ and $\delta$", r"$\delta$", r"$\epsilon$", r"$\beta$"]
    elif plots == 2:
        args.compare = "force_framework"
        args.keep_fixed = "reward_framework"
        args.keep_fixed_value = r"$\epsilon$ and $\delta$"
        frameworks = ["Full", "Normal", "Binary", "None"]

    prefixes = [
        "test_23Aug_SmallTau25kRew26Aug_f",
        "test_28Aug_SmallTau25kMore30Aug_f",
        "test_28Aug_SmallTau25kMore02Sep_f",
        "test_28Aug_SmallTau25kMore03Sep_f",
        "test_28Aug_SmallTau25kMore04SepFestival_f",
        "test_28Aug_SmallTau25kMore06SepBerlin_f",
        "test_28Aug_SmallTau25kMore08SepBluecherNeu_f",
        "test_28Aug_SmallTau25kMore09SepBluecherThursday_f",
        "test_28Aug_SmallTau25kMore10SepBluecherFriday_f",
        "test_28Aug_SmallTau25kMore10SepBluecherSaturday_f",
    ]

    # select num_seeds first experiments
    num_seeds = 41

    df = get_csv_data(args, prefixes, num_seeds)

    print(df.sort_values(by=["sustained_holding_mean"]).to_string())

    import pdb

    pdb.set_trace()

    title = "Num seeds for "

    valid_tb_logs = []
    # print unique number of seeds for each compare framework
    for i in frameworks:
        num_seeds = df.loc[df[args.compare] == i, "try_id"].nunique()
        print(f"num seeds for {i} is", num_seeds)
        title = title + f"{i}: {num_seeds}, "

        unique_tb_logs = df.loc[df[args.compare] == i, "tb_log"].unique()
        for j in range(len(unique_tb_logs)):
            # strip each log
            valid_tb_logs.append(unique_tb_logs[j].split("test_")[1].split("_wdelta0.5")[0])
