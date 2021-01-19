from argparse import ArgumentParser

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rcParams
import matplotlib.font_manager

rcParams["font.family"] = "Arial"
rcParams["xtick.labelsize"] = 11
rcParams["ytick.labelsize"] = 11
rcParams["axes.labelsize"] = 12
rcParams["axes.titlesize"] = 12
rcParams["axes.grid"] = True

def plot(csv_name):
    headers = [
        "time_stamp",
        "object_name",
        # "object_mass",
        "final_state",
        "duration",
        "pos_error_x",
        "pos_error_y",
        "pos_error_z",
        "polar",
        "azimuthal",
        "offset",
        "status_msg"
    ]

    dataframe = pd.read_csv("data/" + csv_name + ".csv", names=headers)

    polar = dataframe["polar"]
    azimuthal = dataframe["azimuthal"]
    final_state = dataframe["final_state"]

    polar_range = np.unique(polar)
    azimuthal_range = np.unique(azimuthal)

    x, y = np.meshgrid(polar_range, azimuthal_range)

    rows = len(azimuthal_range)
    cols = len(polar_range)
    z = np.zeros((rows, cols))

    # iterate over z and fill in correct values
    for r in range(0, rows):
        for c in range(0, cols):

            # check what current x and y are
            cur_x = x[r, c]
            cur_y = y[r, c]

            # obtain row id that matches [cur_x, cur_y]
            cur_x_rows = dataframe.index[polar == cur_x]
            cur_y_rows = dataframe.index[azimuthal == cur_y]
            row_id = np.intersect1d(cur_x_rows, cur_y_rows)

            # if we don't have data in this field then declare experiment failed
            if len(row_id) == 0:
                print(
                    f"Oops, looks like there is no final_state for the combination {cur_x_rows}, {cur_y_rows}. Probably you stopped the experiment early."
                )
                z[r, c] = -2
            else:
                # access final state at this row_id and assign to z
                z[r, c] = final_state[row_id].values
    
    # assumes step size is uniform across one axis
    step_size_x = abs(x[0,0]-x[0,1])
    step_size_y = abs(y[0,0]-y[1,0])

    # transform x and y to boundaries of x and y
    x2,y2 = np.meshgrid(np.arange(x.min()-step_size_x/2,x.max()+step_size_x,step_size_x),np.arange(y.min()-step_size_y/2,y.max()+step_size_y,step_size_y))

    fig = plt.figure(constrained_layout=True, figsize=(8, 5))
    plot = plt.pcolormesh(x2,y2,z, cmap=plt.cm.Greens, vmin=-1,vmax=3)
    plt.xlabel("Polar angle (rad)")
    plt.ylabel("Azimuthal angle (rad)")
    plt.grid(True, color="grey", lw=0.5)
    cbar = fig.colorbar(plot, ticks=[-1, 0, 1, 2, 3])
    cbar.set_label("Final state")
    cbar.ax.set_yticklabels(
        [
            "-1, Simulation Failure",
            "0, Not Grasped",
            "1, Grasped But Not Lifted",
            "2, Grasped And Lifted",
            "3, Grasped And In Goal Pose",
        ]
    )
    plt.show()
    fig.savefig(f"./figures/{csv_name}.png", dpi=500)

if __name__ == "__main__":

    parser = ArgumentParser()
    parser.add_argument("--csv_name", type=str, default="test", help="Name of csv (without .csv)")
    args = parser.parse_args()
    plot(args.csv_name)
