import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import numpy as np
from pathlib import Path

sns.set(style="whitegrid", font_scale=1.1)


def plot_force_space():
    fig = plt.figure(figsize=(7, 7), tight_layout=True, constrained_layout=False)
    ax = plt.axes(projection="3d", proj_type="ortho")

    # normals
    ax.quiver(0, 0, 0, -0.995027, -0.087835, 0.046968, color="r", label=r"Normal $n_1$", linestyle="dashed", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, -0.997995, -0.062678, 0.008758, color="g", label=r"Normal $n_2$", linestyle="dashed", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, 0.988656, -0.147230, 0.029692, color="b", label=r"Normal $n_3$", linestyle="dashed", arrow_length_ratio=0.2)

    # forces
    ax.quiver(0, 0, 0, -0.801991, 0.597336, 0.000028, color="r", label=r"Forces $f_{1,j}$", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, -0.769880, -0.103180, 0.629792, color="r", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, -0.681663, -0.728305, -0.070061, color="r", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, -0.713773, -0.027789, -0.699825, color="r", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, -0.784070, 0.619901, -0.030942, color="g", label=r"Forces $f_{2,j}$", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, -0.749792, -0.022614, 0.661287, color="g", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, -0.700777, -0.713156, 0.017912, color="g", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, -0.735055, -0.070640, -0.674318, color="g", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, 0.835810, 0.545561, -0.061522, color="b", label=r"Forces $f_{3,j}$", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, 0.714093, -0.146111, -0.684634, color="b", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, 0.642889, -0.765768, 0.017113, color="b", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, 0.764606, -0.074096, 0.640225, color="b", arrow_length_ratio=0.2)

    ax.legend(loc=(0.3, 0.78), ncol=2)

    ax.view_init(elev=20, azim=110)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.xaxis.set_rotate_label(False)
    ax.yaxis.set_rotate_label(False)
    ax.zaxis.set_rotate_label(False)
    ax.set_xlim(-1.1, 1.1)
    ax.set_ylim(-1.1, 1.1)
    ax.set_zlim(-1.1, 1.1)
    ax.xaxis.set_ticks(np.arange(-1, 1.2, 0.5))
    ax.yaxis.set_ticks(np.arange(-1, 1.2, 0.5))
    ax.zaxis.set_ticks(np.arange(-1, 1.2, 0.5))

    plt.show()

    # save figure
    Path("./output").mkdir(parents=True, exist_ok=True)
    fig.savefig("output/force_components.pdf")


def plot_torque_space():
    fig = plt.figure(figsize=(7, 7), tight_layout=True, constrained_layout=False)
    ax = plt.axes(projection="3d", proj_type="ortho")

    # normals
    scaling_fac = 0.07
    ax.quiver(0, 0, 0, -0.995027 * scaling_fac, -0.087835 * scaling_fac, 0.046968 * scaling_fac, color="r", label=r"Normal $n_1$", linestyle="dashed", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, -0.997995 * scaling_fac, -0.062678 * scaling_fac, 0.008758 * scaling_fac, color="g", label=r"Normal $n_2$", linestyle="dashed", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, 0.988656 * scaling_fac, -0.147230 * scaling_fac, 0.029692 * scaling_fac, color="b", label=r"Normal $n_3$", linestyle="dashed", arrow_length_ratio=0.2)

    # torques
    ax.quiver(0, 0, 0, -0.013071, -0.017551, 0.027694, color="r", label=r"Torques $\tau_{1,j}$", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, 0.005481, -0.041718, -0.000134, color="r", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, 0.015578, -0.012150, -0.025273, color="r", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, -0.002974, 0.012018, 0.002556, color="r", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, 0.018600, 0.024978, 0.029100, color="g", label=r"Torques $\tau_{2,j}$", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, 0.003056, -0.003603, 0.003342, color="g", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, -0.021498, 0.020511, -0.024411, color="g", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, -0.005954, 0.049092, 0.001348, color="g", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, 0.001916, -0.005878, -0.026093, color="b", label=r"Torques $\tau_{3,j}$", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, -0.004876, -0.029228, 0.001152, color="b", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, -0.003120, -0.002054, 0.025326, color="b", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, 0.003671, 0.021296, -0.001920, color="b", arrow_length_ratio=0.2)

    # lever arms
    ax.quiver(0, 0, 0, 0.039491, 0.005118, 0.021882, color="r", label=r"Lever $r_{1}$", linestyle="dotted", arrow_length_ratio=0.2)
    ax.quiver(0, 0, 0, 0.039788, 0.005657, -0.030287, color="g", label=r"Lever $r_{2}$", linestyle="dotted", arrow_length_ratio=0.2)
    ax.quiver(00, 0, 0, -0.038296, 0.006222, -0.004214, color="b", label=r"Lever $r_{3}$", linestyle="dotted", arrow_length_ratio=0.2)

    ax.legend(loc=(0.17, 0.78), ncol=3)

    ax.view_init(elev=20, azim=110)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.xaxis.set_rotate_label(False)
    ax.yaxis.set_rotate_label(False)
    ax.zaxis.set_rotate_label(False)
    ax.xaxis.set_ticks(np.arange(-0.04, 0.05, 0.04))
    ax.yaxis.set_ticks(np.arange(-0.04, 0.05, 0.04))
    ax.zaxis.set_ticks(np.arange(-0.04, 0.05, 0.04))
    plt.show()

    # save figure
    Path("./output").mkdir(parents=True, exist_ok=True)
    fig.savefig("output/torque_components.pdf")


plot_force_space()
plot_torque_space()
