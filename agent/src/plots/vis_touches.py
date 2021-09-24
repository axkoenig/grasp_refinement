import pandas as pd
import os

dir = "/home/parallels/cluster_logs_sd6"
exp = "28Aug_SmallTau25kMore10SepBluecherFriday_f2_s1_id1104_algosac_lr0.0001_tau0.0001_wdelta0.5_TEST_io.csv"

dir = os.path.join(dir, exp)


df = pd.read_csv(dir)

# remove unnecessary lines
df = df.drop(df[df["object_type"] == "object_type"].index)


parts = [1, 2, 3, 4]

df = df.astype(float, errors="ignore")
import pdb

pdb.set_trace()

x_vals = ["contact_force_p0_x"]
y_vals = [df[x_vals[0] > 0].count()]


for p in [2, 3, 4]:
    for x in ["_prox", "_dist"]:
        quantity = "contact_force_p" + p + x + "_x"


import pdb

pdb.set_trace()
