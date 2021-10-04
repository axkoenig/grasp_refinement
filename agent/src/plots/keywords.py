import pandas
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# search words for Google Scholar were "robot grasp reinforcement learning"
df = pandas.DataFrame(
    {
        "Year": [2000, 2001, 2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009, 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019, 2020],
        "Number of Papers": [362, 403, 368, 404, 473, 546, 556, 753, 730, 857, 836, 1030, 1180, 1270, 1320, 1350, 1700, 1940, 2610, 3180, 3990],
    }
)

# create figure 
sns.set(rc={"figure.figsize": (5, 3)})
sns.set(style="darkgrid", font_scale=1.1)
fig = sns.lineplot(data=df, x="Year", y="Number of Papers")
plt.tight_layout(rect=[0, 0, 1, 1], pad=0.5)
plt.xticks(np.arange(2000, 2021, step=5))

# save figure 
Path("./output").mkdir(parents=True, exist_ok=True)
plt.savefig("output/keywords.pdf")
