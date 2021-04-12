#!/usr/bin/python
#coding: utf-8

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator

df = pd.read_csv('/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0221/CAVS/rosbag/CAVS34_2021-02-22-00-58-55.csv')
df = df.drop("Unnamed: 0", axis=1)
# print(df)
# ylabel = ["[deg]", "[deg/s]", "[mm]", "[mm/step]"]
# xlabel = ["time [s]"]
fig, axes = plt.subplots(nrows=4, ncols=1, figsize=(12, 6))
df.plot(x="time", subplots=True, ax=axes)


axes[0].legend(bbox_to_anchor=(1, 0.1), loc='lower right', borderaxespad=0, fontsize=12)
axes[0].hlines(60, 0, 30, linestyles='dashed')
axes[0].set_ylabel("[deg]")

axes[1].legend(bbox_to_anchor=(1, 0.9), loc='upper right', borderaxespad=0, fontsize=12)
axes[1].hlines(10, 0, 30, linestyles='dashed')
# axes[1].yaxis.set_ticks(np.arange(-10,31,10)) 
axes[1].set_ylabel("[deg/s]")

axes[2].legend(bbox_to_anchor=(1, 0.1), loc='lower right', borderaxespad=0, fontsize=12)
axes[2].yaxis.set_ticks(np.arange(10,21,5))
axes[2].set_ylabel("[mm]")

axes[3].legend(bbox_to_anchor=(1, 0.9), loc='upper right', borderaxespad=0, fontsize=12)
axes[3].set_ylabel("[mm/step]")

plt.xlabel('time [s]', fontsize=12)

# plt.savefig("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0221/CAVS/rosbag/CAVS35_2021-02-22-01-01-39.eps")
# plt.savefig("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0221/CAVS/rosbag/CAVS35_2021-02-22-01-01-39.png")
plt.show()