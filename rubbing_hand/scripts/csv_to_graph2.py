#!/usr/bin/python
#coding: utf-8

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator

df_CAVS = pd.read_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/CAVS_error.csv")
df_CAVS = df_CAVS.astype(float)
df_FLAT = pd.read_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/FLAT_error.csv")


fig, axes = plt.subplots(nrows=1, ncols=1, figsize=(12, 6))

axes[0].scatter(x=df_FLAT["step"], y=df_FLAT["error"]/60, color="blue", label="FLAT")
# axes[1].scatter(x=df_FLAT["step"], y=df_FLAT["max angular velocity"], color="blue", label="FLAT")
# plt.legend("FLAT")

axes[0].scatter(x=df_CAVS["step"], y=df_CAVS["error"]/60, color="red", label="CAVS")
# axes[1].scatter(x=df_CAVS["step"], y=df_CAVS["max angular velocity"], color="red", label="CAVS")
# plt.legend("CAVS")

axes[0].set_ylabel("error")
# axes[1].set_ylabel("max angular velocity [deg/s]")
plt.xlabel("step", fontsize=18)

axes[0].set_ylim(-100, 12000)
# axes[1].set_ylim(-10, 20)

# axes[0].set_yscale("log")

# plt.xscale("log")
# plt.xlim(0.0001, 0.06)
# plt.ylim(0, 25000)
# plt.grid(axis='x', which='major',color='black',linestyle='-', linewidth=1.5)
# plt.grid(axis='x', which='minor',color='black',linestyle='dashed')
# plt.semilogx()
# plt.xticks([0.00312, 0.00562, 0.01, 0.0178, 0.0312, 0.0562])
# plt.hlines(10, 0, 0.11, linestyles="dashed")
axes[0].legend(bbox_to_anchor=(1, 0), loc='lower right', borderaxespad=1, fontsize=18)
# axes[1].legend(bbox_to_anchor=(1, 0), loc='lower right', borderaxespad=1, fontsize=18)

# plt.savefig("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/result2.png")
plt.show()