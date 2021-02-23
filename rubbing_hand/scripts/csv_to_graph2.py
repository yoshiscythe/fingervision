#!/usr/bin/python
#coding: utf-8

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator

df_CAVS = pd.read_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/CAVS_error.csv")
df_FLAT = pd.read_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/FLAT_error.csv")

# plt.scatter(x=df_FLAT["step"], y=df_FLAT["error"]/60, color="blue", label="FLAT")
plt.scatter(x=df_FLAT["step"], y=df_FLAT["last angle"]-60, color="blue", label="FLAT")
# plt.legend("FLAT")

# plt.scatter(x=df_CAVS["step"], y=df_CAVS["error"]/60, color="red", label="CAVS")
plt.scatter(x=df_CAVS["step"], y=df_CAVS["last angle"]-60, color="red", label="CAVS")
# plt.legend("CAVS")

plt.ylabel("last angle", fontsize=18)
plt.xlabel("step", fontsize=18)

# plt.xscale("log")
# plt.xlim(0.0001, 0.06)
# plt.ylim(0, 60)
plt.grid(axis='x', which='major',color='black',linestyle='-', linewidth=1.5)
plt.grid(axis='x', which='minor',color='black',linestyle='dashed')
# plt.semilogx()
# plt.xticks([0.00312, 0.00562, 0.01, 0.0178, 0.0312, 0.0562])
# plt.hlines(10, 0, 0.11, linestyles="dashed")
plt.legend(bbox_to_anchor=(1, 0), loc='lower right', borderaxespad=1, fontsize=18)

plt.savefig("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/lastangle.png")
plt.show()