#!/usr/bin/python
#coding: utf-8

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv('/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0221/matome_data.csv')

plt.scatter(x=df["FLAT step"], y=df["FLAT error"]/60, color="blue", label="FLAT")
# plt.legend("FLAT")

plt.scatter(x=df["CAVS step"], y=df["CAVS error"]/60, color="red", label="CAVS")
# plt.legend("CAVS")

plt.ylabel("error", fontsize=18)
plt.xlabel("step", fontsize=18)

# plt.xscale("log")
plt.xlim(0.0009, 0.11)
plt.ylim(0, 25)
plt.grid(axis='x', which='major',color='black',linestyle='-', linewidth=1.5)
plt.grid(axis='x', which='minor',color='black',linestyle='dashed')
plt.semilogx()
# plt.xticks([0.00312, 0.00562, 0.01, 0.0178, 0.0312, 0.0562])
plt.hlines(10, 0, 0.11, linestyles="dashed")
plt.legend(bbox_to_anchor=(0, 1), loc='upper left', borderaxespad=1, fontsize=18)

plt.savefig("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0221/matome_data.png")
plt.show()