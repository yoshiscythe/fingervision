#!/usr/bin/python
#coding: utf-8

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator

df_CAVS = pd.read_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/CAVS_error.csv")
df_CAVS = df_CAVS.astype(float)
df_CAVS["step"] = df_CAVS["step"]*50
df_FLAT = pd.read_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/FLAT_error.csv")
df_FLAT["step"] = df_FLAT["step"]*50

plt.scatter(x=df_FLAT["step"], y=df_FLAT["error"]/60, color="blue", label="FLAT")
plt.scatter(x=df_CAVS["step"], y=df_CAVS["error"]/60, color="red", label="CAVS")

plt.ylim=([0,20000])
plt.xlim=([0.0001, 0.01])

plt.yticks(np.arange(0,20001, 5000))

plt.ylabel("error", fontsize=18)
plt.xlabel("open velocity [mm/s]", fontsize=18)

plt.legend(bbox_to_anchor=(1, 0), loc='lower right', borderaxespad=1, fontsize=18)

# plt.savefig("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/result2.png")
plt.show()