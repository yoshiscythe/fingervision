#!/usr/bin/python
#coding: utf-8

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator

df_CAVS = pd.read_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0525/CAVS_error.csv")
df_CAVS = df_CAVS.astype(float)
# print(df_CAVS)
# df_CAVS = df_CAVS[df_CAVS["id"]>=88]
df_CAVS = df_CAVS.iloc[60,:]

print(df_CAVS)
fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(12, 6))

df_CAVS.plot(x="ratio", y="error", kind="scatter", ax=axes[0,0], sharex=True)
df_CAVS.plot(x="ratio", y="mean", kind="scatter", ax=axes[0,1], sharex=True)
df_CAVS.plot(x="ratio", y="std", kind="scatter", ax=axes[1,0], sharex=True)
df_CAVS.plot(x="ratio", y="max_angular_velocity", kind="scatter", ax=axes[1,1], sharex=True)
df_CAVS.plot(x="ratio", y="rms", kind="scatter", ax=axes[1,2], sharex=True)

plt.show()