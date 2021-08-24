# -*- coding: utf-8 -*-
import pandas as pd
import glob
import matplotlib as mpl
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import numpy as np

hz = 42.3
fz =40
offsettime = 5 #FLAT:5, CAVS:10

df = pd.read_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0117/inhand0117FLAT0.01_shusei.csv", header=0)
df = df.iloc[int(hz*offsettime):, :].reset_index(drop=True)
orientations = -df.iloc[:, ::2]
intervals = df.iloc[:, 1::2]
# df = df.iloc[:, :28]
# df = df.iloc[:800]



# orientations = orientations.iloc[400:, :].reset_index(drop=True)


legend_list = orientations.columns.values
time_label = pd.Series(np.arange(len(orientations))/hz)

orientations["time"] = time_label

#colormapサンプル
#https://matplotlib.org/examples/color/colormaps_reference.html
ax = orientations.plot(x="time", colormap='Blues', figsize=(18, 10), xlim=[0,15], ylim=[-5, 70], fontsize=fz, legend=True, linestyle='solid')

orientations_goal = []
interval_list = []
for i in range(len(orientations.columns)-1):
    orientation = orientations.iloc[:,i]
    interval = intervals.iloc[:,i]
    interval_th = float(orientation.name)

    orientation2 = orientation.loc[interval >= interval_th]
    # print(orientation2)
    interval_list.append(orientation2.index[0]/hz)
    # orientation2.plot(color = "r",  ax = ax)
    orientations_goal.append(orientation2)

orientations_goal = pd.concat(orientations_goal, axis=1, sort=True)
orientations_goal["time"] = time_label
orientations_goal.plot(x="time", colormap="Reds", ax = ax, legend = True, fontsize=fz, linestyle='solid')

# ax = orientations_goal.plot(x="time", colormap="winter", legend = True, fontsize=fz, linestyle='solid', linewidth=3)
# orientations.plot(x="time", colormap='summer', figsize=(18, 12), xlim=[0,15], ylim=[-10, 90], fontsize=fz, legend=True, linestyle='dashed', ax = ax, linewidth=2)


# # Select the color map named rainbow
# cmap = cm.get_cmap('Reds', len(interval_list))
# cmap_list = [cmap(i) for i in range(len(interval_list))]

# plt.vlines(interval_list, -10, 90, color=cmap_list)
# print(interval_list)

ax.set_xlabel(r"time $t$ [s]", fontsize = fz)
ax.set_xticks(np.arange(0, 16, 5))
ax.set_ylabel(r"angle $\theta$ [deg]", fontsize = fz)
lg=ax.legend(legend_list, fontsize=25, loc="upper right", title="target position")
lg.get_title().set_fontsize(20)

save_file_name = "inhand0117FLAT0.01_IRC2021_mod"
plt.savefig(save_file_name+".png")
plt.savefig(save_file_name+".eps")
plt.show()