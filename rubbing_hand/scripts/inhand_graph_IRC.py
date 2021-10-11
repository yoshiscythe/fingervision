# -*- coding: utf-8 -*-
import pandas as pd
import glob
import matplotlib as mpl
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import numpy as np

def generate_cm(cm, num):
    cm_array = np.linspace(0.3,1.0,num)
    color_map = [cm(x) for x in cm_array]

    return color_map

def print_target_position(df):
    cm = generate_cm(cm_red,len(df.columns)-1)
    i = 0
    offset = [0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0]  # CAVS
    offset_y = [0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0]
    for column_name, item in df.iteritems():
        if column_name == "time":
            continue
        x = 9.5 + offset[i]*1.5
        y = item[378] + offset_y[i]*3
        s = column_name
        plt.text(x, y, s, size=40, color=cm[i])
        i+=1
    # plt.text(9.5, 65, "target position", size=25)

def print_target_position_FLAT(df):
    cm = generate_cm(cm_red,len(df.columns)-1)
    i = 0
    _y = [-6, -4.8, 5.3, 0.5, 5.3, 50, 60]
    _x = [12, 13.5, 12, 13.5, 13.5, 6, 11.5]
    for column_name, item in df.iteritems():
        if column_name == "time":
            continue
        x = _x[i]
        y = _y[i]
        s = column_name
        plt.text(x, y, s, size=40, color=cm[i])
        i+=1
    # plt.text(9.5, 65, "target position", size=25)

hz = 42.3
fz =40
offsettime = 10 #FLAT:5, CAVS:10

df = pd.read_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0117/inhand0117CAVS0.01_shusei.csv", header=0)
df = df.iloc[int(hz*offsettime):, :].reset_index(drop=True)
orientations = -df.iloc[:, ::2]
# orientations = orientations.iloc[:570]
intervals = df.iloc[:, 1::2]
# df = df.iloc[:, :28]
# df = df.iloc[:800]



# orientations = orientations.iloc[400:, :].reset_index(drop=True)


legend_list = orientations.columns.values
time_label = pd.Series(np.arange(len(orientations))/hz)

orientations["time"] = time_label

cm_blue = plt.get_cmap("Blues")
cm_red = plt.get_cmap("Reds")

# print(orientations)

#colormapサンプル
#https://matplotlib.org/examples/color/colormaps_reference.html
ax = orientations.plot(x="time", color=generate_cm(cm_blue,len(orientations.columns)-1), figsize=(18, 10), xlim=[0,15], ylim=[-5, 70], fontsize=fz, legend=False, linestyle='solid')

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
orientations_goal.plot(x="time", color=generate_cm(cm_red,len(orientations.columns)-1), ax = ax, legend = False, fontsize=fz, linestyle='solid')

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
# lg=ax.legend(legend_list, fontsize=25, loc="upper right", title="target position")
# lg.get_title().set_fontsize(20)

print_target_position(orientations)

save_file_name = "inhand0117CAVS0.01_IRC2021_legends"
plt.savefig(save_file_name+".png")
plt.savefig(save_file_name+".eps")
plt.show()