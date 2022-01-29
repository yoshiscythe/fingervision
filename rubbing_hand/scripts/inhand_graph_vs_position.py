# -*- coding: utf-8 -*-
import pandas as pd
import glob
import matplotlib as mpl
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import numpy as np
from collections import OrderedDict

def create_df(path, surface):
    df = pd.read_csv(path, header=0)
    hz = 42.3
    offsettime = 10 if surface else 5  #FLAT:5, CAVS:10
    df = df.iloc[int(hz*offsettime):, :].reset_index(drop=True)
    df_dict = OrderedDict()
    for i in range(len(df.columns)/2):
        df_tmp = df.iloc[:,i*2:i*2+2].reset_index(drop= True).dropna(how='any')
        itv = df_tmp.columns[0]
        df_tmp.set_axis(['angle', 'interval'], axis='columns', inplace=True)
        df_tmp['angle'] = -df_tmp['angle'].rolling(5, center=True).mean()
        df_tmp["time"] = [i/hz for i in df_tmp.index.tolist()]
        df_tmp["angular_velocity"] = df_tmp["angle"].diff().rolling(10, center=True).mean()*hz  # angleの移動平均をangular velocityにしてる
        df_dict[itv] = df_tmp
    return df_dict

def generate_cm(cm, num):
    cm_array = np.linspace(0.3,1.0,num)
    color_map = [cm(x) for x in cm_array]

    return color_map

def print_target_position_CAVS(df):
    i = 0
    offset = [0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0]  # CAVS
    offset_y = [0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0]
    for key, value in df_dict.items():
        itv = key
        df = value.dropna(how='any')
        x = 9.5 + offset[i]*1.5
        y = df["angle"].tail(1) + offset_y[i]*3
        s = itv
        plt.text(x, y, s, size=50, color=cm_red[i])
        i+=1
    # plt.text(9.5, 65, "target position", size=25)

def print_target_position_itv_CAVS(df):
    i = 0
    offset = [0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0]  # CAVS
    offset_y = [0, 1,2, 3, 4, 5, 0, 0, 0, 0, 0, 0]
    for key, value in df_dict.items():
        itv = key
        df = value.dropna(how='any')
        x = float(itv)
        y = df["angle"].tail(1) + offset_y[i]*3 + 5
        s = itv
        plt.text(x, y, s, size=50, color=cm_red[i], horizontalalignment="center")
        i+=1
    # plt.text(9.5, 65, "target position", size=25)

def print_target_position_FLAT(df):
    i = 0
    _y = [-6, -4.8, 5.3, 0.5, 5.3, 40, 40]
    _x = [12, 13.5, 12, 13.5, 13.5, 6, 11.5]
    for key, value in df_dict.items():
        itv = key
        x = _x[i]
        y = _y[i]
        s = itv
        plt.text(x, y, s, size=40, color=cm_red[i])
        i+=1
    # plt.text(9.5, 65, "target position", size=25)

def print_target_position_itv_FLAT(df):
    i = 0
    _y = [5, 10, 5, 10, 5, 40, 45]
    _x = [12, 13.5, 12, 13.5, 13.5, 6, 11.5]
    for key, value in df_dict.items():
        itv = key
        x = float(itv)
        y = _y[i]
        s = itv
        plt.text(x, y, s, size=40, color=cm_red[i], horizontalalignment="center")
        i+=1
    # plt.text(9.5, 65, "target position", size=25)

def separate_df(df):
    df_dict = {}
    

hz = 42.3
fz =50
offsettime = 10 #FLAT:5, CAVS:10

df_CAVS = create_df("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0117/inhand0117CAVS0.01_shusei.csv", True)
df_FLAT = create_df("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0117/inhand0117FLAT0.01_shusei.csv", False)

df_dict = df_FLAT

cm_blue = generate_cm(plt.get_cmap("Blues"),len(df_dict))
cm_red = generate_cm(plt.get_cmap("Reds"),len(df_dict))

# cm_blue=generate_cm(plt.get_cmap("Blues"),len())

fig = plt.figure(figsize=(18, 10))
ax = fig.add_subplot(1,1,1)
i = 0
for key, value in df_dict.items():
    itv = key
    df = value
    df_open = df[df["interval"] < float(itv)]
    df_stay = df[df["interval"] >= float(itv)]
    
    df_open.plot(x="interval", y="angle", ax = ax, color = cm_blue[i], legend = False, fontsize=fz, linestyle='solid')
    df_stay.plot(x="interval", y="angle", ax = ax, color = cm_red[i], legend = False, fontsize=fz, linestyle='solid')


    i += 1

# ax.set_xlim(0, 15)
ax.set_ylim(-5, 70)
ax.set_xlabel(r"gripper position $p$ [mm]", fontsize = fz)
# ax.set_xticks(np.arange(0, 16, 5))
# ax.set_xticks(np.arange(20, 23.6, 0.5))
ax.set_xticks(np.arange(18, 20.1, 0.5))
ax.set_ylabel(r"angle $\theta$ [deg]", fontsize = fz)
plt.subplots_adjust(bottom=0.15)

# print_target_position_itv_FLAT(df_dict)

save_file_name = "inhand0117FLAT0.01_interval"
plt.savefig(save_file_name+".png")
plt.savefig(save_file_name+".eps")
plt.show()