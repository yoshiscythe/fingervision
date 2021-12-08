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
        plt.text(x, y, s, size=50, color=cm[i])
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

def separate_df(df):
    df_dict = {}
    

hz = 42.3
fz =50
offsettime = 10 #FLAT:5, CAVS:10

df = pd.read_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0117/inhand0117CAVS0.01_shusei.csv", header=0)
df = df.iloc[int(hz*offsettime):, :].reset_index(drop=True)
orientations = -df.iloc[:, ::2]
# orientations = orientations.iloc[:570]
intervals = df.iloc[:, 1::2]
# df = df.iloc[:, :28]
# df = df.iloc[:800]

cm_blue=generate_cm(plt.get_cmap("Blues"),len(orientations.columns))

for i in range(len(orientations.columns)):
    print(i)
    plt.scatter(x=intervals.iloc[:,i], y=orientations.iloc[:,i], s=5, c=cm_blue[i])


save_file_name = "inhand0117CAVS0.01_vs_position"
plt.savefig(save_file_name+".png")
plt.savefig(save_file_name+".eps")
plt.show()