#!/usr/bin/python
#coding: utf-8
# 参考
# https://answers.ros.org/question/55037/is-there-a-way-to-save-all-rosbag-data-into-a-csv-or-text-file/

# 回転が起きた瞬間を切り取って比較する画像を作る

import sys
sys.path.append("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/src")
import rosbag
from rubbing_hand.msg import inhand
import pandas as pd
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import glob
import os

def create_df(ID):
    global base, ext
    directory_path = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0419/*/rosbag/"
    file_name = directory_path+ID+"*.bag"
    l = glob.glob(file_name)[-1]

    base, ext = os.path.splitext(l)

    # The bag file should be in the same directory as your terminal
    bag = rosbag.Bag(l)
    topic = '/inhand'
    column_names = ['time', 'angle', "angular velocity", "gripper position", "gripper velocity"]
    df = pd.DataFrame(columns=column_names)

    first_f=False
    for topic, msg, t in bag.read_messages(topics=topic):
        time = msg.header.stamp.to_sec()
        angle = msg.obj_orientation_filtered
        angular_velocity = msg.d_obj_orientation_filtered
        gripper_position = msg.interval
        manipulated_variable = msg.MV

        if not first_f:
            start_time = time
            first_f=True

        df = df.append(
            {'time': time-start_time,
            'angle': angle,
            'angular velocity': angular_velocity,
            "gripper position": gripper_position,
            "gripper velocity": manipulated_variable*50},
            ignore_index=True
        )
    print(ID, ":ok")
    return df

# 一番角速度が大きいとこで時間軸合わせる
def Shift_time(df):
    # df = df[:500]
    index = df['angular velocity'].idxmax()
    time = df["time"].iloc[index] - 0.5
    df = df[df["time"]>time]
    df["time"] = df["time"] - time
    return df

global base, ext

fs = 30
ls = 20
lgs = 20

c=[0 for i in range(6)]
f=[0 for i in range(6)]

ID = "CAVS00"
c[0] = create_df(ID)
ID = "CAVS01"
c[1] = create_df(ID)
ID = "CAVS02"
c[2] = create_df(ID)
ID = "CAVS03"
c[3] = create_df(ID)
ID = "CAVS04"
c[4] = create_df(ID)
ID = "CAVS05"
c[5] = create_df(ID)

ID = "CAVSst00"
f[0] = create_df(ID)
ID = "CAVSst01"
f[1] = create_df(ID)
ID = "CAVSst02"
f[2] = create_df(ID)
ID = "CAVSst03"
f[3] = create_df(ID)
ID = "CAVSst04"
f[4] = create_df(ID)
ID = "CAVSst05"
f[5] = create_df(ID)

# df = df[df["time"]>6]
# df["time"] = df["time"] - 6

for i in range(len(f)):
    f[i] = Shift_time(f[i])
for i in range(len(c)):
    c[i] = Shift_time(c[i])

# fig, axes = plt.subplots(nrows=4, ncols=1, figsize=(16, 12))
fig, ax = plt.subplots(figsize=(16, 12))
for df in c:
    df.plot(x="time", y="angular velocity", ax=ax, color="red", legend=False)

for df in f:
    df.plot(x="time", y="angular velocity", ax=ax, color="blue", legend=False)
x_min=0
x_max=1

# axes[0].legend(bbox_to_anchor=(0, 1), loc='upper left', borderaxespad=0, fontsize=lgs)
# axes[0].hlines(60, x_min, x_max, linestyles='dashed')
# axes[0].yaxis.set_ticks([0, 30, 60, 90]) 
# axes[0].set_ylabel("[deg]", fontsize=fs)
# axes[0].set_xlabel('')
# axes[0].tick_params(labelbottom=False)
# axes[0].set_ylim(-5, 90)
# axes[0].tick_params(labelsize=ls) 

# ax.legend(bbox_to_anchor=(0, 1), loc='upper left', borderaxespad=0, fontsize=lgs)
# ax.hlines(10, x_min, x_max, linestyles='dashed')
# ax.hlines(0, x_min, x_max, linestyles='dashed')
# # axes[1].yaxis.set_ticks([0, 10, 30, 50]) 
ax.set_ylabel("angular velocity [deg/s]", fontsize=fs)
ax.set_xlabel('time [s]', fontsize=fs)
# ax.set_ylim(-5, 200)
ax.tick_params(labelsize=ls) 
# ax.tick_params(labelbottom=False)

# axes[2].legend(bbox_to_anchor=(0, 1), loc='upper left', borderaxespad=0, fontsize=lgs)
# axes[2].yaxis.set_ticks([15, 20, 25, 30])
# axes[2].set_ylabel("[mm]", fontsize=fs)
# axes[2].set_xlabel('')
# axes[2].set_ylim(20, 30)
# axes[2].tick_params(labelsize=ls)
# axes[2].tick_params(labelbottom=False)

# axes[3].legend(bbox_to_anchor=(0, 1), loc='upper left', borderaxespad=0, fontsize=lgs)
# axes[3].hlines(0, x_min, x_max, linestyles='dashed')
# MV_open=df["gripper velocity"].max()
# MV_ticks=[-0.5, 0, MV_open]
# axes[3].yaxis.set_ticks(MV_ticks)
# axes[3].set_ylabel("[mm/s]", fontsize=fs)
# axes[3].set_ylim(-0.55, MV_open+1)
# axes[3].tick_params(labelsize=ls)
# axes[3].set_xlabel('time [s]', fontsize=fs)

# plt.savefig(base+"4.eps")
# plt.savefig(base+"4.png")
plt.show()