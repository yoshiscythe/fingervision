#!/usr/bin/python
#coding: utf-8
# 参考
# https://answers.ros.org/question/55037/is-there-a-way-to-save-all-rosbag-data-into-a-csv-or-text-file/

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
    directory_path = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/*/rosbag/"
    file_name = directory_path+ID+"*.bag"
    l = glob.glob(file_name)[-1]

    base, ext = os.path.splitext(l)

    # The bag file should be in the same directory as your terminal
    bag = rosbag.Bag(l)
    topic = '/inhand'
    column_names = ['time', 'angle', "angular velocity", "gripper position", "manipulated variable"]
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
            "manipulated variable": manipulated_variable},
            ignore_index=True
        )
    return df

global base, ext

ID = "FLAT18"
df = create_df(ID)
df = df[df["time"]>4]
df["time"] = df["time"] - 4
df_flat = df

ID = "CAVS33"
df = create_df(ID)
df = df[df["time"]>4]
df["time"] = df["time"] - 4
df_cavs = df


fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(12, 6))
df_cavs.plot(x="time", y=["angle", "angular velocity"], subplots=True, ax=axes[:][0])
df_flat.plot(x="time", y=["angle", "angular velocity"], subplots=True, ax=axes[:][1])

x_min=0
x_max=df["time"].tail(1)

axes[0][0].legend(bbox_to_anchor=(1, 0.1), loc='lower right', borderaxespad=0, fontsize=12)
axes[0][0].hlines(60, x_min, x_max, linestyles='dashed')
axes[0][0].yaxis.set_ticks([0, 30, 60, 90]) 
axes[0][0].set_ylabel("[deg]")
axes[0][0].set_ylim(-5, 90)

axes[0][1].legend(bbox_to_anchor=(1, 0.9), loc='upper right', borderaxespad=0, fontsize=12)
axes[0][1].hlines(10, x_min, x_max, linestyles='dashed')
axes[0][1].hlines(0, x_min, x_max, linestyles='dashed')
# axes[1].yaxis.set_ticks([0, 10, 30, 50]) 
axes[0][1].set_ylabel("[deg/s]")
axes[0][1].set_ylim(-5, 200)

axes[1][0].legend(bbox_to_anchor=(1, 0.1), loc='lower right', borderaxespad=0, fontsize=12)
axes[1][0].hlines(60, x_min, x_max, linestyles='dashed')
axes[1][0].yaxis.set_ticks([0, 30, 60, 90]) 
axes[1][0].set_ylabel("[deg]")
axes[1][0].set_ylim(-5, 90)

axes[1][1].legend(bbox_to_anchor=(1, 0.9), loc='upper right', borderaxespad=0, fontsize=12)
axes[1][1].hlines(10, x_min, x_max, linestyles='dashed')
axes[1][1].hlines(0, x_min, x_max, linestyles='dashed')
# axes[1].yaxis.set_ticks([0, 10, 30, 50]) 
axes[1][1].set_ylabel("[deg/s]")
axes[1][1].set_ylim(-5, 200)

plt.xlabel('time [s]', fontsize=12)

# plt.savefig(base+"2.eps")
# plt.savefig(base+"2.png")
plt.show()