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
    directory_path = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0621/*/rosbag/"
    file_name = directory_path+ID+"*.bag"
    ls = glob.glob(file_name)
    ls = sorted(ls)
    result_dict = {}

    for l in ls:
        base, ext = os.path.splitext(l)

        if os.path.isfile(base+".png"):
            continue

        # The bag file should be in the same directory as your terminal
        bag = rosbag.Bag(l)
        topic = '/inhand'
        column_names = ['time', 'angle', "angular velocity", "gripper position", "gripper velocity", "process"]
        df = pd.DataFrame(columns=column_names)

        first_f=False
        for topic, msg, t in bag.read_messages(topics=topic):
            time = msg.header.stamp.to_sec()
            angle = msg.obj_orientation_filtered
            angular_velocity = msg.d_obj_orientation_filtered
            gripper_position = msg.interval
            manipulated_variable = msg.MV
            process_f = msg.process_f

            if not first_f:
                start_time = time
                first_f=True

            df = df.append(
                {'time': time-start_time,
                'angle': angle,
                'angular velocity': angular_velocity,
                "gripper position": gripper_position,
                "gripper velocity": manipulated_variable,
                "process": process_f},
                ignore_index=True
            )
        result_dict[base] = [df, base, ext]
        print(base, "is loaded.")
    return result_dict


fs = 30
ls = 20
lgs = 20

ID = "CAVS"



result_dict = create_df(ID)


for item in result_dict.values():
    df = item[0]
    base = item[1]
    ext = item[2]

    # df = df[df["time"]>6]

    # df["time"] = df["time"] - 6


    fig, axes = plt.subplots(nrows=4, ncols=1, figsize=(16, 12))
    df.plot(x="time", y=['angle', "angular velocity", "gripper position", "gripper velocity"],subplots=True, ax=axes)

    x_min=0
    x_max=df["time"].tail(1)

    axes[0].legend(bbox_to_anchor=(0, 1), loc='upper left', borderaxespad=0, fontsize=lgs)
    axes[0].hlines(60, x_min, x_max, linestyles='dashed')
    axes[0].yaxis.set_ticks([0, 30, 60, 90]) 
    axes[0].set_ylabel("[deg]", fontsize=fs)
    axes[0].set_xlabel('')
    axes[0].tick_params(labelbottom=False)
    axes[0].set_ylim(-5, 90)
    axes[0].tick_params(labelsize=ls) 

    axes[1].legend(bbox_to_anchor=(0, 1), loc='upper left', borderaxespad=0, fontsize=lgs)
    axes[1].hlines(15, x_min, x_max, linestyles='dashed')
    axes[1].hlines(0, x_min, x_max, linestyles='dashed')
    # axes[1].yaxis.set_ticks([0, 10, 30, 50]) 
    axes[1].set_ylabel("[deg/s]", fontsize=fs)
    axes[1].set_xlabel('')
    axes[1].set_ylim(-5, 200)
    axes[1].tick_params(labelsize=ls) 
    axes[1].tick_params(labelbottom=False)

    axes[2].legend(bbox_to_anchor=(0, 1), loc='upper left', borderaxespad=0, fontsize=lgs)
    axes[2].yaxis.set_ticks([15, 20, 25, 30])
    axes[2].set_ylabel("[mm]", fontsize=fs)
    axes[2].set_xlabel('')
    axes[2].set_ylim(20, 30)
    axes[2].tick_params(labelsize=ls)
    axes[2].tick_params(labelbottom=False)

    axes[3].legend(bbox_to_anchor=(0, 1), loc='upper left', borderaxespad=0, fontsize=lgs)
    axes[3].hlines(0, x_min, x_max, linestyles='dashed')
    MV_open=df["gripper velocity"].max()
    MV_ticks=[-2.5, 0, MV_open]
    axes[3].yaxis.set_ticks(MV_ticks)
    axes[3].set_ylabel("[mm/s]", fontsize=fs)
    axes[3].set_ylim(-2.55, MV_open+1)
    axes[3].tick_params(labelsize=ls)
    axes[3].set_xlabel('time [s]', fontsize=fs)

    df.to_csv(base+".csv")
    plt.savefig(base+".eps")
    plt.savefig(base+".png")
# plt.show()