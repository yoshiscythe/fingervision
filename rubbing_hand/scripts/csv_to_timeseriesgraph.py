#!/usr/bin/python
#coding: utf-8

import pandas as pd
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import glob
import os


def create_df(ID):
    directory_path = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/1121/*/rosbag/"
    ls = []
    if isinstance(ID, list):
        for id in ID:
            file_name = directory_path+id+"*.csv"
            ls += glob.glob(file_name)
    else:
        file_name = directory_path+ID+"*.csv"
        ls += glob.glob(file_name)
    # 重複削除
    ls = list(set(ls))
    # ソート
    ls = sorted(ls)
    result_dict = {}

    for l in ls:
        base, ext = os.path.splitext(l)

        if os.path.isfile(base+"*.png"):
            continue

        df = pd.read_csv(l)

        result_dict[base] = [df, base, ext]
        # print(base+" is loaded.")

    return result_dict


def Create_and_save_graph(df, base):
    start_index = list(df[df["process"]>1].index)[0]
    start_time = df.iloc[start_index]["time"]
    finish_index = list(df[df["process"]>0].index)[-1]
    df = df[start_index-30:finish_index-60]
    df["time"] = df["time"] - start_time

    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(16, 12), sharex="all")
    df.plot(x="time", y=['angle', "angular velocity", "gripper position"],subplots=True, ax=axes, legend=False)

    x_min=-1
    x_max=df["time"].tail(1)
    # x_max=15

    plt.rcParams["mathtext.fontset"] = "cm"

    axes[0].hlines(60, x_min, x_max, linestyles='dashed')
    axes[0].yaxis.set_ticks(np.arange(0, 91, 15)) 
    axes[0].set_ylabel("angle\n $"+r"\theta$ [deg]", fontsize=fs)
    axes[0].set_xlabel('')
    axes[0].tick_params(labelbottom=False)
    axes[0].set_ylim(-5, 90)
    axes[0].tick_params(labelsize=ls) 

    axes[1].hlines(15, x_min, x_max, linestyles='dashed')
    axes[1].hlines(0, x_min, x_max, linestyles='dashed')
    # axes[1].yaxis.set_ticks([0, 10, 30, 50]) 
    axes[1].set_ylabel("angular velocity\n"+r"$\dot{\theta}$ [deg/s]", fontsize=fs)
    axes[1].set_xlabel('')
    axes[1].set_ylim(-5, 300)
    axes[1].tick_params(labelsize=ls) 
    axes[1].tick_params(labelbottom=False)

    axes[2].yaxis.set_ticks(np.arange(0, 50, 2))
    axes[2].set_ylabel("gripper position\n"+r"$p$ [mm]", fontsize=fs)
    axes[2].set_xlabel('time $t$ [s]', fontsize=fs)
    axes[2].set_ylim(df["gripper position"].min() -1, df["gripper position"].min() +16)
    axes[2].tick_params(labelsize=ls)
    # axes[2].tick_params(labelbottom=False)

    # plt.xlim(-0.5, 14.5)

    plt.savefig(base+"_3series.eps")
    plt.savefig(base+"_3series.png")

    # plt.show()
    print(base+" is saved.")

    plt.cla()


ID = "CAVS"
fs = 30
ls = 20
lgs = 20

result_dict = create_df(ID)

for item in result_dict.values():
    df = item[0]
    base = item[1]
    ext = item[2]

    try:
        Create_and_save_graph(df, base)
    except:
        print("error: "+base)