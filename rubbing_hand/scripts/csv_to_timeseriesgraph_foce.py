#!/usr/bin/python
#coding: utf-8

import pandas as pd
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import glob
import os
import re

base_path = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/1212/"

def create_df(ID):
    directory_path = base_path+"*/rosbag/"
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

        id = re.match("^.*((CAVS|FLAT)(\d+)).*", base).group(1)
        # print(base,id)
        df = pd.read_csv(l)

        result_dict[id] = [df, base, ext]
        # print(base+" is loaded.")

    return result_dict

def create_ELF_df(ID):
    directory_path = base_path+"ELF/modified/"
    ls = []
    if isinstance(ID, list):
        for id in ID:
            file_name = directory_path+"*"+id+"*.csv"
            ls += glob.glob(file_name)
    else:
        file_name = directory_path+"*"+ID+"*.csv"
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

        id = re.match("^.*((CAVS|FLAT)(\d+)).*", base).group(1)
        df = pd.read_csv(l)

        result_dict[id] = [df, base, ext]
        # print(base+" is loaded.")

    return result_dict

def Create_and_save_graph(df, base, id, force_dict):
    start_index = list(df[df["process"]>1].index)[0]
    start_time = df.iloc[start_index]["time"]
    finish_index = list(df[df["process"]>0].index)[-1]
    df = df[start_index-30:finish_index-60]
    df["time"] = df["time"] - start_time

    df_force=force_dict[id][0]
    df_force["offset"] = df_force["offset"] - start_time


    fig, axes = plt.subplots(nrows=4, ncols=1, figsize=(16, 16), sharex="all")
    df.plot(x="time", y=['angle', "angular velocity", "gripper position"],subplots=True, ax=axes[:3], legend=False)
    force_dict[id][0].plot(x="offset", y="force", ax = axes[3], legend=False, )

    x_min=-1
    x_max=float(df["time"].tail(1))
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

    axes[2].yaxis.set_ticks(np.arange(0, 50, 5))
    axes[2].set_ylabel("gripper position\n"+r"$p$ [mm]", fontsize=fs)
    axes[2].set_xlabel('time $t$ [s]', fontsize=fs)
    axes[2].set_ylim(df["gripper position"].min() -1, df["gripper position"].min() +16)
    axes[2].tick_params(labelsize=ls)
    # axes[2].tick_params(labelbottom=False)

    # axes[3].yaxis.set_ticks(np.arange(0, 50, 2))
    axes[3].set_ylabel("grip force\n"+r"$f$ [N]", fontsize=fs)
    axes[3].set_xlabel('time $t$ [s]', fontsize=fs)
    axes[3].set_xlim(x_min, x_max)
    axes[3].set_ylim(0, 6)
    axes[3].tick_params(labelsize=ls)



    # plt.xlim(-0.5, 14.5)

    plt.savefig(base+"_4series.eps")
    plt.savefig(base+"_4series.png")

    # plt.show()
    print(base+" is saved.")

    plt.cla()

save_path=base_path+"force_graph/"
ID = "FLAT"
fs = 30
ls = 20
lgs = 20

result_dict = create_df(ID)
force_dict = create_ELF_df(ID)

for k, v in result_dict.items():
    id = k
    df = v[0]
    base = v[1]
    ext = v[2]
    base_name = os.path.basename(base)
    save_name = save_path+base_name
    
    # if os.path.isfile(save_name+"_4series.eps"):
    #     continue

    try:
        Create_and_save_graph(df, save_name, id, force_dict)
    except:
        print("error: "+base)