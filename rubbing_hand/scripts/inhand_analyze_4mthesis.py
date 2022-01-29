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
        df_tmp = df.iloc[:,i*2:i*2+2].reset_index(drop= True).dropna(how='all')
        itv = df_tmp.columns[0]
        df_tmp.set_axis(['angle', 'interval'], axis='columns', inplace=True)
        df_tmp['angle'] = -df_tmp['angle'].rolling(5, center=True).mean()
        df_tmp["time"] = [i/hz for i in df_tmp.index.tolist()]
        df_tmp["angular_velocity"] = df_tmp["angle"].diff().rolling(10, center=True).mean()*hz  # angleの移動平均をangular velocityにしてる
        df_dict[itv] = df_tmp
    return df_dict

def calcurate_error(df, itv):
    error_dict = OrderedDict()

    max_angular_velocity = df["angular_velocity"].max()
    time_at_max_omega =  df['angular_velocity'].idxmax()
    error_dict["max_angular_velocity"] = max_angular_velocity
    error_dict["time_at_max_omega"] = df["time"].iloc[time_at_max_omega]

    index_at_finish = list(df.loc[(df['interval'] >= float(itv))].index)[0]
    lug = time_at_max_omega - index_at_finish
    time_at_finish = df["time"].iloc[index_at_finish]
    error_dict["time_at_finish"] = time_at_finish
    error_dict["lug"] = lug

    after_list = list((df[df['interval'] >= float(itv)])["angle"].dropna(how='any'))
    moving_angle_after = after_list[-1] - after_list[0]
    error_dict["moving_angle_after"] = moving_angle_after

    angle_at_fin =  after_list[0]
    time_at_overlist = list(df[(df['interval'] >= float(itv)) & (df['angle'] >= angle_at_fin + 10)]["time"])
    slip_lug = time_at_overlist[0] - time_at_finish if time_at_overlist else 0
    error_dict["slip_lug"] = slip_lug

    time_at_overvellist = list(df[(df['interval'] >= float(itv)) & (df['angular_velocity'] >= 15)]["time"])
    slip_lug_angular = time_at_overvellist[0] - time_at_finish if time_at_overvellist else 0
    error_dict["slip_lug_angular"] = slip_lug_angular

    return error_dict

def create_df_analyze(df_dict, surface):
    progress = 0
    for key, value in df_dict.items():
        itv = key
        df = value
        result_dict = OrderedDict()
        result_dict["itv"]=itv
        result_dict["surface"] = surface
        error_dict = calcurate_error(df, itv)
        result_dict.update(error_dict)
        
        # 初回は，いい感じのカラム名がついたpdデータフレームを作成
        if progress == 0:
            progress = 1
            column_names = list(result_dict.keys())
            df_analyze = pd.DataFrame(columns=column_names)
        df_analyze = df_analyze.append(result_dict,ignore_index=True)

        # create_graph(df, result_dict)
    return df_analyze

def create_graph(df, result_dict):
    figsize_x = 24
    figsize_y = 20
    base = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0117/test2/"
    fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(figsize_x, figsize_y), sharex="col", sharey="row")

    df_open = df[df["interval"] < float(result_dict["itv"])]
    df_stay = df[df["interval"] >= float(result_dict["itv"])]

    df_open.plot(x="time", y=['angle', "angular_velocity"],subplots=True, ax=[axes[0][0], axes[1][0]], legend=False, color="b")
    df_open.plot(x="interval", y=['angle', "angular_velocity"],subplots=True, ax=[axes[0][1], axes[1][1]], legend=False, color="b")

    df_stay.plot(x="time", y=['angle', "angular_velocity"],subplots=True, ax=[axes[0][0], axes[1][0]], legend=False, color="r")
    df_stay.plot(x="interval", y=['angle', "angular_velocity"],subplots=True, ax=[axes[0][1], axes[1][1]], legend=False, color="r")

    y_min=-10
    y_max=20

    axes[0][0].set_ylim(-5, 70)
    axes[0][1].set_ylim(-5, 70)
    axes[1][0].set_ylim(-5, 200)
    axes[1][1].set_ylim(-5, 200)

    x_min = 0
    x_max = 9 if result_dict["surface"] == "CAVS" else 12
    itv_min = 20 if result_dict["surface"] == "CAVS" else 18
    itv_max = 23.5 if result_dict["surface"] == "CAVS" else 20

    axes[0][0].set_xlim(x_min, x_max)
    axes[1][0].set_xlim(x_min, x_max)
    axes[0][1].set_xlim(itv_min, itv_max)
    axes[1][1].set_xlim(itv_min, itv_max)

    fs = 30
    ls = 20
    lgs = 20

    axes[0][0].set_ylabel("angle\n $"+r"\theta$ [deg]", fontsize=fs)
    axes[0][0].yaxis.set_ticks(np.arange(0, 61, 20))
    axes[0][0].tick_params(labelsize=ls) 
    axes[1][0].set_ylabel("angular velocity\n $"+r"\dot{\theta}$ [deg/s]", fontsize=fs)
    axes[1][0].set_xlabel("time\n $"+r"t$ [s]", fontsize=fs)
    axes[1][0].yaxis.set_ticks(np.arange(0, 201, 100))
    axes[1][0].tick_params(labelsize=ls) 
    axes[1][1].set_xlabel("gripper position\n $"+r"p$ [mm]", fontsize=fs)
    axes[1][1].xaxis.set_ticks(np.arange(itv_min, itv_max+1, 0.5))
    axes[1][1].tick_params(labelsize=ls) 

    fig.subplots_adjust(bottom=0.3)

    save_name = str(figsize_x)+str(figsize_y)+result_dict["surface"]+result_dict["itv"]

    plt.savefig(base+save_name+".eps")
    plt.savefig(base+save_name+".png")

    # plt.show()
    print(save_name+" is saved.")

    plt.cla()

def create_hist(df):
    plt.cla()

    # ax1 = df[df["surface"]=="CAVS"]["max_angular_velocity"]
    # ax2 = df[df["surface"]=="FLAT"]["max_angular_velocity"]
    # edges = range(0, 160, 20)

    # plt.hist([ax1, ax2], bins=edges, label=["CAVS", "FLAT"])
    # plt.legend(loc="upper right", fontsize=13)
    # plt.xlabel("max angular velocity [deg/s]", fontsize=20)
    # plt.ylabel("frequency", fontsize=20)
    # plt.subplots_adjust(bottom=0.13)

    ax1 = df[(df["surface"]=="CAVS") & (df["slip_lug_angular"]>0)]["slip_lug_angular"]
    ax2 = df[(df["surface"]=="FLAT") & (df["slip_lug_angular"]>0)]["slip_lug_angular"]
    edges = range(0, 10, 1)

    plt.hist([ax1, ax2], bins=edges, label=["CAVS", "FLAT"])
    plt.legend(loc="upper right", fontsize=13)
    plt.xlabel("lug time [s]", fontsize=20)
    plt.ylabel("frequency", fontsize=20)
    plt.subplots_adjust(bottom=0.13)

    base = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0117/hist/"
    save_name = "hist_slip_lug"

    # plt.savefig(base+save_name+".eps")
    # plt.savefig(base+save_name+".png")

    plt.show()

fz =50

df_CAVS = create_df("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0117/inhand0117CAVS0.01_shusei.csv", True)
df_FLAT = create_df("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0117/inhand0117FLAT0.01_shusei.csv", False)

df_analyze_CAVS = create_df_analyze(df_CAVS, "CAVS")
df_analyze_FLAT = create_df_analyze(df_FLAT, "FLAT")

df_analyze = pd.concat([df_analyze_CAVS, df_analyze_FLAT])

create_hist(df_analyze)

df_analyze.to_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0117/analyze.csv")

# save_file_name = "inhand0117CAVS0.01_IRC2021_big"
# plt.savefig(save_file_name+".png")
# plt.savefig(save_file_name+".eps")
# plt.show()