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
        df_tmp["angular_velocity"] = df_tmp["angle"].diff().rolling(5, center=True).mean()*hz  # angleの移動平均をangular velocityにしてる
        df_dict[itv] = df_tmp
    return df_dict

def calcurate_error(df, itv):
    error_dict = OrderedDict()

    max_angular_velocity = df["angular_velocity"].max()
    time_at_max_omega =  df['angular_velocity'].idxmax()
    error_dict["max_angular_velocity"] = max_angular_velocity
    error_dict["time_at_max_omega"] = df["time"].iloc[time_at_max_omega]

    time_at_finish = list(df.loc[(df['interval'] >= float(itv))].index)[0]
    lug = time_at_max_omega - time_at_finish
    error_dict["time_at_finish"] = df["time"].iloc[time_at_finish]
    error_dict["lug"] = lug

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

        create_graph(df, result_dict)
    return df_analyze

def create_graph(df, result_dict):
    base = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0117/test/"
    fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(16, 12))
    df.plot(x="time", y=['angle', "angular_velocity"],subplots=True, ax=[axes[0][0], axes[1][0]], legend=False)
    df.plot(x="interval", y=['angle', "angular_velocity"],subplots=True, ax=[axes[0][1], axes[1][1]], legend=False)

    x_min=-10
    x_max=20

    axes[0][0].set_ylim(-5, 90)
    axes[0][1].set_ylim(-5, 90)
    axes[1][0].set_ylim(-5, 200)
    axes[1][1].set_ylim(-5, 200)

    axes[0][0].vlines(result_dict["time_at_max_omega"], x_min, x_max, linestyles='solid')
    axes[0][0].vlines(result_dict["time_at_finish"], x_min, x_max, linestyles='dashed')

    axes[0][1].vlines(result_dict["time_at_max_omega"], x_min, x_max, linestyles='solid')
    axes[0][1].vlines(result_dict["time_at_finish"], x_min, x_max, linestyles='dashed')

    save_name = result_dict["surface"]+result_dict["itv"]

    plt.savefig(base+save_name+".eps")
    plt.savefig(base+save_name+".png")

    # plt.show()
    print(save_name+" is saved.")

    plt.cla()

fz =50

df_CAVS = create_df("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0117/inhand0117CAVS0.01_shusei.csv", True)
df_FLAT = create_df("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0117/inhand0117FLAT0.01_shusei.csv", False)

df_analyze_CAVS = create_df_analyze(df_CAVS, "CAVS")
df_analyze_FLAT = create_df_analyze(df_FLAT, "FLAT")

df_analyze = pd.concat([df_analyze_CAVS, df_analyze_FLAT])

save_file_name = "inhand0117CAVS0.01_IRC2021_big"
# plt.savefig(save_file_name+".png")
# plt.savefig(save_file_name+".eps")
# plt.show()