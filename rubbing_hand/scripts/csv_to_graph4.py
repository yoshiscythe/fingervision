#!/usr/bin/python
#coding: utf-8

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator

df_CAVS = pd.read_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0705/CAVS_error.csv")
df_FLAT = pd.read_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0705/FLAT_error.csv")

def create_error_bar_graph(df, x_label, x_label_display, y_label, y_label_display, ax, color, label):
    data_dict = {}
    for index, row in df.iterrows():
        row = row.to_dict()
        x_data = row[x_label]
        y_data = row[y_label]
        if not x_data in data_dict:
            data_dict[x_data] = [y_data]
        else:
            data_dict[x_data].append(y_data)
    x = []
    y_mean = []
    y_err = []
    for k, v in data_dict.items():
        print(k)
        x.append(k)
        y_mean.append(np.mean(v))
        y_err.append(np.std(v))
    print(x, y_mean, y_err)

    ax.errorbar(x, y_mean, yerr=y_err, capsize=4, fmt='o', ecolor=color, color=color, label=label)
    ax.set_xlabel(x_label_display, fontsize=fontsize)
    ax.set_ylabel(y_label_display, fontsize=fontsize)

def create_rmse_graph(df, x_label, x_label_display, y_label, y_label_display, ax, color, label):
    data_dict = {}
    for index, row in df.iterrows():
        row = row.to_dict()
        x_data = row[x_label]
        rmse_sum = row["rms_sum"]
        rmse_len = row["rms_len"]
        data = np.array([rmse_sum, rmse_len])
        if not x_data in data_dict:
            data_dict[x_data] = data
        else:
            data_dict[x_data] += data
    y = []
    x = []
    for k, v in data_dict.items():
        print(k)
        x.append(k)
        rmse = np.sqrt(v[0]/v[1])
        y.append(rmse)
    ax.scatter(x, y, color=color, label=label)
    ax.set_xlabel(x_label_display, fontsize=fontsize)
    ax.set_ylabel(y_label_display, fontsize=fontsize)

def create_rmse_graph2(df, x_label, x_label_display, y_label, y_label_display, ax, color, label):
    data_dict = {}
    for index, row in df.iterrows():
        row = row.to_dict()
        x_data = row[x_label]
        y_data = row[y_label]
        if not x_data in data_dict:
            data_dict[x_data] = [y_data]
        else:
            data_dict[x_data].append(y_data)
    x = []
    y = []
    for k, v in data_dict.items():
        print(k)
        rmse = np.sqrt(np.square(v).mean(axis=0))
        x.append(k)
        y.append(rmse)
    print(x, y)

    ax.scatter(x, y, color=color, label=label)
    ax.set_xlabel(x_label_display, fontsize=fontsize)
    ax.set_ylabel(y_label_display, fontsize=fontsize)

fig, axes = plt.subplots(2, 2, figsize=(16, 12))

CAVS_ruler_sin = df_CAVS[(df_CAVS["tool"] == 0) & (df_CAVS["method"] == 0)]
CAVS_ruler_linear = df_CAVS[(df_CAVS["tool"] == 0) & (df_CAVS["method"] == 1)]
CAVS_wood_sin = df_CAVS[(df_CAVS["tool"] == 1) & (df_CAVS["method"] == 0)]
CAVS_wood_linear = df_CAVS[(df_CAVS["tool"] == 1) & (df_CAVS["method"] == 1)]

FLAT_ruler_sin = df_FLAT[(df_FLAT["tool"] == 0) & (df_FLAT["method"] == 0)]
FLAT_ruler_linear = df_FLAT[(df_FLAT["tool"] == 0) & (df_FLAT["method"] == 1)]
FLAT_wood_sin = df_FLAT[(df_FLAT["tool"] == 1) & (df_FLAT["method"] == 0)]
FLAT_wood_linear = df_FLAT[(df_FLAT["tool"] == 1) & (df_FLAT["method"] == 1)]

fontsize=16

# df_C = CAVS_ruler_sin
# df_F = CAVS_ruler_linear
# create_error_bar_graph(df_C, "step", "max_angular_velocity", axes[0][1], "red", label="CAVS")
# create_error_bar_graph(df_F, "step", "max_angular_velocity", axes[0][1], "blue", label="FLAT")
# create_error_bar_graph(df_C, "step", "last_angle", axes[1][0], "red", label="CAVS")
# create_error_bar_graph(df_F, "step", "last_angle", axes[1][0], "blue", label="FLAT")
# create_error_bar_graph(df_C, "step", "elasped_time", axes[1][1], "red", label="CAVS")
# create_error_bar_graph(df_F, "step", "elasped_time", axes[1][1], "blue", label="FLAT")
# create_rmse_graph(df_C, "step", "rmse", axes[0][0], "red", label="CAVS")
# create_rmse_graph(df_F, "step", "rmse", axes[0][0], "blue", label="FLAT")


# sinとlinearの比較
df_C = CAVS_ruler_sin
df_F = CAVS_ruler_linear
create_error_bar_graph(df_C, "step", "open velocity [mm/s]", "max_angular_velocity", "max angular velocity", axes[0][1], "red", label="vibration")
create_error_bar_graph(df_F, "step", "open velocity [mm/s]", "max_angular_velocity", "max angular velocity", axes[0][1], "blue", label="non vibration")
create_rmse_graph2(df_C, "step", "open velocity [mm/s]", "last_angle", "RMSE of last angle", axes[1][0], "red", label="vibration")
create_rmse_graph2(df_F, "step", "open velocity [mm/s]", "last_angle", "RMSE of last angle", axes[1][0], "blue", label="non vibration")
create_error_bar_graph(df_C, "step", "open velocity [mm/s]", "elasped_time", "elasped time", axes[1][1], "red", label="vibration")
create_error_bar_graph(df_F, "step", "open velocity [mm/s]", "elasped_time", "elasped time", axes[1][1], "blue", label="non vibration")
create_rmse_graph(df_C, "step", "open velocity [mm/s]", "rmse", "RMSE of angular velocity", axes[0][0], "red", label="vibration")
create_rmse_graph(df_F, "step", "open velocity [mm/s]", "rmse", "RMSE of angular velocity", axes[0][0], "blue", label="non vibration")

# 凡例表示
for ax1 in axes:
    for ax in ax1:
        ax.legend()

# create_rmse_graph(df_CAVS, "step", ax, "red")
# create_rmse_graph(df_FLAT, "step", ax, "blue")

plt.savefig("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0705/CAVS_sin_vs_linear.png")
plt.savefig("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0705/CAVS_sin_vs_linear.eps")
plt.show()

# plt.scatter(x=df_FLAT["step"], y=df_FLAT["error"]/60, color="blue", label="FLAT")
# plt.scatter(x=df_CAVS["step"], y=df_CAVS["error"]/60, color="red", label="CAVS")

# plt.ylim=([0,20000])
# plt.xlim=([0.0001, 0.01])

# plt.yticks(np.arange(0,20001, 5000))

# plt.ylabel("error", fontsize=18)
# plt.xlabel("open velocity [mm/s]", fontsize=18)

# plt.legend(bbox_to_anchor=(1, 0), loc='lower right', borderaxespad=1, fontsize=18)

# # plt.savefig("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/result2.png")
# plt.show()