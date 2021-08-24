#!/usr/bin/python
#coding: utf-8

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator
import seaborn as sns

df_CAVS = pd.read_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0718/CAVS_error.csv")
df_FLAT = pd.read_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0718/FLAT_error.csv")

def create_scatter_graph(df, x_label, x_label_display, y_label, y_label_display, ax, color, label):
    x = df[x_label].values.tolist()
    y = df[y_label].values.tolist()

    ax.scatter(x, y, color=color, label=label)
    ax.set_xlabel(x_label_display, fontsize=fontsize)
    ax.set_ylabel(y_label_display, fontsize=fontsize)

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
        # print(k)
        x.append(k)
        y_mean.append(np.mean(v))
        y_err.append(np.std(v))
    # print(x, y_mean, y_err)

    ax.errorbar(x, y_mean, yerr=y_err, capsize=4, fmt='o', ecolor=color, color=color, label=label)
    ax.set_xlabel(x_label_display, fontsize=fontsize)
    ax.set_ylabel(y_label_display, fontsize=fontsize)

def create_boxplot_graph(df, x_label, x_label_display, y_label, y_label_display, ax, color, label):
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
        # print(k)
        points = tuple(v)
        x.append(k)
        y.append(points)
    # print(x, y)

    ax.boxplot(y, whis="range", widths=0.3)
    ax.set_xticklabels(x)
    ax.set_xlabel(x_label_display, fontsize=fontsize)
    ax.set_ylabel(y_label_display, fontsize=fontsize)

# CAVSとFLATと横に並べて一緒に箱ひげ図つくる
# seaborn(sns)パッケージ利用
# 参考：https://www.it-swarm-ja.com/ja/python/seaborn%E3%81%A7%E8%A4%87%E6%95%B0%E3%81%AE%E7%AE%B1%E3%81%B2%E3%81%92%E5%9B%B3%E3%82%92%E3%83%97%E3%83%AD%E3%83%83%E3%83%88%E3%81%97%E3%81%BE%E3%81%99%E3%81%8B%EF%BC%9F/831754565/
def create_boxplot_graph2(dfs, x_label, x_label_display, y_label, y_label_display, ax, colors, labels):
    for (df, label) in zip(dfs, labels):
        # print(df, label)
        df["pad"] = label
    cdf = pd.concat(dfs)
    # mdf = pd.melt(cdf, id_vars=[x_label], var_name=[y_label])

    # print(cdf.head())

    sns.boxplot(x=x_label, y=y_label, hue="pad", data=cdf, ax=ax, whis="range", width=0.3)
    # ax.set_xticklabels(x)
    ax.set_xlabel(x_label_display, fontsize=fontsize)
    ax.set_ylabel(y_label_display, fontsize=fontsize)

def create_error_bar_bar(df, x_label, x_label_display, y_label, y_label_display, ax, color, label):
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
        # print(k)
        x.append(k)
        y_mean.append(np.mean(v))
        y_err.append(np.std(v))
    # print(x, y_mean, y_err)

    ax.bar(x, y_mean, yerr=y_err, capsize=4, ecolor=color, color=color, label=label)
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
        # print(k)
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
        # print(k)
        rmse = np.sqrt(np.square(v).mean(axis=0))
        x.append(k)
        y.append(rmse)
    print(x, y)

    ax.scatter(x, y, color=color, label=label, marker="*")
    ax.set_xlabel(x_label_display, fontsize=fontsize)
    ax.set_ylabel(y_label_display, fontsize=fontsize)

# x_labelカラムデータがlist[rms, rms_sum, rms_len]となっている
def create_rmse_graph3(df, x_label, x_label_display, y_label, y_label_display, ax, color, label):
    data_dict = {}
    for index, row in df.iterrows():
        row = row.to_dict()
        x_data = row[x_label]
        y_data = eval(row[y_label])
        rmse_sum = float(y_data[1])
        rmse_len = float(y_data[2])
        data = np.array([rmse_sum, rmse_len])
        # print(type(y_data), y_data, data)
        if not x_data in data_dict:
            data_dict[x_data] = data
        else:
            data_dict[x_data] += data
    y = []
    x = []
    for k, v in data_dict.items():
        # print(k)
        x.append(k)
        rmse = np.sqrt(v[0]/v[1]) if v[1] != 0 else 0
        y.append(rmse)
    ax.scatter(x, y, color=color, label=label)
    ax.set_xlabel(x_label_display, fontsize=fontsize)
    ax.set_ylabel(y_label_display, fontsize=fontsize)
    # ax.set_ylim(ymin=0)

fig, axes = plt.subplots(2, 2, figsize=(16, 12))

CAVS_ruler_sin = df_CAVS[(df_CAVS["tool"] == 0) & (df_CAVS["method"] == 0)]
CAVS_ruler_linear = df_CAVS[(df_CAVS["tool"] == 0) & (df_CAVS["method"] == 1)]
CAVS_wood_sin = df_CAVS[(df_CAVS["tool"] == 1) & (df_CAVS["method"] == 0)]
CAVS_wood_linear = df_CAVS[(df_CAVS["tool"] == 1) & (df_CAVS["method"] == 1)]

FLAT_ruler_sin = df_FLAT[(df_FLAT["tool"] == 0) & (df_FLAT["method"] == 0)]
FLAT_ruler_linear = df_FLAT[(df_FLAT["tool"] == 0) & (df_FLAT["method"] == 1)]
FLAT_wood_sin = df_FLAT[(df_FLAT["tool"] == 1) & (df_FLAT["method"] == 0)]
FLAT_wood_linear = df_FLAT[(df_FLAT["tool"] == 1) & (df_FLAT["method"] == 1)]

# 軸名のサイズ
fontsize=24
# 軸ラベルのサイズ
labelsize=16
# 凡例のサイズ
lfontsize=20

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
df_F = FLAT_ruler_sin


create_error_bar_graph(df_C, "step", "$v_{open}$ [mm/s]", "max_angular_velocity", "max angular velocity", axes[0][1], "red", label="CAVS")
create_error_bar_graph(df_F, "step", "$v_{open}$ [mm/s]", "max_angular_velocity", "max angular velocity", axes[0][1], "blue", label="FLAT")
create_error_bar_graph(df_C, "step", "$v_{open}$ [mm/s]", "last_angle_error", "RMSE of last angle", axes[1][0], "red", label="CAVS")
create_error_bar_graph(df_F, "step", "$v_{open}$ [mm/s]", "last_angle_error", "RMSE of last angle", axes[1][0], "blue", label="FLAT")
create_rmse_graph2(df_C, "step", "$v_{open}$ [mm/s]", "last_angle_error", "RMSE of last angle", axes[1][0], "red", label="CAVS")
create_rmse_graph2(df_F, "step", "$v_{open}$ [mm/s]", "last_angle_error", "RMSE of last angle", axes[1][0], "blue", label="FLAT")
create_error_bar_graph(df_C, "step", "$v_{open}$ [mm/s]", "elasped_time", "elapsed time", axes[1][1], "red", label="CAVS")
create_error_bar_graph(df_F, "step", "$v_{open}$ [mm/s]", "elasped_time", "elapsed time", axes[1][1], "blue", label="FLAT")
create_rmse_graph3(df_C, "step", "$v_{open}$ [mm/s]", "rms_th=15", "RMSE of angular velocity", axes[0][0], "red", label="CAVS")
create_rmse_graph3(df_F, "step", "$v_{open}$ [mm/s]", "rms_th=15", "RMSE of angular velocity", axes[0][0], "blue", label="FLAT")



# -------------------------------------------------------------------------------------------------------------------------------------
# -----------------------------------------[position at close] vs----------------------------------------------------------------------
# create_scatter_graph(df_C, "pos_when_close", "$p$ [mm]", "last_angle_error", "error of last angle", axes[1][0], "red", label="CAVS")
# create_scatter_graph(df_F, "pos_when_close", "$p$ [mm]", "last_angle_error", "error of last angle", axes[1][0], "blue", label="FLAT")
# create_scatter_graph(df_C, "pos_when_close", "$p$ [mm]", "max_angular_velocity", "max angular velocity", axes[0][1], "red", label="CAVS")
# create_scatter_graph(df_F, "pos_when_close", "$p$ [mm]", "max_angular_velocity", "max angular velocity", axes[0][1], "blue", label="FLAT")
# create_scatter_graph(df_C, "pos_when_close", "$p$ [mm]", "rms", "RMSE of angular velocity", axes[0][0], "red", label="CAVS")
# create_scatter_graph(df_F, "pos_when_close", "$p$ [mm]", "rms", "RMSE of angular velocity", axes[0][0], "blue", label="FLAT")
# create_scatter_graph(df_C, "pos_when_close", "$p$ [mm]", "elasped_time", "elapsed time", axes[1][1], "red", label="CAVS")
# create_scatter_graph(df_F, "pos_when_close", "$p$ [mm]", "elasped_time", "elapsed time", axes[1][1], "blue", label="FLAT")
# -------------------------------------------------------------------------------------------------------------------------------------


# # -------------------------------------------------------------------------------------------------------------------------------------
# # -----------------------------------------variety of th of rmse of angular velosity ------------------------------------------------------------------------------
# create_rmse_graph3(df_C, "step", "$v_{open}$ [mm/s]", "rms_th=15", "th=15", axes[0][0], "red", label="CAVS")
# create_rmse_graph3(df_F, "step", "$v_{open}$ [mm/s]", "rms_th=15", "th=15", axes[0][0], "blue", label="FLAT")
# create_rmse_graph3(df_C, "step", "$v_{open}$ [mm/s]", "rms_th=30", "th=30", axes[0][1], "red", label="CAVS")
# create_rmse_graph3(df_F, "step", "$v_{open}$ [mm/s]", "rms_th=30", "th=30", axes[0][1], "blue", label="FLAT")
# create_rmse_graph3(df_C, "step", "$v_{open}$ [mm/s]", "rms_th=60", "th=60", axes[1][0], "red", label="CAVS")
# create_rmse_graph3(df_F, "step", "$v_{open}$ [mm/s]", "rms_th=60", "th=60", axes[1][0], "blue", label="FLAT")
# create_rmse_graph3(df_C, "step", "$v_{open}$ [mm/s]", "rms_th=90", "th=90", axes[1][1], "red", label="CAVS")
# create_rmse_graph3(df_F, "step", "$v_{open}$ [mm/s]", "rms_th=90", "th=90", axes[1][1], "blue", label="FLAT")
# # -------------------------------------------------------------------------------------------------------------------------------------

# create_error_bar_graph(df_C, "step", "$v_{open}$ [mm/s]", "omega_when_close", "omega_when_close", axes[0][0], "red", label="CAVS")
# create_error_bar_graph(df_F, "step", "$v_{open}$ [mm/s]", "omega_when_close", "omega_when_close", axes[0][0], "blue", label="FLAT")
# create_boxplot_graph2([df_C, df_F], "step", "$v_{open}$ [mm/s]", "omega_when_close", "omega_when_close", axes[0][0], colors=["red", "blue"], labels=["CAVS", "FLAT"])
# create_rmse_graph3(df_C, "step", "$v_{open}$ [mm/s]", "final_rms", "final_rms", axes[0][1], "red", label="CAVS")
# create_rmse_graph3(df_F, "step", "$v_{open}$ [mm/s]", "final_rms", "final_rms", axes[0][1], "blue", label="FLAT")

# create_boxplot_graph2([df_C, df_F], "step", "$v_{open}$ [mm/s]", "last_angle_error", "error of last angle", axes[1][0], colors=["red", "blue"], labels=["CAVS", "FLAT"])
# create_boxplot_graph2([df_C, df_F], "step", "$v_{open}$ [mm/s]", "max_angular_velocity", "max angular velocity", axes[0][1], colors=["red", "blue"], labels=["CAVS", "FLAT"])
# create_boxplot_graph2([df_C, df_F], "step", "$v_{open}$ [mm/s]", "elasped_time", "elapsed time", axes[1][1], colors=["red", "blue"], labels=["CAVS", "FLAT"])

# axes[0][0].legend(fontsize=lfontsize)
# axes[0][0].set_ylim(ymin=0)
# axes[0][1].set_ylim(ymin=0)
# axes[1][1].set_ylim(ymin=0)

# # 凡例表示
# for ax1 in axes:
#     for ax in ax1:
#         ax.legend(fontsize=lfontsize)
#         ax.tick_params(labelsize=labelsize)

# create_rmse_graph(df_CAVS, "step", ax, "red")
# create_rmse_graph(df_FLAT, "step", ax, "blue")

# plt.savefig("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0718/CAVS_sin_vs_FLAT_sin_ruler_final.png")
# plt.savefig("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0718/CAVS_sin_vs_FLAT_sin_ruler_final.eps")
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