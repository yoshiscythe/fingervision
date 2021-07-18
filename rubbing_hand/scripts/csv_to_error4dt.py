#!/usr/bin/python3
#coding: utf-8
# 参考
# https://answers.ros.org/question/55037/is-there-a-way-to-save-all-rosbag-data-into-a-csv-or-text-file/

import sys
sys.path.append("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/src")
# import rosbag
from rubbing_hand.msg import inhand
import pandas as pd
from datetime import datetime
import os
import matplotlib.pyplot as plt
import numpy as np
import re

data_directory = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0717"

# 入力  path: ディレクリパス
# 出力  csv_file: 入力ディレクトリ内のすべてのcsvファイル（IDが重複した場合は日時が遅い方を取得）の絶対パスをバリュー，IDをキーとした辞書型
# CAVS00_2021-05-10-20-04-57.csvのようなcsvファイル名を想定．
def create_csv_dict(path):
    csv_file = {}
    for f in os.listdir(path):
        base, ext = os.path.splitext(f)
        if ext == '.csv':
            id = re.match("^.*?(\d+).*", base).group(1)
            if id in csv_file:
                print(id, " exists two!")

            csv_file[id]=os.path.join(path, f)
    return csv_file

# 色々計算
def calcurate_error(df, id):
    error_dict = {}
    df_last = df[df["process"]==1]
    df_mani = df[df["gripper velocity"]>0]

    last_angle = df_last['angle'].mean()
    error_dict["last_angle"] = last_angle

    df_finish = df[df["angle"]>(last_angle-0.5)]
    t_1 = df_finish.iloc[0]["time"]
    error_dict["t_1"] = t_1

    max_vel = max(df['angular velocity'])
    error_dict["max_angular_velocity"] = max_vel

    t_0 = df_mani.iloc[-1]["time"]
    error_dict["t_0"] = t_0

    dt = t_1 - t_0
    error_dict["dt"] = dt

    index_list = list(df_mani.index)
    close_index = index_list[-1]+1
    theta_dot = df.iloc[close_index]["angular velocity"]
    theta = df.iloc[close_index]["angle"]
    error_dict["theta_dot"] = theta_dot
    error_dict["theta"] = theta

    rotate_index = return_rotate_index(df[:close_index+1], 10)
    df_rotate = df[close_index-rotate_index:close_index+1]
    df_wide = df[close_index-rotate_index:close_index+11]
    plot_kinji(df_rotate, df_wide, id)

    # fig, ax = plt.subplots()
    # df.plot.scatter(x="time", y="angle", ax=ax, color="r")
    # df_mani.plot.scatter(x="time", y="angle", ax=ax, color="b")
    # df_last.plot.scatter(x="time", y="angle", ax=ax, color="y")
    # df_finish.plot.scatter(x="time", y="angle", ax=ax, color="g")
    # plt.show()

    return error_dict

def return_rotate_index(df, th):
    y = []
    data_list = df["angular velocity"].values.tolist()
    for v in reversed(data_list):
        if v < th:
            break
        else:
            y.insert(0,v)
    print(y)
    return len(y)

def plot_kinji(df_rotate, df_wide, id):
    y = df_rotate["angle"].values.tolist()
    y_wide = df_wide["angle"].values.tolist()

    # x_array = [-len(y_array), -len(y_array)+1, ... , 0]
    x = np.flip(np.arange(len(y))*-1)
    x_over = np.append(x, np.arange(1,11))

    #近似式の係数
    res1=np.polyfit(x, y, 1)
    res2=np.polyfit(x, y, 2)
    res3=np.polyfit(x, y, 3)
    #近似式の計算
    y1 = np.poly1d(res1)(x_over) #1次
    y2 = np.poly1d(res2)(x_over) #2次
    y3 = np.poly1d(res3)(x_over) #3次

    plt.cla()
    plt.scatter(x_over, y_wide, label='wide')
    plt.scatter(x, y, label='org data')
    plt.plot(x_over, y1, label='1st order')
    plt.plot(x_over, y2, label='2nd order')
    # plt.plot(x_over, y3, label='3rd order')
    plt.vlines(6,0,160)
    plt.legend()
    plt.savefig(data_directory+"/dt/estimate2/dt"+id+"_estimate.png")
    # plt.show()

# カラム名の変更用の辞書を作成
# "CAVS id" → "id"のように変更
# rename_columns_dict = {元のカラム名: 新しいカラム名, ...}
def Generate_rename_columns_dict(org_columns):
    rename_columns_dict = {}
    for s in org_columns:
        rename_columns_dict[s]=s.split(" ")[1]
    return rename_columns_dict

# IDと各種情報を関連付けたファイル（手作業でつくってねハート）を読み込む
# カラム名は，CAVS id/CAVS step/.../FLAT id/FLAT step/...みたいな感じで，接頭語にCAVS/FLATつけて，半角スペース置いて，stepとかfrequencyとかの情報を書く
id_file_name = data_directory+"/matome_data_include_id.csv"
df_id = pd.read_csv(id_file_name)

# 接頭語に応じて，CAVSとFLATのデータを分ける
columns_list = df_id.columns.values
FLAT_columns = [s for s in columns_list if s.startswith("FLAT")]
if FLAT_columns:
    df_id_FLAT = df_id.loc[:, FLAT_columns]
    FLAT_rename_columns_dict = Generate_rename_columns_dict(FLAT_columns)
    df_id_FLAT = df_id_FLAT.rename(columns=FLAT_rename_columns_dict)
    df_id_FLAT = df_id_FLAT.set_index("id")
    df_id_FLAT = df_id_FLAT.dropna(subset=['step'])
CAVS_columns = [s for s in columns_list if s.startswith("CAVS")]
if CAVS_columns:
    df_id_CAVS = df_id.loc[:, CAVS_columns]
    CAVS_rename_columns_dict = Generate_rename_columns_dict(CAVS_columns)
    df_id_CAVS = df_id_CAVS.rename(columns=CAVS_rename_columns_dict)
    df_id_CAVS = df_id_CAVS.set_index("id")
    df_id_CAVS = df_id_CAVS.dropna(subset=['step'])
    # print(df_id_CAVS)

print("read csv")

# rosbag_to_csv_multi.pyとかでrosbagから作った実験データのcsvファイルと，IDを紐付けるディクショナリを作る．
# キーがIDで，バリューがcsvのパス
csv_CAVS = create_csv_dict(data_directory+"/dt/rosbag")
# csv_FLAT = create_csv_dict(data_directory+"/FLAT/rosbag")
print("created csv_dict")


# calcurate_error()でerrorとかを計算して，dfにまとめる
if "df_id_FLAT" in locals():
    progress = 0
    for index, row in df_id_FLAT.iterrows():
        id = str(int(index)).rjust(2, "0")
        result_dict={"id": id}
        result_dict.update(row.to_dict())
        error_dict = calcurate_error(pd.read_csv(csv_FLAT[id]), id)
        result_dict.update(error_dict)
        
        # 初回は，いい感じのカラム名がついたpdデータフレームを作成
        if progress == 0:
            FLAT_column_names = list(result_dict.keys())
            df_error_FLAT = pd.DataFrame(columns=FLAT_column_names)

        df_error_FLAT = df_error_FLAT.append(result_dict,ignore_index=True)
        progress += 1
        print("FLAT: ", progress, "/", len(df_id_FLAT.index.values.tolist()))

if "df_id_CAVS" in locals():
    progress = 0
    for index, row in df_id_CAVS.iterrows():
        id = str(int(index)).rjust(2, "0")
        result_dict={"id": id}
        result_dict.update(row.to_dict())
        error_dict = calcurate_error(pd.read_csv(csv_CAVS[id]), id)
        result_dict.update(error_dict)
        
        if progress == 0:
            CAVS_column_names = list(result_dict.keys())
            df_error_CAVS = pd.DataFrame(columns=CAVS_column_names)

        df_error_CAVS = df_error_CAVS.append(result_dict,ignore_index=True)
        progress += 1
        print("CAVS: ", progress, "/", len(df_id_CAVS.index.values.tolist()))

if "df_error_CAVS" in locals():
    df_error_CAVS.to_csv(data_directory+"/CAVS_error.csv", index = False)
if "df_error_FLAT" in locals():
    df_error_FLAT.to_csv(data_directory+"/FLAT_error.csv", index = False)

# df_error_CAVS.plot.scatter(x="theta_dot", y="dt")
# df_error_CAVS.plot.scatter(x="step", y="max_angular_velocity")
# df_error_CAVS.plot.scatter(x="step", y="mean")
# df_error_CAVS.plot.scatter(x="step", y="rms")
# df_error_FLAT.plot.scatter(x="step", y="rms")
# df_error_CAVS.plot.scatter(x="step", y="error")


fig, ax = plt.subplots()
dt05 = df_error_CAVS[df_error_CAVS["step"] == 0.5]
dt05.plot.scatter(x="theta_dot", y="dt", ax=ax, color="r")
dt20 = df_error_CAVS[df_error_CAVS["step"] == 2]
dt20.plot.scatter(x="theta_dot", y="dt", ax=ax, color="r")
# omega30 = df_error_CAVS[(df_error_CAVS["omegatrg"] == 80)]
# omega30.plot.scatter(x="theta_dot", y="dt", ax=ax, color="g")

# grid
major_ticks_top=np.linspace(0,0.2,5)
minor_ticks_top=np.linspace(0,0.2,25)

ax.set_yticks(major_ticks_top)
ax.set_yticks(minor_ticks_top,minor=True)
ax.grid(which="major",alpha=0.6)
ax.grid(which="minor",alpha=0.3)
ax.set_ylim(0.05,0.2)

plt.savefig(data_directory+"/dt_all.png")
plt.show()