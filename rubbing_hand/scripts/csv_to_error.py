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

data_directory = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0525"

# 入力  path: ディレクリパス
# 出力  csv_file: 入力ディレクトリ内のすべてのcsvファイル（IDが重複した場合は日時が遅い方を取得）の絶対パスをバリュー，IDをキーとした辞書型
# CAVS00_2021-05-10-20-04-57.csvのようなcsvファイル名を想定．
def create_csv_dict(path):
    csv_file = {}
    for f in os.listdir(path):
        base, ext = os.path.splitext(f)
        if ext == '.csv':
            id = base[4:6]
            if id in csv_file:
                print(id, " exists two!")

            csv_file[id]=os.path.join(path, f)
    return csv_file

# 色々計算
def calcurate_error(df):
    error_dict = {}
    df = df[df["gripper position"]>20]

    last_angle = sum(df['angle'].tail(60))/60
    error_dict["last_angle"] = last_angle

    max_vel = max(df['angular velocity'])
    error_dict["max_angular_velocity"] = max_vel

    df_error = df["angular velocity"]-15
    df_error = df_error[df_error>0]
    df_error = df_error**2
    error = sum(df_error)
    error_dict["error"] = error

    df_rms = (df["angular velocity"]-15)**2
    rms = np.sqrt(sum(df_rms)/len(df_rms))
    error_dict["rms"] = rms

    std = df["angular velocity"].std()
    error_dict["std"] = std

    mean = df["angular velocity"].mean()
    error_dict["mean"] = mean

    return error_dict

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
df_id_FLAT = df_id.loc[:, FLAT_columns]
FLAT_rename_columns_dict = Generate_rename_columns_dict(FLAT_columns)
df_id_FLAT = df_id_FLAT.rename(columns=FLAT_rename_columns_dict)
df_id_FLAT = df_id_FLAT.set_index("id")
df_id_FLAT = df_id_FLAT.dropna()
CAVS_columns = [s for s in columns_list if s.startswith("CAVS")]
df_id_CAVS = df_id.loc[:, CAVS_columns]
CAVS_rename_columns_dict = Generate_rename_columns_dict(CAVS_columns)
df_id_CAVS = df_id_CAVS.rename(columns=CAVS_rename_columns_dict)
df_id_CAVS = df_id_CAVS.set_index("id")
df_id_CAVS = df_id_CAVS.dropna()

print("read csv")

# rosbag_to_csv_multi.pyとかでrosbagから作った実験データのcsvファイルと，IDを紐付けるディクショナリを作る．
# キーがIDで，バリューがcsvのパス
csv_CAVS = create_csv_dict(data_directory+"/CAVS/rosbag")
csv_FLAT = create_csv_dict(data_directory+"/FLAT/rosbag")
print("created csv_dict")


# calcurate_error()でerrorとかを計算して，dfにまとめる
progress = 0
for index, row in df_id_FLAT.iterrows():
    id = str(int(index)).rjust(2, "0")
    result_dict={"id": id}
    result_dict.update(row.to_dict())
    error_dict = calcurate_error(pd.read_csv(csv_FLAT[id]))
    result_dict.update(error_dict)
    
    # 初回は，いい感じのカラム名がついたpdデータフレームを作成
    if progress == 0:
        FLAT_column_names = list(result_dict.keys())
        df_error_FLAT = pd.DataFrame(columns=FLAT_column_names)

    df_error_FLAT = df_error_FLAT.append(result_dict,ignore_index=True)
    progress += 1
    print("FLAT: ", progress, "/", len(df_id_FLAT.index.values.tolist()))

progress = 0
for index, row in df_id_CAVS.iterrows():
    id = str(int(index)).rjust(2, "0")
    result_dict={"id": id}
    result_dict.update(row.to_dict())
    error_dict = calcurate_error(pd.read_csv(csv_CAVS[id]))
    result_dict.update(error_dict)
    
    if progress == 0:
        CAVS_column_names = list(result_dict.keys())
        df_error_CAVS = pd.DataFrame(columns=CAVS_column_names)

    df_error_CAVS = df_error_CAVS.append(result_dict,ignore_index=True)
    progress += 1
    print("CAVS: ", progress, "/", len(df_id_CAVS.index.values.tolist()))

df_error_CAVS.to_csv(data_directory+"/CAVS_error.csv", index = False)
df_error_FLAT.to_csv(data_directory+"/FLAT_error.csv", index = False)

# df_error_CAVS.plot.scatter(x="ratio", y="std")
# df_error_CAVS.plot.scatter(x="ratio", y="max_angular_velocity")
# df_error_CAVS.plot.scatter(x="ratio", y="mean")
# df_error_CAVS.plot.scatter(x="ratio", y="rms")
# df_error_FLAT.plot.scatter(x="amp", y="error", )

plt.show()