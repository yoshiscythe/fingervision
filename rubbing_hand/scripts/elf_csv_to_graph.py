#!/usr/bin/python3
#coding: utf-8

import pandas as pd
from datetime import datetime
import os
import matplotlib.pyplot as plt
import numpy as np
import re
import glob

# 重力加速度[m/s^2]
g_acc = 9.80665

data_directory = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/1212/ELF/"

# elfのcsvをdfにしたものを辞書に登録．pad=(CAVS|FLAT),idも含む
def create_df(data_directory, ID=""):
    ls = []
    if isinstance(ID, list):
        for id in ID:
            file_name = data_directory+"*"+id+"*.csv"
            ls += glob.glob(file_name)
    else:
        file_name = data_directory+"*"+ID+"*.csv"
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

        # 1行目は単位系が書いてあるので，スキップ
        # ヘッダーずれてるので直す
        # 元のcsvファイルでヘッダー末尾にカンマがないことが原因ぽい．ELFふざけるな
        df = pd.read_csv(l,skiprows=1)
        df = df.rename_axis('index').reset_index()
        df = df.dropna(how="all", axis=1)
        df = df.set_axis(['seconds', 'force'], axis='columns')

        # gf->N変換
        df["force"] = df["force"]*g_acc/1000

        m=re.search(r"([A-Z]+)(\d+)$", base)
        if m is None:
            print('Not match')
            pad = None
            id_num = None
        else:
            pad = m.group(1)
            id_num = m.group(2)
        result_dict[base] = [df, base, ext, pad, id_num]
        # print(base+" is loaded.")

    return result_dict

# input: create_graphで作成した辞書 dict[base] = [df, base, ext, pad, id_num]
# output: df columns["pad", "id", "min_force"]
# 入力辞書の各dfのforceの最小値を記録
def calcurate_min(dict):
    column_names = ["pad", "id", "min_force"]
    df2 = pd.DataFrame(columns=column_names)
    for k, v in dict.items():
        df, base, ext, pad, id_num = v
        min_force = min(df["force"])
        df2 = df2.append(
                {"pad": pad,
                "id": id_num,
                "min_force": min_force},
                ignore_index=True
            )
    min_CAVS_df = df2[df2["pad"] == "CAVS"]["min_force"]
    min_FLAT_df = df2[df2["pad"] == "FLAT"]["min_force"]
    print(min_CAVS_df.mean(), min_CAVS_df.std(), min_FLAT_df.mean(), min_FLAT_df.std())

    x = np.array(["CAVS", "FLAT"])
    y = np.array([min_CAVS_df.mean(),min_FLAT_df.mean()])
    e = np.array([min_CAVS_df.std(), min_FLAT_df.std()])

    plt.errorbar(x, y, e, linestyle='None', marker='^')

    # plt.show()

    return df2

# input: create_graphで作成した辞書 result_dict[base] = [df, base, ext, pad, id_num]
# output: void
# 辞書内の各dfについて，横軸seconds，縦軸forceのグラフを作製．
def create_graph(dict):
    for value in dict.values():
        df, base, ext, pad, id_num = value
        df.plot(x="seconds", y='force', legend=False)

        plt.savefig(base+"_raw.eps")
        plt.savefig(base+"_raw.png")

        # plt.show()
        print(base+" is saved.")

        plt.cla()

result_dict = create_df(data_directory)

# create_graph(result_dict)

calcurate_min(result_dict)