#!/usr/bin/python
#coding: utf-8

import csv
import glob
import os
import re
import pandas as pd

# 重力加速度[m/s^2]
g_acc = 9.80665
data_directory = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/1212/ELF/"

# data_directory内のcsvファイルのパスをリストで取得
def create_ls(data_directory):
    ls = []
    file_name = data_directory+"*.csv"
    ls += glob.glob(file_name)
    # 重複削除
    ls = list(set(ls))
    # ソート
    ls = sorted(ls)

    return ls

ls = create_ls(data_directory)

for l in ls:
    # 1行目は単位系が書いてあるので，スキップ
    # ヘッダーずれてるので直す
    # 元のcsvファイルでヘッダー末尾にカンマがないことが原因ぽい．ELFふざけるな
    df = pd.read_csv(l,skiprows=1)
    df = df.rename_axis('index').reset_index()
    df = df.dropna(how="all", axis=1)
    df = df.set_axis(['seconds', 'force'], axis='columns')

    # gf->N変換
    df["force"] = df["force"]*g_acc/1000

    # 保存ファイル名作成
    base, ext = os.path.splitext(l)
    # base = /home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/1212/ELF/20211212_A_CAVS01  を次のように分ける
    # base1 = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/1212/ELF/"
    # base2 = "20211212_A_CAVS01"
    m=re.search(r"(.*/)(.*)$", base)
    if m is None:
        print('Not match')
        base1= None
        base2 = None
    else:
        base1 = m.group(1)
        base2 = m.group(2)
    save_file = base1+"modified/"+base2+"_mod.csv"

    df.to_csv(save_file)
