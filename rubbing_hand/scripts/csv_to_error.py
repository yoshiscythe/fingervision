#!/usr/bin/python
#coding: utf-8
# 参考
# https://answers.ros.org/question/55037/is-there-a-way-to-save-all-rosbag-data-into-a-csv-or-text-file/

import sys
sys.path.append("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/src")
import rosbag
from rubbing_hand.msg import inhand
import pandas as pd
from datetime import datetime
import os
import matplotlib.pyplot as plt

data_directory = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0510"

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

def calcurate_error(df):
    error = 0
    # for omega in df["angular velocity"]:
    #     omega_d = omega - 10
    #     if omega_d > 0:
    #         error+= omega_d**2
    last_angle = sum(df['angle'].tail(60))/60
    max_vel = max(df['angular velocity'])
    df = df["angular velocity"]-10
    df = df[df>0]
    df = df**2
    error = sum(df)

    return error, last_angle, max_vel


id_file_name = data_directory+"/matome_data_include_id.csv"
df_id = pd.read_csv(id_file_name)
df_id_FLAT = df_id.loc[:, ["FLAT id", "FLAT step", "FLAT frequency"]]
df_id_FLAT = df_id_FLAT.set_index("FLAT id")
df_id_FLAT = df_id_FLAT.dropna()
df_id_CAVS = df_id.loc[:, ["CAVS id", "CAVS step", "CAVS frequency"]]
df_id_CAVS = df_id_CAVS.set_index("CAVS id")
df_id_CAVS = df_id_CAVS.dropna()
print("read csv")

# print(df_id_FLAT)

csv_CAVS = create_csv_dict(data_directory+"/CAVS/rosbag")
csv_FLAT = create_csv_dict(data_directory+"/FLAT/rosbag")
print("created csv_dict")

column_names = ["id", "step", "frequency", "error", "last angle"]
df_error_CAVS = pd.DataFrame(columns=column_names)
df_error_FLAT = pd.DataFrame(columns=column_names)

progress = 0
for id in df_id_FLAT.index.values.tolist():
    id = str(int(id)).rjust(2, "0")
    error, last_angle, max_vel = calcurate_error(pd.read_csv(csv_FLAT[id]))
    step = df_id_FLAT["FLAT step"][int(id)]
    frequency = df_id_FLAT["FLAT frequency"][int(id)]
    df_error_FLAT = df_error_FLAT.append(
                        {'id': id,
                        'step': step,
                        'frequency': frequency,
                        'error': error,
                        'last angle': last_angle,
                        'max angular velocity': max_vel},
                        ignore_index=True
                    )
    progress += 1
    print("FLAT: ", progress, "/", len(df_id_FLAT.index.values.tolist()), ", error: ", error)

progress = 0
for id in df_id_CAVS.index.values.tolist():
    id = str(int(id)).rjust(2, "0")
    error, last_angle, max_vel = calcurate_error(pd.read_csv(csv_CAVS[id]))
    step = df_id_CAVS["CAVS step"][int(id)]
    frequency = df_id_CAVS["CAVS frequency"][int(id)]
    df_error_CAVS = df_error_CAVS.append(
                        {'id': id,
                        'step': step,
                        'frequency': frequency,
                        'error': error,
                        'last angle': last_angle,
                        'max angular velocity': max_vel},
                        ignore_index=True
                    )
    progress += 1
    print("CAVS: ", progress, "/", len(df_id_CAVS.index.values.tolist()), ", error: ", error)

df_error_CAVS.to_csv(data_directory+"/CAVS_error.csv")
df_error_FLAT.to_csv(data_directory+"/FLAT_error.csv")

df_error_CAVS.plot.scatter(x="frequency", y="error")
df_error_FLAT.plot.scatter(x="frequency", y="error")

plt.show()