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

def create_pd_from_rosbag(rosbag_fullpath):
    bag = rosbag.Bag(rosbag_fullpath)
    topic = '/inhand'
    column_names = ['time', 'angle', "angular velocity", "gripper position", "manipulated variable"]
    df = pd.DataFrame(columns=column_names)
    rotation_f=False
    finish_f = False
    finish_over = 0

    for topic, msg, t in bag.read_messages(topics=topic):
        time = msg.header.stamp.to_sec()
        angle = msg.obj_orientation_filtered
        angular_velocity = msg.d_obj_orientation_filtered
        gripper_position = msg.interval
        manipulated_variable = msg.MV

        if (not rotation_f) and angle>5:
            rotation_f=True
            start_time = time

        if rotation_f:
            if angle > 60 and not finish_f:
                finish_f = True
            if finish_f:
                if finish_over<60:
                    finish_over +=1
                else: break
            df = df.append(
                {'time': time-start_time,
                'angle': angle,
                'angular velocity': angular_velocity,
                "gripper position": gripper_position,
                "manipulated variable": manipulated_variable},
                ignore_index=True
            )

    return df

# def calculate_error(df):


def create_rosbag_dict(path):
    rosbag_file = {}
    for f in os.listdir(path):
        base, ext = os.path.splitext(f)
        if ext == '.bag':
            id = base[4:6]
            if id in rosbag_file:
                print(id, " exists two!")

            rosbag_file[id]=os.path.join(path, f)
    return rosbag_file

def calcurate_error(df):
    error = 0
    # for omega in df["angular velocity"]:
    #     omega_d = omega - 10
    #     if omega_d > 0:
    #         error+= omega_d**2
    last_angle = sum(df['angle'].tail(60))/60
    df = df["angular velocity"]-10
    df = df[df>0]
    df = df**2
    error = sum(df)

    return error, last_angle


id_file_name = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/data.csv"
df_id = pd.read_csv(id_file_name)
df_id_FLAT = df_id.loc[:, ["FLAT id", "FLAT step"]]
df_id_FLAT = df_id_FLAT.set_index("FLAT id")
df_id_FLAT = df_id_FLAT.dropna()
df_id_CAVS = df_id.loc[:, ["CAVS id", "CAVS step"]]
df_id_CAVS = df_id_CAVS.set_index("CAVS id")
df_id_CAVS = df_id_CAVS.dropna()
print("read csv")

# print(df_id)

rosbag_CAVS = create_rosbag_dict("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/CAVS/rosbag")
rosbag_FLAT = create_rosbag_dict("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/FLAT/rosbag")
print("created rosbag_dict")

column_names = ["id", "step", "error", "last angle"]
df_error_CAVS = pd.DataFrame(columns=column_names)
df_error_FLAT = pd.DataFrame(columns=column_names)

progress = 0
for id in df_id_FLAT.index.values.tolist():
    id = str(int(id)).rjust(2, "0")
    error, last_angle = calcurate_error(create_pd_from_rosbag(rosbag_FLAT[id]))
    step = df_id_FLAT["FLAT step"][int(id)]
    df_error_FLAT = df_error_FLAT.append(
                        {'id': id,
                        'step': step,
                        'error': error,
                        'last angle': last_angle},
                        ignore_index=True
                    )
    progress += 1
    print("FLAT: ", progress, "/", len(df_id_FLAT.index.values.tolist()), ", error: ", error)

progress = 0
for id in df_id_CAVS.index.values.tolist():
    id = str(int(id)).rjust(2, "0")
    error, last_angle = calcurate_error(create_pd_from_rosbag(rosbag_CAVS[id]))
    step = df_id_CAVS["CAVS step"][int(id)]
    df_error_CAVS = df_error_CAVS.append(
                        {'id': id,
                        'step': step,
                        'error': error,
                        'last angle': last_angle},
                        ignore_index=True
                    )
    progress += 1
    print("CAVS: ", progress, "/", len(df_id_CAVS.index.values.tolist()), ", error: ", error)

df_error_CAVS.to_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/CAVS_error.csv")
df_error_FLAT.to_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/FLAT_error.csv")

df_error_CAVS.plot.scatter(x="step", y="error")
df_error_FLAT.plot.scatter(x="step", y="error")

plt.show()