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

# The bag file should be in the same directory as your terminal
bag = rosbag.Bag('/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0221/CAVS/rosbag/CAVS34_2021-02-22-00-58-55.bag')
topic = '/inhand'
column_names = ['time', 'angle', "angular velocity", "gripper position", "manipulated variable"]
df = pd.DataFrame(columns=column_names)
rotation_f=False

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
        df = df.append(
            {'time': time-start_time,
            'angle': angle,
            'angular velocity': angular_velocity,
            "gripper position": gripper_position,
            "manipulated variable": manipulated_variable},
            ignore_index=True
        )

df.to_csv('/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0221/CAVS/rosbag/CAVS34_2021-02-22-00-58-55.csv')