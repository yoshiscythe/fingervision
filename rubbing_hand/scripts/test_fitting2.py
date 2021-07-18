#!/usr/bin/python
#coding: utf-8

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator
import pylab
from scipy.optimize import curve_fit
# https://teratail.com/questions/213418


df = pd.read_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0717/dt/rosbag/dt00_2021-07-17-15-18-18.csv")

df_rotate = df[634:642]
# df_rotate = df[635:639]
df_wide = df[634:651]

y = df_rotate["angle"].values.tolist()
y_wide = df_wide["angle"].values.tolist()
print(y)

# x_array = [-len(y_array), -len(y_array)+1, ... , 0]
x = np.flip(np.arange(len(y))*-1)
x_over = np.append(x, np.arange(1,10))

#近似式の係数
res1=np.polyfit(x, y, 1)
res2=np.polyfit(x, y, 2)
res3=np.polyfit(x, y, 3)
#近似式の計算
y1 = np.poly1d(res1)(x_over) #1次
y2 = np.poly1d(res2)(x_over) #2次
y3 = np.poly1d(res3)(x_over) #3次

p = np.poly1d(res2)
y_trg = 60
x_trg = max((p - y_trg).roots)
if isinstance(x_trg, complex):
    print("complex")
print(y_trg, x_trg/60)


plt.scatter(x_over, y_wide, label='wide')
plt.scatter(x, y, label='org data')
plt.plot(x_over, y1, label='1st order')
plt.plot(x_over, y2, label='2nd order')
plt.scatter(x_trg, y_trg, label='target')
# plt.plot(x_over, y3, label='3rd order')
plt.vlines(6,0,160)
plt.legend()
plt.show()