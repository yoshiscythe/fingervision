#!/usr/bin/python
#coding: utf-8

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator
import pylab
from scipy.optimize import curve_fit
# https://teratail.com/questions/213418


df = pd.read_csv("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0705/CAVS/rosbag/CAVS25_2021-07-05-03-58-58.csv")

df_rotate = df[1352:1400]
df_wide = df[1300:1420]

i=20
df_rotate1, df_rotate2 = df_rotate[0:i],df_rotate[i:-1]


# plt.scatter(x=df["time"], y=df["angle"], color="red", label="org")
# plt.scatter(x=df_rotate["time"], y=df_rotate["angle"], color="blue", label="org")

# plt.ylim=([0,20000])
# plt.xlim=([0.0001, 0.01])

# plt.yticks(np.arange(0,20001, 5000))


def sigmoid(x, x0, k, a, c):
     y = a / (1 + np.exp(-k*(x-x0))) + c
     return y
def func_poly2d(x, a, b, c):
    return a*x**2 + b*x + c
def func_poly2d_zero(x, a, b, c):
    return a*x**2 - 2*a*b*x + a*b**2 + c
def func_poly2d_limited(x, a, x0, y0, y1):
    b = lambda a: y1 - 2*x0*a
    c = lambda a,b: y0 - a*x0**2 - b*x0
    return a*x**2 + b(a)*x + c(a, b(a))
def three_point(func,w,e):
    E_dashth = (func(w+e)-func(w-e)) / (2*e)
    return E_dashth

# func=func_poly2d
func = func_poly2d_zero

xdata0 = df_rotate["time"].values
ydata0 = df_rotate["angle"].values

# init_params = np.array([1, 1, 1, 0])
init_params = np.array([1, 1, 1])
xlim = (min(xdata0) - (max(xdata0)-min(xdata0))/4.0, max(xdata0)+(max(xdata0)-min(xdata0))/4.0)
ylim = (min(ydata0) - (max(ydata0)-min(ydata0))/4.0, max(ydata0)+(max(ydata0)-min(ydata0))/4.0)
sigmoid_format = r'$y = \frac{%f}{1 + e^{-%f (x - %f)}} + %f$'
poly2d_zero_format = r'$y = %f(x - %f)^2 + %f$'

xdata = df_rotate1["time"].values
xdata1 = xdata
ydata = df_rotate1["angle"].values
ydata1 = ydata

sigma= np.ones(len(xdata))
# sigma[[0]]= 0.01 
init_params = np.array([1,xdata[0], ydata[0]])
popt1, pcov1 = curve_fit(lambda x, a, b, c: func(x, a, xdata[0], ydata[0]), xdata, ydata, init_params, sigma=sigma)
# print("Result : x0=%f, k=%f, a=%f, c=%f" % (popt[0], popt[1], popt[2], popt[3]))
# print("Result : a=%f, b=%f, c=%f" % (popt[0], popt[1], popt[2]))

xdata = df_rotate2["time"].values
ydata = df_rotate2["angle"].values

sigma= np.ones(len(xdata))
# sigma[[-1]]= 0.01 
init_params2 = np.array([-1])
x0 = xdata[0]
y0 = func(x0, *popt1)
y1 = three_point(lambda x: func(x, *popt1), x0, 0.01)
# print(y1)
popt2, pcov2 = curve_fit(lambda x, a: func_poly2d_limited(x, a, x0, y0, y1), xdata, ydata, init_params2, sigma=sigma, bounds=(-np.inf, 0))
print(popt2)
# print("Result : x0=%f, k=%f, a=%f, c=%f" % (popt[0], popt[1], popt[2], popt[3]))
# print("Result : a=%f, b=%f, c=%f" % (popt[0], popt[1], popt[2]))

x = np.linspace(xlim[0], xlim[1], 200)
x1 = np.linspace(xlim[0], xdata1[-1], 200)
x2 = np.linspace(xdata1[-1], xlim[1], 200)
yinit = func(x, *init_params)
yopt1 = func(x1, *popt1)
yopt2 = func_poly2d_limited(x2, popt2[0], x0, y0, y1)

pylab.plot(df_wide["time"], df_wide["angle"], 'o', label='all')
pylab.plot(xdata0, ydata0, 'o', label='Target')
# pylab.plot(x, yinit, label=('Pre-Opt : ' + (sigmoid_format % (init_params[2], init_params[1], init_params[0], init_params[3]))) )
# pylab.plot(x, yopt, label=('Optimized : ' + (sigmoid_format % (popt[2], popt[1], popt[0], popt[3]))) )
# pylab.plot(x, yinit, label=('Pre-Opt : ' + (poly2d_zero_format % (init_params[0], init_params[1], init_params[2]))) )
pylab.plot(x1, yopt1, label=('Optimized : ' + (poly2d_zero_format % (popt1[0], popt1[1], popt1[2]))) )
pylab.plot(x2, yopt2, label=('Optimized : ') )
pylab.ylim(ylim[0], ylim[1])
pylab.legend(loc='upper left')
pylab.grid(True)
pylab.show()

y2 = three_point(lambda x: func_poly2d_limited(x, popt2[0], x0, y0, y1), x0, 0.01)
x1 = func(x0, *popt1)
x2 = func_poly2d_limited(x0, popt2[0], x0, y0, y1)
print(x1, y1,x2, y2)

# plt.ylabel("angle [deg]", fontsize=18)
# plt.xlabel("time [s]", fontsize=18)

# plt.legend(bbox_to_anchor=(1, 0), loc='lower right', borderaxespad=1, fontsize=18)

# # plt.savefig("/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0223/result2.png")
# plt.show()