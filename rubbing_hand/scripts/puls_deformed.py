#!/usr/bin/python
#coding: utf-8

# rub.py内のPulse_deformed関数を，プロットするプログラム

import numpy as np
import matplotlib.pyplot as plt

def Pulse_deformed(runvel = 0.01, A = 0.1, f = 5, num = 1, d_ratio = 0.5):
        A = A/2.
        itv_start = 20
        hz = 5000. # モータへの送信周波数，実測値だいたい50くらい
        itv_goal = itv_start + (hz*runvel/f)*num
        rad_from_t = lambda t: np.pi*f*(t%(1./f))/(1-d_ratio) if (t%(1./f))*f < 1- d_ratio else np.pi*f*(t%(1./f))/d_ratio + (2 - 1./d_ratio)*np.pi
        run_time = float(num)/f
        # print(A)
        time = np.arange(0, run_time, 1/hz)

        if itv_start > itv_goal:
            linear_sign = -1
        else:
            linear_sign = 1
        traj_linear = [runvel*hz*t for t in time]
        # len_traj_linear = len(traj_linear)/num
        # len_second_half = int(len_traj_linear * d_ratio)
        # len_first_half = int(len_traj_linear - len_second_half)
        # traj_sin_first = [i*rad_per_step for i in range(len_first_half)]
        # traj_sin_second = [i*rad_per_step for i in range(len_second_half)]
        traj_sin_angle = [rad_from_t(t) for t in time]
        traj_sin = -A*np.cos(traj_sin_angle) + A
        go2itv_array = traj_linear + traj_sin + itv_start
        go2itv_array= np.append(go2itv_array, [itv_goal])
        
        return go2itv_array

if __name__=='__main__':
    # ratio = [0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0.01]
    # pulses = [Pulse_deformed(0, 3, 5, 1, r) for r in ratio]
    hzs = [40, 20, 10, 5, 3, 2, 1]
    pulses = [Pulse_deformed(0, 3, h, 1, 0.1) for h in hzs]
    fig, ax = plt.subplots()
    for i in range(len(pulses)):
        p = pulses[i]
        ax.plot(p, '-', label=hzs[i])
    ax.legend(loc=0)
    plt.show()
    