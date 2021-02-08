#!/usr/bin/python
#coding: utf-8

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import fmin
import math

# 参考
# https://satomacoto.blogspot.com/2011/06/python.html
def lkf(T, Y, U, A, B, C, Q, R, mu0, Sigma0):
    '''Linear Kalman Filter
    
    - 状態方程式
        x = A * x_ + B * u + w, w ~ N(0,Q)
    - 観測方程式
        y = C * x + v, v ~ N(0,R)
    
    Parameters
    ==========
    - T : ステップ数
    - Y : 観測列
    - U : 入力列
    - mu0 : 初期状態推定値
    - Sigma0 : 初期誤差共分散行列
    - A, B, C, Q, R : カルマンフィルタの係数 
    
    Returns
    =======
    - M : 状態推定値列
    '''
    mu = mu0 # 初期状態推定値
    Sigma = Sigma0 # 初期誤差共分散行列

    M = [mu] # 状態推定値列
    F = [np.matrix([[1e-100]])]
    V = [np.matrix([[0]])]

    for i in range(T):
        # 推定
        mu_ = A * mu + B * U[i]
        Sigma_ = Q + A * Sigma * A.T

        # 更新
        yi = Y[i+1] - C * mu_
        V.append(yi)
        S = C * Sigma_ * C.T + R
        K = Sigma_ * C.T * S.I
        mu = mu_ + K * yi
        Sigma = Sigma_ - K * C * Sigma_
        F.append(S)
        M.append(mu)

    return M, F, V

def yudo(F, V):
    return 1./2*sum(np.log(F)+V**2/F)

def calcloglike(X):
    Q1, R1 = X

    # 状態方程式
    # x = A * x_ + B * u + w, w ~ N(0,Q)
    A = np.mat([[1,0.0333], [0,1]])
    B = np.mat([[1,0], [0,1]])
    Q = np.mat([[float(Q1),0], [0,float(Q1)]])
    # 観測方程式
    # y = C * x + v, v ~ N(0,R)
    C = np.mat([[1,0]])
    R = np.mat([[float(R1)]])
    
    mu0 = np.mat([[0],[0]]) # 初期状態推定値
    Sigma0 = np.mat([[0,0],[0,0]]) # 初期誤差共分散行列

    T = len(nama_data)-1
    Y = nama_data
    u = np.mat([[0],[0]]) # 入力（一定）
    U = [u for i in range(T+1)]

    M, F, V = lkf(T, Y, U, A, B, C, Q, R, mu0, Sigma0)

    M1 = np.array(M).transpose()[0][0]
    F1 = np.array(F).transpose()[0][0]
    V1 = np.array(V).transpose()[0][0]

    return yudo(F1,V1)

file_name = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0208/4lkf_orientation.csv"
nama_data = pd.read_csv(file_name)["field.data2"]
nama_data = nama_data.values.tolist()

# scipyのfmin
# https://qiita.com/fukuit/items/c7ae3f71898399530dc1
count = 0
plt.axis([0, 100, 0, 1000])
plt.ion()

def cbf(Xi):
    global count
    count += 1
    f = calcloglike(Xi)
    print('%d, %f, %f, %f' % (count, Xi[0], Xi[1], f))
    plt.scatter(count, f)
    plt.pause(0.001)

[xopt, fopt, iter, funcalls, warnflag, allvecs] = fmin(calcloglike, [1., 1.], callback=cbf, maxiter=400, maxfun=400)