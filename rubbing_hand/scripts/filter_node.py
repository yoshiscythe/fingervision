#!/usr/bin/python
#coding: utf-8

import numpy as np
import matplotlib.pyplot as plt
import time
from scipy import signal

import rospy
import roslib; roslib.load_manifest('rubbing_hand')
from rubbing_hand.msg import Float64Array
from fingervision_msgs.msg import Filter1ObjInfo

class Log():
    def __init__(self, length = 101):
        self.log = []
        self.length = length
        self.need_modify = False

    def storing(self, data):
        if self.need_modify:
            k = 0.7267534839
            data = np.degrees(np.arctan(k*np.tan(data)))
        self.log.append(data)
        if len(self.log)>self.length: self.log.pop(0)
    
    def get_log(self):
        return self.log

class LKF():
    def __init__(self, pub, T = 100):
        self.T = T
        self.log = {}
        self.log["obj_orientation"] = Log(T+1)
        self.log["obj_orientation"].need_modify = True
        self.log["d_obj_orientation"] = Log(T+1)
        self.log["d_obj_orientation"].need_modify = True
        self.pub = pub

        # 状態方程式
        # x = A * x_ + B * u + w, w ~ N(0,Q)
        self.A = np.mat([[1,0.033], [0,1]])
        self.B = np.mat([[1,0], [0,1]])
        self.Q = np.mat([[1.,0], [0,1.]])
        # 観測方程式
        # y = C * x + v, v ~ N(0,R)
        self.C = np.mat([[1,0],[0,1]])
        self.R = np.mat([[2.,0],[0,2.]])
        
        self.mu0 = np.mat([[0],[0]]) # 初期状態推定値
        self.Sigma0 = np.mat([[0,0],[0,0]]) # 初期誤差共分散行列

    # 参考
    # https://satomacoto.blogspot.com/2011/06/python.html
    def lkf(self, T, Y, U):
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
        mu0 = self.mu0
        Sigma0 = self.Sigma0
        A = self.A
        B = self.B
        C = self.C
        Q = self.Q
        R = self.R

        mu = mu0 # 初期状態推定値
        Sigma = Sigma0 # 初期誤差共分散行列

        M = [mu] # 状態推定値列

        for i in range(T):
            # 推定
            mu_ = A * mu + B * U[i]
            Sigma_ = Q + A * Sigma * A.T

            # 更新
            yi = Y[i+1] - C * mu_
            S = C * Sigma_ * C.T + R
            K = Sigma_ * C.T * S.I
            mu = mu_ + K * yi
            Sigma = Sigma_ - K * C * Sigma_
            M.append(mu)

        return M

    def callback(self, msg):
        self.log["obj_orientation"].storing(msg.obj_orientation)
        self.log["d_obj_orientation"].storing(-msg.d_obj_orientation_filtered)

        T = self.T
        y = np.array([self.log["obj_orientation"].get_log(),self.log["d_obj_orientation"].get_log()]).T
        Y = [np.mat(y.reshape(2,1)) for y in y]
        u = np.mat([[0],[0]]) # 入力（一定）
        U = [u for i in range(T+1)]

        if len(Y) < T+1:
            return

        M = self.lkf(T, Y, U)

        data = Float64Array()
        data.data = M[-1]
        self.pub.publish(data)

class LKF2():
    def __init__(self, pub, T = 100):
        self.T = T
        self.log = {}
        self.log["obj_orientation"] = Log(T+1)
        self.log["obj_orientation"].need_modify = True
        self.pub = pub

        # 状態方程式
        # x = A * x_ + B * u + w, w ~ N(0,Q)
        self.A = np.mat([[1,0.0333], [0,1]])
        self.B = np.mat([[1,0], [0,1]])
        self.Q = np.mat([[1.,0], [0,1.]])
        # 観測方程式
        # y = C * x + v, v ~ N(0,R)
        self.C = np.mat([[1,0]])
        self.R = np.mat([[1.]])
        
        self.mu0 = np.mat([[0],[0]]) # 初期状態推定値
        self.Sigma0 = np.mat([[0,0],[0,0]]) # 初期誤差共分散行列

    # 参考
    # https://satomacoto.blogspot.com/2011/06/python.html
    def lkf(self, T, Y, U):
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
        mu0 = self.mu0
        Sigma0 = self.Sigma0
        A = self.A
        B = self.B
        C = self.C
        Q = self.Q
        R = self.R

        mu = mu0 # 初期状態推定値
        Sigma = Sigma0 # 初期誤差共分散行列

        M = [mu] # 状態推定値列

        for i in range(T):
            # 推定
            mu_ = A * mu + B * U[i]
            Sigma_ = Q + A * Sigma * A.T

            # 更新
            yi = Y[i+1] - C * mu_
            S = C * Sigma_ * C.T + R
            K = Sigma_ * C.T * S.I
            mu = mu_ + K * yi
            Sigma = Sigma_ - K * C * Sigma_
            M.append(mu)

        # print(Sigma)

        return M

    def callback(self, msg):
        self.log["obj_orientation"].storing(msg.obj_orientation)
        # self.log["d_obj_orientation"].storing(-msg.d_obj_orientation_filtered)

        T = self.T
        Y = self.log["obj_orientation"].get_log()
        u = np.mat([[0],[0]]) # 入力（一定）
        U = [u for i in range(T+1)]

        if len(Y) < T+1:
            return

        M = self.lkf(T, Y, U)

        data = Float64Array()
        data.data = list(M[-1])
        data.data.append(Y[-1])
        self.pub.publish(data)

class LKF3():
    def __init__(self, pub):
        self.pub = pub

        # 状態方程式
        # x = A * x_ + B * u + w, w ~ N(0,Q)
        self.A = np.mat([[1,0.0333], [0,1]])
        self.B = np.mat([[1,0], [0,1]])
        self.Q = np.mat([[1.,0], [0,1.]])
        # 観測方程式
        # y = C * x + v, v ~ N(0,R)
        self.C = np.mat([[1,0]])
        self.R = np.mat([[1.]])
        
        self.mu = np.mat([[0],[0]]) # 初期状態推定値
        self.Sigma = np.mat([[0,0],[0,0]]) # 初期誤差共分散行列

    # 参考
    # https://satomacoto.blogspot.com/2011/06/python.html
    def lkf(self, Y, U):
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
        A = self.A
        B = self.B
        C = self.C
        Q = self.Q
        R = self.R

        # 推定
        mu_ = A * self.mu + B * U
        Sigma_ = Q + A * self.Sigma * A.T

        # 更新
        yi = Y - C * mu_
        S = C * Sigma_ * C.T + R
        K = Sigma_ * C.T * S.I
        self.mu = mu_ + K * yi
        self.Sigma = Sigma_ - K * C * Sigma_

        return self.mu

    def callback(self, msg):
        Y = msg.obj_orientation
        k = 0.7267534839
        Y = np.degrees(np.arctan(k*np.tan(Y)))
        U = np.mat([[0],[0]]) # 入力（一定）

        M = self.lkf(Y, U)

        data = Float64Array()
        data.data = list(M)
        data.data.append(Y)
        self.pub.publish(data)

class SGF:
    log = {}
    def __init__(self, pub):
        self.log["obj_orientation"] = Log()
        self.log["obj_orientation"].need_modify = True
        self.pub = pub

    def callback(self, msg):
        self.log["obj_orientation"].storing(msg.obj_orientation)

        len_window = 99
        y = self.log["obj_orientation"].get_log()
        dim_approx = 5

        if len(y) < 101:
            return

        theta = signal.savgol_filter(y, len_window, dim_approx, deriv=0, mode="interp")[-1]
        theta_dot = signal.savgol_filter(y, len_window, dim_approx, deriv=1, mode="interp")[-1]
        theta_dot = theta_dot/0.033

        data = Float64Array()
        data.data = [theta, theta_dot]
        self.pub.publish(data)

class SMAF:
    log = {}
    def __init__(self, pub):
        self.log["obj_orientation"] = Log()
        self.log["obj_orientation"].need_modify = True
        self.log["d_obj_orientation"] = Log(10)
        self.pub = pub

    def callback(self, msg):
        self.log["obj_orientation"].storing(msg.obj_orientation)

        len_window = 10
        y = self.log["obj_orientation"].get_log()

        if len(y) < len_window+1:
            return

        theta = np.mean(y[-len_window:])
        self.log["d_obj_orientation"].storing(y[-1]-y[-len_window-1])
        theta_dot = np.mean(self.log["d_obj_orientation"].get_log())/(0.033*len_window)

        data = Float64Array()
        data.data = [theta, theta_dot]
        self.pub.publish(data)

def callback(msg, func):
    times = []
    times.append(time.time())
    for i in range(len(func)):
        func[i].callback(msg)
        times.append(time.time())

    # for i in range(len(times)-1):
    #     elapsed_time = times[i+1] - times[i]
    #     print("func",i+1,":",elapsed_time)
    # print("all:",times[-1]-times[0])

if __name__=='__main__':
    rospy.init_node('filter_node')

    obj_orientation_lkf_pub1 = rospy.Publisher(rospy.get_namespace()+"obj_orientation_lkf1", Float64Array, queue_size=1)
    obj_orientation_lkf_pub2 = rospy.Publisher(rospy.get_namespace()+"obj_orientation_lkf2", Float64Array, queue_size=1)
    # obj_orientation_sgf_pub = rospy.Publisher("obj_orientation_sgf", Float64Array, queue_size=10)
    obj_orientation_smaf_pub = rospy.Publisher("obj_orientation_smaf", Float64Array, queue_size=10)
    my_LKF1 = LKF2(obj_orientation_lkf_pub1, 20)
    my_LKF2 = LKF3(obj_orientation_lkf_pub2)
    # my_LKF1.A = np.mat([[1,0.033], [0,1]])
    # my_LKF2.A = np.mat([[1,0.033], [0,1]])
    # my_LKF1.Q = np.mat([[0.647288,0], [0,0.647288]])
    # my_LKF1.R = np.mat([[0.355608]])
    my_LKF1.Q *= 0.64
    my_LKF1.R *= 0.35
    my_LKF2.Q *= 0.64
    my_LKF2.R *= 0.35
    # my_LKF2.Sigma0 = np.mat([[2,0],[0,2]])
    # my_LKF1.A = np.mat([[1,0.00333], [0,1]])

    # my_SGF = SGF(obj_orientation_sgf_pub)
    my_SMAF = SMAF(obj_orientation_smaf_pub)

    rospy.Subscriber("/fingervision/fv_filter1_objinfo", Filter1ObjInfo, lambda msg: callback(msg,[my_LKF1, my_LKF2, my_SMAF]), queue_size=1)
    # rospy.Subscriber("/fingervision/fv_filter1_objinfo", Filter1ObjInfo, my_LKF1.callback)

    print("start kalman_filter")

    rospy.spin()
