#!/usr/bin/python
#coding: utf-8

import numpy as np
import threading
import rospy
from rub import PID

import roslib
roslib.load_manifest('rubbing_hand')
from rubbing_hand.srv import *
from rubbing_hand.msg import *

class Inhand:
    def __init__(self, rubbing, sub_fv_filtered1_objinfo, sub_fv_lkf):
        self.is_running= False
        self.rubbing = rubbing
        self.sub_fv_filtered1_objinfo = sub_fv_filtered1_objinfo
        self.sub_fv_lkf = sub_fv_lkf

        #get the angle of object
        self.get_theta = lambda: self.sub_fv_lkf.req.data[0]
        #get the angle velocity of object
        self.get_omega = lambda: self.sub_fv_lkf.req.data[1]
        #get the slip of object
        self.get_slip = lambda: sum(self.sub_fv_filtered1_objinfo.req.mv_s)

        self.target_angle = 40.
        self.min_gstep = 0.01
        self.th_slip = 0.0001
        self.target_omega = -2

        self.hz = 60
        # self.tmp_pub = rospy.Publisher(rospy.get_namespace()+"tmp", Float64, queue_size=1)

    def Start(self):
        self.Stop()
        self.thread= threading.Thread(name='inhand', target=self.Maniloop)
        self.is_running= True
        self.thread.start()

    def Stop(self):
        if self.is_running:
            self.is_running= False
            # self.thread.join()

    # 初速度vel_0から最高速度vel_tまで時間timeで一定加速度で変化する速度軌跡を生成,末尾−1
    def Create_trapezoidal_array(self, vel_t, time, vel_0 = 0):
        acc = float(vel_t-vel_0) / time
        array = [t*acc + vel_0 for t in range(time)]
        # # 番兵
        # array.append(-1)

        return array

    def sgn_(self, d_omega):
        max_=0.005
        min_=-0.0001
        d_pos = -d_omega*0.001
        if d_pos>max_:
            d_pos = max_
        elif d_pos<min_:
            d_pos = min_
        return d_pos

    def Maniloop(self):
        #get the initial angle
        theta0 = self.get_theta()
        thread_cond = lambda: self.is_running and not rospy.is_shutdown()
        r = rospy.Rate(self.hz)

        print("start inhand manipulation")

        #Open gripper until slip is detected
        self.rubbing.Go2itv(50, 0.01)
        while thread_cond():
            print(self.get_slip())
            if self.get_slip() > self.th_slip:
                print("open", self.get_theta(), self.get_slip())
                break
            r.sleep()

        #Control the velocity angle of obj
        g_pos= self.rubbing.interval
        # self.rubbing.Set_interval(g_pos-1)

        # v_array = self.Create_trapezoidal_array(self.target_omega, 120)
        while thread_cond():
            if abs(theta0-self.get_theta())>self.target_angle:
                print("Done!")
                break
            # if v_array:
            #     omega_trg = v_array.pop(0)
            # else:
            #     omega_trg = self.target_omega
            omega_trg = self.target_omega
            omega = self.get_omega()
            omega_d = omega_trg - omega
            d_pos = self.sgn_(omega_d)
            g_pos += d_pos
            print(omega_trg, omega, d_pos, g_pos)
            # data = Float64()
            # data.data = omega_trg
            # self.tmp_pub.publish(data)
            self.rubbing.Set_interval(g_pos)
            r.sleep()

        
        print("angle=", self.get_theta(), ", target=", self.target_angle, ", diff=", theta0-self.get_theta())
        self.rubbing.Set_interval(18)

        self.Stop()

def modify_rad_curve(theta):
    k = 0.7
    return np.arctan(k*np.tan(theta))

def Go2itv_client(data1, data2):
  rospy.wait_for_service('/Go2itv')
  try:
    Go2itv = rospy.ServiceProxy('/Go2itv', Set2Float64)
    Go2itv(data1, data2)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e