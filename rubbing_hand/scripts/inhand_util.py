#!/usr/bin/python
#coding: utf-8

import numpy as np
import threading
import rospy
from rub import PID
import time

import roslib
roslib.load_manifest('rubbing_hand')
from rubbing_hand.srv import *
from rubbing_hand.msg import *

class Inhand:
    def __init__(self, rubbing, sub_fv_filtered1_objinfo, sub_fv_smaf):
        self.is_running= False
        self.rubbing = rubbing
        self.sub_fv_filtered1_objinfo = sub_fv_filtered1_objinfo
        self.sub_fv_smaf = sub_fv_smaf

        #get the angle of object
        self.get_theta = lambda: -self.sub_fv_smaf.req.data[0]
        #get the angle velocity of object
        self.get_omega = lambda: -self.sub_fv_smaf.req.data[1]
        #get the slip of object
        self.get_slip = lambda: sum(self.sub_fv_filtered1_objinfo.req.mv_s)
        self.MV = 0

        self.target_angle = 40.
        # self.min_gstep = 0.01
        self.th_slip = 0.0001
        self.target_omega = 10.
        # ex. MV_input  = [neutral_min, neutral_max , drop]
        self.MV_i = [-3, 0, 15]
        # ex. MV_output = [open, close, quick_close]
        self.MV_o = [0.001, -0.01, 0.05]

        self.hz = 60
        # self.tmp_pub = rospy.Publisher(rospy.get_namespace()+"tmp", Float64, queue_size=1)

        # publisher設定
        self.inhand_pub = rospy.Publisher("inhand", inhand, queue_size=1)
        self.pub_is_running = True
        self.Start_pub()

    def Start(self):
        self.Stop()
        self.thread= threading.Thread(name='inhand', target=self.Maniloop)
        self.is_running= True
        self.thread.start()

    def Stop(self):
        if self.is_running:
            self.is_running= False
            # self.thread.join()
        if self.pub_is_running:
            self.pub_is_running = False

    def Start_pub(self):
        pub_thread = threading.Thread(name='inhand_msg', target=self.publish_inhand_data)
        pub_thread.start()

    # 初速度vel_0から最高速度vel_tまで時間timeで一定加速度で変化する速度軌跡を生成,末尾−1
    def Create_trapezoidal_array(self, vel_t, time, vel_0 = 0):
        acc = float(vel_t-vel_0) / time
        array = [t*acc + vel_0 for t in range(time)]
        # # 番兵
        # array.append(-1)

        return array

    def sgn_(self, d_omega):
        max_=0.001
        min_=-0.5
        d_pos = -d_omega*0.01
        if d_pos>max_:
            d_pos = max_
        elif d_pos<min_:
            d_pos = min_
        return d_pos

    def publish_inhand_data(self):
        r = rospy.Rate(self.hz)
        while self.pub_is_running and not rospy.is_shutdown():
            inhand_msg = self.generate_inhand_msg()
            self.inhand_pub.publish(inhand_msg)
            r.sleep()

    def generate_inhand_msg(self):
        inhand_msg = inhand()

        inhand_msg.header.stamp = rospy.Time.now()
        inhand_msg.interval = self.rubbing.interval
        inhand_msg.MV = self.MV
        inhand_msg.mv_s = self.sub_fv_filtered1_objinfo.req.mv_s
        inhand_msg.obj_orientation = -self.sub_fv_filtered1_objinfo.req.obj_orientation
        inhand_msg.obj_orientation_filtered = -self.sub_fv_smaf.req.data[0]
        inhand_msg.d_obj_orientation_filtered = -self.sub_fv_smaf.req.data[1]
        inhand_msg.target_obj_orientation = self.target_angle
        inhand_msg.target_d_obj_orientation = self.target_omega
        inhand_msg.th_slip = self.th_slip
        inhand_msg.MV_i = self.MV_i
        inhand_msg.MV_o = self.MV_o

        return inhand_msg

    def Maniloop(self):
        time_start = time.time()
        r = rospy.Rate(self.hz)

        #get the initial angle
        avg_angle = 0
        for i in range(60):
            avg_angle += self.get_theta()
            r.sleep()
        avg_angle /= 60
        theta0 = avg_angle

        thread_cond = lambda: self.is_running and not rospy.is_shutdown()

        print("start inhand manipulation! initial angle=", theta0)

        #Open gripper until slip is detected
        self.rubbing.Go2itv(50, 0.01)
        self.MV = 0.01
        while thread_cond():
            # print(self.get_slip())
            if self.get_slip() > self.th_slip:
                # self.publish_inhand_data()
                print("open", self.get_theta(), self.get_slip())
                break
            # self.publish_inhand_data()
            r.sleep()

        #Control the velocity angle of obj
        # g_pos= self.rubbing.interval
        # self.rubbing.Set_interval(g_pos-1)

        while thread_cond():
            if self.get_theta()>self.target_angle:
                self.MV = 0
                # self.publish_inhand_data()
                print("Done!")
                break
            omega_trg = self.target_omega
            omega = self.get_omega()
            omega_d = omega_trg - omega
            self.MV = -0.05 if omega<-15 else -0.01 if omega_d>0 else 0.01
            g_pos = self.rubbing.interval - self.MV
            # print(omega_trg, omega, omega_d,  d_pos, g_pos)
            # data = Float64()
            # data.data = omega_trg
            # self.tmp_pub.publish(data)
            self.rubbing.Set_interval(g_pos)
            # self.publish_inhand_data()
            r.sleep()

        self.rubbing.Set_interval(20)
        rospy.sleep(0.5)
        avg_angle = 0
        for i in range(60):
            avg_angle += self.get_theta()
            r.sleep()
        avg_angle /= 60
        elapsed_time = time.time() - time_start
        print("elapsed time=", elapsed_time, ", last angle=", avg_angle, ", target=", self.target_angle, ", diff=", self.target_angle-avg_angle)
        

        self.Stop()

def modify_rad_curve(theta):
    k = 0.7267534839
    return np.degrees(np.arctan(k*np.tan(theta)))

def Go2itv_client(data1, data2):
  rospy.wait_for_service('/Go2itv')
  try:
    Go2itv = rospy.ServiceProxy('/Go2itv', Set2Float64)
    Go2itv(data1, data2)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e