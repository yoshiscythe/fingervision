#!/usr/bin/python
#coding: utf-8

import numpy as np
import threading
import rospy
from rub import PID

import roslib
roslib.load_manifest('rubbing_hand')
from rubbing_hand.srv import *

class Inhand:
    def __init__(self, rubbing, sub_fv_filtered1_objinfo, sub_fv_lkf):
        self.is_running= False
        self.rubbing = rubbing
        self.sub_fv_filtered1_objinfo = sub_fv_filtered1_objinfo
        self.sub_fv_lkf = sub_fv_lkf

        #get the angle of object
        self.get_theta = lambda: modify_rad_curve(self.sub_fv_lkf.req.data[0])
        #get the angle velocity of object
        self.get_omega = lambda: self.sub_fv_lkf.req.data[1]
        #get the slip of object
        self.get_slip = lambda: sum(self.sub_fv_filtered1_objinfo.req.mv_s)

        self.target_angle = np.deg2rad(40)
        self.min_gstep = 0.01
        self.th_slip = 0.0001
        self.target_omega = -0.2

        self.hz = 60

    def Start(self):
        self.Stop()
        self.thread= threading.Thread(name='inhand', target=self.Maniloop)
        self.is_running= True
        self.thread.start()

    def Stop(self):
        if self.is_running:
            self.is_running= False
            # self.thread.join()

    def Maniloop(self):
        #get the initial angle
        theta0 = self.get_theta()
        thread_cond = lambda: self.is_running and not rospy.is_shutdown()
        r = rospy.Rate(self.hz)

        print("start inhand manipulation")

        #Open gripper until slip is detected
        self.rubbing.Go2itv(50, 0.005)
        while thread_cond():
            print(self.get_slip())
            if self.get_slip() > self.th_slip:
                print("open", self.get_theta(), self.get_slip())
                break
            r.sleep()

        #Control the velocity angle of obj
        g_pos= self.rubbing.interval
        # self.rubbing.Set_interval(g_pos-1)
        while thread_cond():
            if abs(theta0-self.get_theta())>self.target_angle:
                print("Done!")
                break
            omega = self.get_omega()
            omega_d = self.target_omega - omega
            g_pos -= omega_d
            print(g_pos)
            self.rubbing.Set_interval(g_pos)
            r.sleep()

        
        print("angle=", np.rad2deg(self.get_theta()), ", target=", np.rad2deg(self.target_angle))
        self.rubbing.Set_interval(15)

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