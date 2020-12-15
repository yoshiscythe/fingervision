#!/usr/bin/python
#coding: utf-8

import numpy as np
import threading
import rospy

class Inhand:
    def __init__(self, rubbing, sub_fv_filtered1_objinfo):
        self.is_running= False
        self.rubbing = rubbing
        self.sub_fv_filtered1_objinfo = sub_fv_filtered1_objinfo

        #get the angle of object
        self.get_theta = lambda: self.sub_fv_filtered1_objinfo.req.obj_orientation
        #get the slip of object
        self.get_slip = lambda: sum(self.sub_fv_filtered1_objinfo.req.mv_s)

        self.target_angle = np.deg2rad(20)
        self.min_gstep = 0.1
        self.th_slip = 0.001

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

        print("start inhand manipulation")

        #loop while rospy is not shutdown
        while thread_cond():
            g_pos= self.rubbing.interval

            if abs(theta0-self.get_theta())>self.target_angle:
                print("Done! target=", np.rad2deg(self.target_angle))
                g_pos -= self.min_gstep*5
                self.rubbing.interval = g_pos
                break

            #Open gripper until slip is detected
            g_pos= self.rubbing.interval
            while thread_cond() and (self.get_slip() < self.th_slip):
                g_pos += self.min_gstep
                self.rubbing.interval = g_pos
                for i in range(10):
                    if self.get_slip() >= self.th_slip:  break
                    rospy.sleep(0.01)
                print("open", self.get_theta(), self.get_slip())

            #Close gripper to stop slip
            g_pos= self.rubbing.interval
            while thread_cond() and (self.get_slip() > self.th_slip):
                g_pos -= self.min_gstep
                self.rubbing.interval = g_pos
                for i in range(10):
                    if self.get_slip() < self.th_slip:  break
                    rospy.sleep(0.01)
                print("close", self.get_theta(), self.get_slip())
            
            print(np.rad2deg(theta0-self.get_theta()), np.rad2deg(self.get_theta()), self.get_theta())
            # rospy.sleep(0.1)

        self.Stop()
