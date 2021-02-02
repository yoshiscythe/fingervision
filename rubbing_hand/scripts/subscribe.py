#!/usr/bin/python
#coding: utf-8

import numpy as np
import rospy
import roslib
roslib.load_manifest('rubbing_hand')
roslib.load_manifest('fingervision_msgs')
from rubbing_hand.msg import *
from fingervision_msgs.msg import ProxVision
from fingervision_msgs.msg import Filter1ObjInfo
from std_srvs.srv import Empty

class Subscribe:
    def __init__(self, topic):
        if topic == "ProxVision":
            self.topic_name = "/fingervision/fv_1/prox_vision"
            self.topic_type = ProxVision 
        elif topic == "Filter1ObjInfo":
            self.topic_name = "/fingervision/fv_filter1_objinfo"
            self.topic_type = Filter1ObjInfo
        elif topic == "dynamixel_param":
            self.topic_name = "/dynamixel_param"
            self.topic_type = dynamixel_param_msg
        elif topic == "dynamixel_data":
            self.topic_name = "/dynamixel_data"
            self.topic_type = dynamixel_msg
        elif topic == "fv_lkf":
            self.topic_name = "/fingervision/obj_orientation_lkf1"
            self.topic_type = Float64Array
        else:
            self.topic_name = None
            self.topic_type = None
            print("topic is not found")
        self.req = self.topic_type()
        self.start()

    def set_data(self, req):
        self.req = req

    def callback(self,req):
        self.set_data(req)

    def start(self):
        rospy.Subscriber(self.topic_name, self.topic_type, self.callback)

def main():
    rospy.init_node('subscribe_test_node')

    sub_fv_prox = Subscribe("ProxVision")
    # sub_fv_prox.start()
    sub_fv_filtered1_objinfo = Subscribe("Filter1ObjInfo")
    # sub_fv_filtered1_objinfo.start()

    while(1):
        print(sub_fv_prox.req.MvS)
        print(sub_fv_filtered1_objinfo.req.obj_orientation)
        rospy.sleep(1)

    # rospy.spin()

if __name__=='__main__':
  main()