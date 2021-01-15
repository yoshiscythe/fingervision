#!/usr/bin/python
#coding: utf-8

import numpy as np

import rospy
import roslib
roslib.load_manifest('rubbing_hand')
roslib.load_manifest('fingervision_msgs')
from rubbing_hand.msg import dynamixel_msg
from fingervision_msgs.msg import ProxVision
from fingervision_msgs.msg import Filter1ObjInfo

dist_orientation = [0 for i in range(100)]

def callback(msg):
    orientation_rad = msg.obj_orientation
    orientation_deg = np.rad2deg(orientation_rad)
    dist_orientation[1:109] = dist_orientation[:99]
    dist_orientation[0] = orientation_deg

    print(np.mean(dist_orientation))

if __name__=='__main__':
    rospy.init_node('show_orientation_node')

    rospy.Subscriber("/fingervision/fv_filter1_objinfo", Filter1ObjInfo, callback)

    rospy.spin()
