#!/usr/bin/python
#coding: utf-8

import roslib; roslib.load_manifest('ay_py')
import rospy
from ay_py.core import *
from ay_py.ros import *  # LRToStrS, etc.
roslib.load_manifest('fingervision_msgs')
import fingervision_msgs.msg
import fingervision_msgs.srv
import geometry_msgs.msg
roslib.load_manifest('rubbing_hand')
import rubbing_hand.msg

rospy.init_node("dynamixel_node")

print "hogehoge"

