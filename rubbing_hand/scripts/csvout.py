#!/usr/bin/python
#coding: utf-8

import numpy as np
import matplotlib.pyplot as plt
import csv
import os
import re

import rospy
import roslib
roslib.load_manifest('rubbing_hand')
roslib.load_manifest('fingervision_msgs')
from rubbing_hand.msg import dynamixel_msg
from fingervision_msgs.msg import ProxVision
from std_srvs.srv import Empty

def start_record_client(rl):
  rl = str(rl)
  service_name = "/fingervision/rh_"+rl+"/start_record"
  rospy.wait_for_service(service_name)
  try:
    start = rospy.ServiceProxy(service_name, Empty)
    start()

  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def stop_record_client(rl):
  rl = str(rl)
  service_name = "/fingervision/rh_"+rl+"/stop_record"
  rospy.wait_for_service(service_name)
  try:
    stop = rospy.ServiceProxy(service_name, Empty)
    stop()

  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def pause_client():
  rospy.wait_for_service('/fingervision/rh_l/pause')
  try:
    pause = rospy.ServiceProxy('/fingervision/rh_l/pause', Empty)
    pause()
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def resume_client():
  rospy.wait_for_service('/fingervision/rh_l/resume')
  try:
    resume = rospy.ServiceProxy('/fingervision/rh_l/resume', Empty)
    resume()
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def hoge(msg):
  # rospy.loginfo(msg)
  s = storing.present_data
  f = msg.header.seq
  print f, s

def output2csv(prox_data, mycsv, storing):
  frame_seq = [prox_data.header.seq]
  vel_data = list(storing.present_data.Vel)
  MvS = list(prox_data.MvS)

  data = frame_seq + vel_data + MvS

  mycsv.Write(data)


class CSVoutput:
  def __init__(self, name = "test(001).csv"):
    self.file_dir = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/"
    self.file_Name = name
    self.file_name = self.file_dir + self.file_Name
    self.file_name = duplicate_rename(self.file_name)
    print("file name is: %s" %self.file_name)

    header =[
      "frame",
      "vel_0", "vel_1", "vel_2", "vel_3",
      "MvS[00]", "MvS[10]", "MvS[20]", "MvS[01]", "MvS[11]", "MvS[21]", "MvS[02]", "MvS[12]", "MvS[22]"
    ]

    with open(self.file_name, 'a') as f:
      writer = csv.writer(f)
      # print "File is opened!
      writer.writerow(header)

  def Write(self, data):
    with open(self.file_name, 'a') as f:
      writer = csv.writer(f)
      # print "File is opened!
      writer.writerow(data)


class Storing_dydata:
  def __init__(self):
    self.present_data = dynamixel_msg()
    self.present_data.Vel = [0, 0, 0, 0]

  def SetCurrentData(self, data):
    self.present_data = data

# 同じファイル名が存在するかチェック
def duplicate_rename(value, count=1):
  if os.path.exists(value):
    # 存在した場合

    # フルパスから「フルパスタイトル」と「拡張子」を分割
    ftitle, fext = os.path.splitext(value)

    # タイトル末尾に(数値)が在れば削除する。
    newftitle = re.sub(r'\(\d{3}\)$', "", ftitle)

    # (001) という文字列を作成
    addPara = '(' + "{:03}".format(count) + ')'

    # フルパスタイトル + (001) + 拡張子のファイル名を作成
    fpath = os.path.join(newftitle + addPara + fext)

    # リネームしたファイルを表示
    # print('Rename: %s' % fpath)

    # 再度渡してリネームしたファイル名が存在しないかチェック
    return (duplicate_rename(fpath, count + 1))
  else:
    # 存在しない場合
    return value

def main():
  rospy.init_node('csvout_node')

  storing = Storing_dydata()
  mycsv_l = CSVoutput("thick_l(000).csv")
  mycsv_r = CSVoutput("thick_r(000).csv")

  start_record_client("l")
  start_record_client("r")

  print "start record!"
  
  rospy.Subscriber("/fingervision/rh_l/prox_vision", ProxVision, lambda msg:output2csv(msg, mycsv_l, storing))
  rospy.Subscriber("/fingervision/rh_r/prox_vision", ProxVision, lambda msg:output2csv(msg, mycsv_r, storing))
  rospy.Subscriber("/fingervision/dynamixel_data", dynamixel_msg, storing.SetCurrentData)

  print "ready OK!"

  while True:
    s = raw_input('type "q" to finish: ')
    print s=="q"
    if s=="q":
      stop_record_client("l")
      stop_record_client("r")
      break

  rospy.signal_shutdown('finish')
  rospy.spin()

if __name__=='__main__':
  main()