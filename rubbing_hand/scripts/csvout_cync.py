#!/usr/bin/python
#coding: utf-8

import numpy as np
import matplotlib.pyplot as plt
import csv
import os
import re
import datetime

import rospy
import roslib
roslib.load_manifest('rubbing_hand')
roslib.load_manifest('fingervision_msgs')
from rubbing_hand.msg import dynamixel_msg
from fingervision_msgs.msg import ProxVision
from std_srvs.srv import Empty
import message_filters

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

def output2csv(msg_prox_l, msg_prox_r, msg_dyna_data):
  frame_seq = [prox_data.header.seq]
  vel_data = list(storing.present_data.Vel)
  MvS = list(prox_data.MvS)

  data = frame_seq + vel_data + MvS

  mycsv.data.append(data)


class CSVoutput:
  def __init__(self, name = "test"):
    self.data = []
    self.file_dir = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/"
    self.file_Name = name
    today = datetime.datetime.now().strftime("%Y%m%d")
    self.file_name = self.file_dir + self.file_Name + today + "(000).csv"
    self.file_name = duplicate_rename(self.file_name)
    print("file name is: %s" %self.file_name)

    header =[
      "time",
      "vel_0", "vel_1", "vel_2", "vel_3",
      "MvS_l[00]", "MvS_l[10]", "MvS_l[20]", "MvS_l[01]", "MvS_l[11]", "MvS_l[21]", "MvS_l[02]", "MvS_l[12]", "MvS_l[22]",
      "MvS_r[00]", "MvS_r[10]", "MvS_r[20]", "MvS_r[01]", "MvS_r[11]", "MvS_r[21]", "MvS_r[02]", "MvS_r[12]", "MvS_r[22]"
    ]

    with open(self.file_name, 'a') as f:
      writer = csv.writer(f)
      # print "File is opened!
      writer.writerow(header)

  def Storing(self, msg_prox_l, msg_prox_r, msg_dyna_data):
    time_stamp = [format(msg_dyna_data.header.stamp.secs+msg_dyna_data.header.stamp.nsecs*10**-9, '.3f')]
    vel_data = list(msg_dyna_data.Vel)
    MvS_l = list(msg_prox_l.MvS)
    MvS_r = list(msg_prox_r.MvS)
    data = time_stamp + vel_data + MvS_l + MvS_r

    self.data.append(data)

  def Write(self):
    with open(self.file_name, 'a') as f:
      writer = csv.writer(f)
      # print "File is opened!
      for i in range(len(self.data)):
        writer.writerow(self.data[i])


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
  rospy.init_node('csvout_cync_node')

  # storing = Storing_dydata()
  # mycsv_l = CSVoutput("thick_l(000).csv")
  mycsv = CSVoutput("thick")

  start_record_client("l")
  start_record_client("r")

  print "start record!"
  
  sub_prox_l = message_filters.Subscriber("/fingervision/rh_l/prox_vision", ProxVision)
  sub_prox_r = message_filters.Subscriber("/fingervision/rh_r/prox_vision", ProxVision)
  sub_dyna_data = message_filters.Subscriber("/fingervision/dynamixel_data", dynamixel_msg)
  
  fps = 10. #fpsが整数だと、1/fpsをpython2が評価すると0になってしまう(整数同士の除算は切り捨て除算)
  delay = 1/fps*0.5

  ts = message_filters.ApproximateTimeSynchronizer([sub_prox_l, sub_prox_r, sub_dyna_data], 10, delay)
  ts.registerCallback(mycsv.Storing)
  
  # rospy.Subscriber("/fingervision/rh_l/prox_vision", ProxVision, lambda msg:output2csv(msg, mycsv_l, storing))
  # rospy.Subscriber("/fingervision/rh_r/prox_vision", ProxVision, lambda msg:output2csv(msg, mycsv_r, storing))
  # rospy.Subscriber("/fingervision/dynamixel_data", dynamixel_msg, storing.SetCurrentData)

  print "ready OK!"

  while True:
    s = raw_input('type "q" to finish: ')
    print s=="q"
    if s=="q":
      mycsv.Write()
      stop_record_client("l")
      stop_record_client("r")
      break

  rospy.signal_shutdown('finish')
  rospy.spin()

if __name__=='__main__':
  main()