#!/usr/bin/python
#coding: utf-8

import numpy as np
import matplotlib.pyplot as plt
import csv
import os
import re
import datetime
import time

import rospy
import roslib
roslib.load_manifest('rubbing_hand')
roslib.load_manifest('fingervision_msgs')
from rubbing_hand.msg import dynamixel_msg, dynamixel_param_msg
from fingervision_msgs.msg import ProxVision, Filter1ObjInfo
from rubbing_hand.srv import *
from std_srvs.srv import Empty
import message_filters
from subscribe import Subscribe

# https://github.com/uos/rospy_message_converter
from rospy_message_converter import message_converter

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

def set_interval_client(data):
  rospy.wait_for_service('/Set_interval')
  try:
    set_interval = rospy.ServiceProxy('/Set_interval', SetFloat64)
    set_interval(data)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def Go2itv_client(data1, data2):
  rospy.wait_for_service('/Go2itv')
  try:
    Go2itv = rospy.ServiceProxy('/Go2itv', Set2Float64)
    Go2itv(data1, data2)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def hoge(msg):
  # rospy.loginfo(msg)
  s = storing.present_data
  f = msg.header.seq
  print f, s

# def output2csv(msg_prox_l, msg_prox_r, msg_dyna_data):
#   frame_seq = [prox_data.header.seq]
#   vel_data = list(storing.present_data.Vel)
#   MvS = list(prox_data.MvS)

#   data = frame_seq + vel_data + MvS

#   mycsv.data.append(data)


class CSVoutput:
  def __init__(self, name = "test", msg_list = []):
    self.data = []
    self.file_dir = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0120/"
    self.file_Name = name
    today = datetime.datetime.now().strftime("%Y%m%d")
    self.file_name = self.file_dir + self.file_Name + today + "(000).csv"
    self.file_name = duplicate_rename(self.file_name)
    print("file name is: %s" %self.file_name)

    header = self.create_header(msg_list)

    with open(self.file_name, 'a') as f:
      writer = csv.writer(f)
      # print "File is opened!
      writer.writerow(header)

  def create_header(self, msg_list):
      header = []
      for name, mag in msg_list:
        header += dictionary_key2flatten_list(message_converter.convert_ros_message_to_dictionary(mag), name)
      return header

  def Storing(self, msg_list):
    data = []
    # time_stamp = [format(msg_objdata.header.stamp.secs+msg_objdata.header.stamp.nsecs*10**-9, '.3f')]
    for msg in msg_list:
      dictionary = message_converter.convert_ros_message_to_dictionary(msg)
      data += dictionary2flatten_list(dictionary)
    self.data.append(data)

  def Write(self):
    with open(self.file_name, 'a') as f:
      writer = csv.writer(f)
      # print "File is opened!
      for i in range(len(self.data)):
        writer.writerow(self.data[i])

def dictionary2flatten_list(d):
    l = d.values()
    value_list = []
    for el in l:
        if isinstance(el, dict):
            value_list += dictionary2flatten_list(el)
        else:
            value_list += [el]
    return value_list

def dictionary_key2flatten_list(d, initial_path):
    key_list = []
    for k, v in d.items():
        if isinstance(v, dict):
            key_list += dictionary_key2flatten_list(v, initial_path+k+"/")
        else:
            key_list += [initial_path+k]
    return key_list

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
  rospy.init_node('csvout_cync_inhand_node')

  while(True):
    input_data = raw_input("input interval range(0,50): ")
    if 0 < float(input_data) < 50:
      interval = float(input_data)
      print("interval sets ", interval)
      break
    elif input_data == "q":
      print("exit")
      rospy.signal_shutdown('finish')
    else:
      print("ERROR: invalid value")

  # storing = Storing_dydata()
  # mycsv_l = CSVoutput("thick_l(000).csv")
  runvel = 0.01
  file_name = "test" + str(interval) + "-" + str(runvel) + "_"
  msg_list = [["/dynamixel_param/", dynamixel_param_msg()], ["/fingervision/fv_filter1_objinfo/", Filter1ObjInfo()], ["/dynamixel_data/", dynamixel_msg()]]
  mycsv = CSVoutput(file_name, msg_list)

  # start_record_client("l")
  # start_record_client("r")

  # print "start record!"

  #subscribe設定
  sub_fv_filtered1_objinfo = Subscribe("Filter1ObjInfo")
  sub_dynamixel_data = Subscribe("dynamixel_data")
  rospy.sleep(0.5)
  rospy.Subscriber("/dynamixel_param", dynamixel_param_msg, lambda msg:mycsv.Storing([msg, sub_fv_filtered1_objinfo.req, sub_dynamixel_data.req]))

  # set_interval_client(interval)
  Go2itv_client(interval, runvel)

  time_start = time.time()

  print "ready OK!"

  # while True:
  #   s = raw_input('type "q" to finish: ')
  #   print s=="q"
  #   if s=="q":
  #     mycsv.Write()
  #     # stop_record_client("l")
  #     # stop_record_client("r")
  #     break
  
  while True:
    if time.time()-time_start > 20:
      mycsv.Write()
      break
    rospy.sleep(0.01)

  set_interval_client(16.0)

  rospy.signal_shutdown('finish')
  rospy.spin()

if __name__=='__main__':
  main()