#!/usr/bin/python
#coding: utf-8
#Control dynamixel with key input (velocity control ver 1).
#use bottom before this program. fpsが上がる
#$ bash /home/suzuki/prg/DynamixelSDK/python/src/ay_Dynamixel/robots/dynamixel/fix_usb_latency.sh

import roslib; roslib.load_manifest('rubbing_hand')
import rospy, rostopic
rospy.init_node("dynamixel_node")

from rubbing_hand.msg import dynamixel_msg, dynamixel_param_msg
from rubbing_hand.srv import SetFloat64, Set2Float64, SetFloat64_array

from dxl_util import *
from _config import *
import time

# from kbhit2 import TKBHit
from rate_adjust import TRate
import threading
import sys
import evdev

from rub import Rubbing, PID
import yaml
from subscribe import Subscribe
from inhand_statemachine_util import Inhand
# from inhand_util import Inhand

file_name = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/scripts/init_pos_rhp12rn.yaml"

#ROSのpublisher設定
data_pub = rospy.Publisher("dynamixel_data", dynamixel_msg, queue_size=1)
param_pub = rospy.Publisher("dynamixel_param", dynamixel_param_msg, queue_size=1)

#subscribe設定
sub_fv_prox = Subscribe("ProxVision")
sub_fv_filtered1_objinfo = Subscribe("Filter1ObjInfo")
sub_fv_smaf = Subscribe("fv_smaf")

#Setup the device
DXL_ID= [1]   #Note: value and 
# BAUDRATE= 57600
# BAUDRATE= 115200
BAUDRATE = 2e6
DXL_TYPE= 'RH-P12-RN'
dev='/dev/ttyUSB0'
dxl= [TDynamixel1(DXL_TYPE,dev)]
# dxl[0].MIN_POSITION, dxl[0].MAX_POSITION = 0, 1300
# dxl[1].MIN_POSITION, dxl[1].MAX_POSITION = -3200, 3800
# dxl[2].MIN_POSITION, dxl[2].MAX_POSITION = 700, 1600
# dxl[3].MIN_POSITION, dxl[3].MAX_POSITION = 3700, 5000
for id in range(len(dxl)):
  dxl[id].Id= DXL_ID[id]
  dxl[id].Baudrate= BAUDRATE
  dxl[id].OpMode= 'CURRPOS'
  dxl[id].Setup()
  dxl[id].SetBaudRate(BAUDRATE)
  dxl[id].EnableTorque()

#------------------------------------------------------------------------------

'''
Holding controller for Dynamixel.
It uses a virtual offset to increase position control power (like PI).
The effect is similar to increasing POS_I_GAIN but this offset is zero when the servo
is moving; it is more stable (i.e. less vibration/chatter).

Example:
  port_locker= threading.RLock()
  def holding_observer():
    with port_locker:
      pos,vel,pwm= dxl.Position(), dxl.Velocity(), dxl.PWM()
    return pos,vel,pwm

  def holding_controller(target_position):
    with port_locker:
      dxl.MoveTo(target_position, blocking=False)

  holding= TDxlHolding()
  holding.observer= holding_observer
  holding.controller= holding_controller
  holding.SetTarget(TARGET_POS, MAX_PWM)
  holding.Start()

  user-defined-loop:
    ...
    holding.SetTarget(TARGET_POS, MAX_PWM)
    ...

  holding.Stop()

'''
class TDxlHolding(object):
  def __init__(self, rate=30):
    self.trg_pos= [2048 for id in range(len(dxl))]
    self.max_pwm= [100 for id in range(len(dxl))]
    self.is_running= False
    self.dv = 0
    self.p = 1
    self.v = 10

    self.th_p= 3
    self.th_v= 3
    self.ostep= 3
    self.ctrl_rate= rate

    self.observer= None  #Should be: pos,vel,pwm= observer()
    self.controller= None  #Should be: controller(target_velocity)

    self.CHANNELS   = [0,1,2,3,4]
    self.FLAG_MOVES = [False,False,False,False,False]
    self.DIRECTIONS = [0,0,0,0,0]

    self.init_pos_r = 2048
    self.init_pos_l = 2048

    self.offset_f = 0

    self.thick_i = 0
    self.thick_f = 0

    self.clb_cur_th = 10
    self.calibration_f = 0

    self.test_button_f = False
    self.first_f = True

    

  # #Set target velocity.
  # #  trg_vel: Target velocity.
  # def SetTarget(self, trg_vel, ids = 0):
  #   for id in ids:
  #     self.trg_vel[id] = trg_vel

  def SetTarget(self, trg_pos, max_pwm=None, id = 0):
    self.trg_pos[id]= trg_pos
    self.max_pwm[id]= max_pwm if max_pwm is not None else self.max_pwm[id]

  def Start(self):
    self.Stop()
    self.thread= threading.Thread(name='holding', target=self.Loop)
    self.is_running= True
    self.thread.start()

  def Stop(self):
    if self.is_running:
      self.is_running= False
      self.thread.join()

  def Manual_Calibration_initial_position(self):
    for id in range(len(dxl)):
      dxl[id].DisableTorque()
    time.sleep(1)
    print("指を完全に閉じてください。\nその後SLボタンを押してください")
    while(1):
      if self.FLAG_MOVES[4]:
        pos, vel, pwm, cur = self.observer()
        clb_init_pos = pos[0]
        rubbing.write_initial_position_rhp12rn(clb_init_pos)
        rubbing.init_pos_rhp12rn = clb_init_pos
        for id in range(len(dxl)):
          dxl[id].EnableTorque()
        rubbing.Go2itv(30, 0.001)
        break

  def Loop(self):
    rate= rospy.Rate(self.ctrl_rate)

    # topicの配信頻度を取得用
    # dynamixelの制御Hzをとるのじゃ！
    # rate, min_delta, max_delta, standard deviation, window number = self.topic_hz.get_hz([/dynamixel_data])
    self.topic_hz = rostopic.ROSTopicHz(600)
    s = rospy.Subscriber('/dynamixel_data', dynamixel_msg, self.topic_hz.callback_hz, callback_args='/dynamixel_data')

    while self.is_running:
      start = time.time()

      #モータの情報取得
      pos, vel, cur = self.observer()

      time_get = time.time() - start

      self.trg_pos[0] = pos

      #取得した情報をパブリッシュ
      dy_data = dynamixel_msg()
      dy_data.header.stamp = rospy.Time.now()
      dy_data.Pos = [pos]
      dy_data.Vel = [vel]
      dy_data.Pwm = [0]
      dy_data.Cur = [cur]
      data_pub.publish(dy_data)

      # #スティックX軸はモータ２（左指近位関節）を動かす
      # #曲げる方向を正とした
      # if self.DIRECTIONS[0] == 1:
      #   self.trg_pos[1] = dxl[1].Position()-self.p
      # elif self.DIRECTIONS[0] == -1:
      #   self.trg_pos[1] = dxl[1].Position()+self.p

      # #スティックY軸はモータ１（左指遠位関節）を動かす
      # if self.DIRECTIONS[1] == 1:
      #   self.trg_pos[0] = dxl[0].Position()-self.p
      # elif self.DIRECTIONS[1] == -1:
      #   self.trg_pos[0] = dxl[0].Position()+self.p

      #スティックX軸は擦り動作
      if self.DIRECTIONS[0] == 1:
        rubbing.surface_pos = rubbing.surface_pos + self.v*0.01
      elif self.DIRECTIONS[0] == -1:
        rubbing.surface_pos = rubbing.surface_pos - self.v*0.01
      
      #スティックY軸は指缶距離調整
      if self.DIRECTIONS[1] == 1:
        rubbing.Set_interval(rubbing.interval - self.p*0.1)
      elif self.DIRECTIONS[1] == -1:
        rubbing.Set_interval(rubbing.interval + self.p*0.1)

      # #ボタン左右は仮想面の傾き
      # if self.DIRECTIONS[3] == 1:
      #   rubbing.degree_of_surface += self.p*0.1
      # elif self.DIRECTIONS[3] == -1:
      #   rubbing.degree_of_surface -= self.p*0.1
      
      # #ボタン上下は指の傾き
      # if self.DIRECTIONS[2] == 1:
      #   rubbing.degree_of_finger += self.p*0.1
      # elif self.DIRECTIONS[2] == -1:
      #   rubbing.degree_of_finger -= self.p*0.1

      rubbing.control_f = any(self.FLAG_MOVES[:2])

      #目標位置の計算
      if rubbing.running:
        rubbing.Update()
        self.trg_pos[0] = rubbing.itv2pos_rhp12rn(rubbing.interval)

      # 指先を平行に保つの開始
      if self.offset_f:
        rubbing.surface_pos = 0
        rubbing.degree_of_surface = 0
        rubbing.running = 1

      if self.test_button_f:
        if self.first_f:
          rubbing.Pulse_deformed(0.01, 1.5, 5, 10, 0.3)
          self.first_f = False
        self.test_button_f = False

      # キャリブレーション
      if self.calibration_f:
        # self.Calibration_initial_position()
        self.Manual_Calibration_initial_position()
        self.calibration_f = 0

      # 目標位置をdynamixelへ送信
      # print(self.trg_pos)
      self.controller(self.trg_pos[0], 100)

      rate.sleep()

      update_fps = 1/(time.time() - start)

      #各種パラメータをパブリッシュ
      #------------------------------
      dy_param = dynamixel_param_msg()
      dy_param.header.stamp = rospy.Time.now()
      dy_param.surface_pos = rubbing.surface_pos
      dy_param.interval = rubbing.interval
      dy_param.fps = update_fps
      dy_param.trg_pos = self.trg_pos
      dy_param.degree_of_finger = rubbing.degree_of_finger
      # print(dy_param)
      param_pub.publish(dy_param)
      #-------------------------------

      time_all = time.time() - start

      # print(time_all,"s, ", 1./time_all,"Hz")
      # print(rubbing.interval, rubbing.degree_of_finger)

#------------------------------------------------------------------------------

port_locker= threading.RLock()
def holding_observer(id = 0):
  with port_locker:
    pos,vel,cur= dxl[id].Position(), dxl[id].Velocity(), dxl[id].Current()
  return pos,vel,cur

# def holding_controller(target_velocity,id = 0):
#   print target_velocity,
#   with port_locker:
#     dxl[id].SetVelocity(target_velocity)

def holding_controller(target_position, target_current, id = 0):
  # print target_position,
  with port_locker:
    dxl[id].MoveToC(target_position, target_current, blocking=False)

holding= TDxlHolding(rate=120)
holding.observer= holding_observer
holding.controller= holding_controller
# for id in range(len(dxl)):
#   holding.SetTarget(dxl[id].Position(), dxl[id].Read('GOAL_PWM')*0.9, id)
rubbing = Rubbing()
rubbing.finger_interval = 20.
rubbing.link_length = 60.
rubbing.itv_min = 0.
rubbing.filename = file_name
rubbing.read_initial_position_rhp12rn()
rubbing.holding = holding
inhand = Inhand(rubbing, sub_fv_filtered1_objinfo, sub_fv_smaf)
holding.Start()
trg = 0

rospy.Service('Set_interval', SetFloat64, lambda srv:rubbing.Set_interval(srv.data))
rospy.Service('Go2itv', Set2Float64, lambda srv:rubbing.Go2itv(srv.data1, srv.data2))
rospy.Service('Go2itv_sin', SetFloat64_array, lambda srv:rubbing.Go2itv_sin(srv.data[0], srv.data[1], srv.data[2], srv.data[3]))
rospy.Service('Pulse_deformed', SetFloat64_array, lambda srv:rubbing.Pulse_deformed(srv.data[0], srv.data[1], srv.data[2], srv.data[3], srv.data[4]))

def Search_device(device_name):
  devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
  for device in devices:
    if device.name == device_name:
      print("found "+device_name)
      return device.fn
  print('cannot find '+device_name)
  return None

#ジョイコンのキー信号取得のため
device_fn = Search_device("Joy-Con (L)")
device = evdev.InputDevice(device_fn)
print(device)
for event in device.read_loop():
  # event : code sec timestamp() type usec value
  if event.type == evdev.ecodes.EV_KEY:
    # print("{} {} {}".format(event.timestamp(), event.code, event.value))
    if event.code == 317:           # キャプチャーボタン
      break                         # 終了

    elif event.code == 305:           # 下ボタン（縦持ち）
      channel = 2                     # 
      if event.value == 0:
        holding.FLAG_MOVES[channel]  = False
        holding.DIRECTIONS[channel]  = 0
      elif event.value == 1:
        holding.FLAG_MOVES[channel]  = True
        holding.DIRECTIONS[channel]  = -1

    elif event.code == 306:         # 上ボタン（縦持ち）
      channel = 2                     # 
      if event.value == 0:
        holding.FLAG_MOVES[channel]  = False
        holding.DIRECTIONS[channel]  = 0
      elif event.value == 1:
        holding.FLAG_MOVES[channel]  = True
        holding.DIRECTIONS[channel]  = 1

    elif event.code == 307:         # 右ボタン
      channel = 3                     # 
      if event.value == 0:
        holding.FLAG_MOVES[channel]  = False
        holding.DIRECTIONS[channel]  = 0
      elif event.value == 1:
        holding.FLAG_MOVES[channel]  = True
        holding.DIRECTIONS[channel]  = 1
        holding.test_button_f = True

    elif event.code == 304:         # 左ボタン
      channel = 3                     # 
      if event.value == 0:
        holding.FLAG_MOVES[channel]  = False
        holding.DIRECTIONS[channel]  = 0
      elif event.value == 1:
        holding.FLAG_MOVES[channel]  = True
        holding.DIRECTIONS[channel]  = -1
        holding.first_f = True

    elif event.code == 318:         # Lボタン
      if event.value == 0:
        holding.p = 1
        holding.v = 10
      elif event.value == 1:        #押してる間速度早く
        holding.p = 2
        holding.v = 20

    elif event.code == 319:         # ZLボタン
      if event.value == 0:
        holding.p = 1
        holding.v = 10
      elif event.value == 1:        #押してる間もっと速度早く
        holding.p = 3
        holding.v = 30

    elif event.code == 308:         # SLボタン
      channel = 4        
      if event.value == 0:
        holding.FLAG_MOVES[channel]  = False
        holding.DIRECTIONS[channel]  = 0
      elif event.value == 1:
        holding.FLAG_MOVES[channel]  = True
        holding.DIRECTIONS[channel]  = 1
        holding.calibration_f = 1

    elif event.code == 309:         # SRボタン
      channel = 4       
      if event.value == 0:
        holding.FLAG_MOVES[channel]  = False
        holding.DIRECTIONS[channel]  = 0
      elif event.value == 1:
        holding.FLAG_MOVES[channel]  = True
        holding.DIRECTIONS[channel]  = -1
        inhand.Start()
        # holding.thick_f = 1

    


    elif event.code == 312:         # マイナスボタン
      if event.value == 1:
        holding.offset_f = 1
      elif event.value == 0:
        holding.offset_f = 0

      # if event.value == 1:
      #   with port_locker:
      #     for id in range(len(dxl)):           #リブート
      #       dxl[id].Reboot()
      #       time.sleep(0.1)
      #       dxl[id].EnableTorque()
      #       print(id, 'Done')

  elif event.type == evdev.ecodes.EV_ABS:
    # print("{} {} {}".format(event.timestamp(), event.code, event.value))

    if event.code == 17:     # X軸,左右
      channel = 0     # 
      if event.value == -1:
        holding.FLAG_MOVES[channel]  = True
        holding.DIRECTIONS[channel]  = 1
      elif event.value == 0:
        holding.FLAG_MOVES[channel]  = False
        holding.DIRECTIONS[channel]  = 0
      elif event.value == 1:
        holding.FLAG_MOVES[channel]  = True
        holding.DIRECTIONS[channel]  = -1

    elif event.code == 16:   # y軸,上下
      channel = 1     #
      if event.value == -1:
        holding.FLAG_MOVES[channel]  = True
        holding.DIRECTIONS[channel]  = 1
      elif event.value == 0:
        holding.FLAG_MOVES[channel]  = False
        holding.DIRECTIONS[channel]  = 0
      elif event.value == 1:
        holding.FLAG_MOVES[channel]  = True
        holding.DIRECTIONS[channel]  = -1

# is_running[0]= False
# t1.join()
holding.Stop()
inhand.Stop()

for id in range(len(dxl)):
  #dxl[id].PrintStatus()
  #dxl[id].PrintHardwareErrSt()
  dxl[id].DisableTorque()
dxl[0].Quit()
