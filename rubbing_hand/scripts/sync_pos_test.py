#!/usr/bin/python
#coding: utf-8
#Control dynamixel with key input (velocity control ver 1).

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

file_name = "/home/suzuki/prg/DynamixelSDK/python/src/ay_Dynamixel/robots/dynamixel/init_pos.yaml"

#Setup the device
DXL_ID= [1,2,3,4]   #Note: value and 
BAUDRATE= 57600
DXL_TYPE= 'XM430-W350'
dev='/dev/ttyUSB0'
dxl= [TDynamixel1(DXL_TYPE,dev), TDynamixel1(DXL_TYPE,dev), TDynamixel1(DXL_TYPE,dev), TDynamixel1(DXL_TYPE,dev)]
# dxl[0].MIN_POSITION, dxl[0].MAX_POSITION = 0, 1300
# dxl[1].MIN_POSITION, dxl[1].MAX_POSITION = -3200, 3800
# dxl[2].MIN_POSITION, dxl[2].MAX_POSITION = 700, 1600
# dxl[3].MIN_POSITION, dxl[3].MAX_POSITION = 3700, 5000
for id in range(len(dxl)):
  dxl[id].Id= DXL_ID[id]
  dxl[id].Baudrate= BAUDRATE
  dxl[id].OpMode= 'POSITION'
  dxl[id].Setup()
  dxl[id].SetPosGain(150, 50, 10)
  # dxl[id].SetPosGain(900, 0, 0)
  dxl[id].EnableTorque()
dxl[0].SetupPosSyncWrite()
dxl[0].SetupSyncRead()
# dxl[0].SetupBulkRead()
# dxl[0].SetupPosSyncRead()
# dxl[0].SetupVelSyncRead()
# dxl[0].SetupPwmSyncRead()

# def ReadKeyboard(is_running, key_cmd, key_locker):
#   kbhit= TKBHit()
#   dt_hold= 0.1
#   t_prev= 0
#   while is_running[0]:
#     c= kbhit.KBHit()
#     if c is not None or time.time()-t_prev>dt_hold:
#       with key_locker:
#         key_cmd[0]= c
#       t_prev= time.time()
#     time.sleep(0.0025)

# key_cmd= [None]
# key_locker= threading.RLock()
# is_running= [True]
# t1= threading.Thread(name='t1', target=lambda a1=is_running,a2=key_cmd,a3=key_locker: ReadKeyboard(a1,a2,a3))
# t1.start()


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
    self.p = 10
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

  def Loop(self):
    sign= lambda x: 1 if x>0 else -1 if x<0 else 0

    rate= TRate(self.ctrl_rate)

    #Virtual offset:
    self.trg_offset= 0.0

    while self.is_running:
      start = time.time()

      # pos = [dxl[id].Position() for id in range(len(dxl))]
      # pos = [dxl[0].Position(), dxl[1].Position(), dxl[2].Position(), dxl[3].Position()]
      pos, vel, pwm = self.observer()
      # all_data = dxl[0].ReadAllSync()
      # print(all_data)

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
      if self.DIRECTIONS[0] == 1:
        rubbing.surface_pos = rubbing.surface_pos + self.v*0.1
      elif self.DIRECTIONS[0] == -1:
        rubbing.surface_pos = rubbing.surface_pos - self.v*0.1
      
      #スティックY軸はモータ１（左指遠位関節）を動かす
      if self.DIRECTIONS[1] == 1:
        rubbing.interval = rubbing.interval + 1
      elif self.DIRECTIONS[1] == -1:
        rubbing.interval = rubbing.interval - 1

      # #ボタン左右はモータ4（右指近位関節）を動かす
      # if self.DIRECTIONS[3] == 1:
      #   self.trg_vel[3] = self.v
      # elif self.DIRECTIONS[3] == -1:
      #   self.trg_vel[3] = -self.v
      # else:
      #   self.trg_vel[3] = 0
      
      # #ボタン上下はモータ3（右指遠位関節）を動かす
      # if self.DIRECTIONS[2] == 1:
      #   self.trg_vel[2] = self.v
      # elif self.DIRECTIONS[2] == -1:
      #   self.trg_vel[2] = -self.v
      # else:
      #   self.trg_vel[2] = 0  

      # if self.DIRECTIONS[4] == 1:
      #   self.trg_vel[0] = self.v
      #   self.trg_vel[2] = self.v
      # elif self.DIRECTIONS[4] == -1:
      #   self.trg_vel[0] = -self.v
      #   self.trg_vel[2] = -self.v

      if rubbing.running:
        a, b = rubbing.calculation_degree()
        pos_r, pos_l = rubbing.deg2pos(a, b)
        self.trg_pos[2], self.trg_pos[0] = int(pos_r), int(pos_l)

      if self.offset_f:
        # rubbing.init_pos_r = pos[2]
        # rubbing.init_pos_l = pos[0]
        with open(file_name) as file:
          obj = yaml.safe_load(file)
          rubbing.init_pos_r = obj['init_pos_r']
          rubbing.init_pos_l = obj['init_pos_l'] 
        rubbing.running = 1

      # self.controller(self.trg_pos)
      # for id in range(len(dxl)):
      #   # if (self.trg_vel[id] > 0 and dxl[id].Position() > dxl[id].MAX_POSITION) or (self.trg_vel[id] < 0 and dxl[id].Position() < dxl[id].MIN_POSITION):
      #   #   self.trg_vel[id] = 0
      #   self.controller(int(self.trg_pos[id]), id)
      # print ''

      dummy_byte = [b'\x00\x00\x00\x00' for i in range(4)]
      # print(dummy_byte)
      # print(type(self.trg_pos[0]))
      self.controller(self.trg_pos)

    #   for id in [0, 1, 2, 3]:
    #     # if (self.trg_vel[id] > 0 and dxl[id].Position() > dxl[id].MAX_POSITION) or (self.trg_vel[id] < 0 and dxl[id].Position() < dxl[id].MIN_POSITION):
    #     #   self.trg_vel[id] = 0
    #     self.controller(int(self.trg_pos[id]), id)
      print ''  

      print "fps: ", 1/(time.time() - start)
      print rubbing.surface_pos, rubbing.interval
      print pos, vel, pwm

      # for id in range(len(dxl)):
      #   print pos[id],
      # print ''
      print '\033[4A', 

      # rate.sleep()

#------------------------------------------------------------------------------

port_locker= threading.RLock()
def holding_observer(id = 0):
  with port_locker:
    pos,vel,pwm= dxl[id].Position(), dxl[id].Velocity(), dxl[id].PWM()
  return pos,vel,pwm

def sync_observer():
  with port_locker:
    dxl[0].ReadAllSync()
    pos = dxl[0].ReadXSync("PRESENT_POSITION")
    vel = dxl[0].ReadXSync("PRESENT_VELOCITY")
    pwm = dxl[0].ReadXSync("PRESENT_PWM")
  return pos, vel, pwm

def bulk_observer():
  with port_locker:
    pos = dxl[0].ReadPosBulk()
    vel = dxl[0].ReadVelBulk()
    pwm = dxl[0].ReadPwmBulk()
  return pos, vel, pwm


# def holding_controller(target_velocity,id = 0):
#   print target_velocity,
#   with port_locker:
#     dxl[id].SetVelocity(target_velocity)

def holding_controller(target_position, id = 0):
  print target_position,
  with port_locker:
    dxl[id].MoveTo(target_position, blocking=False)

def syncpos_cotroller(target):
  with port_locker:
    dxl[0].SetPosition4PosSync(target)

rubbing = Rubbing()
holding= TDxlHolding()
holding.observer= sync_observer
# holding.controller= holding_controller
holding.controller= syncpos_cotroller
for id in range(len(dxl)):
  holding.SetTarget(dxl[id].Position(), dxl[id].Read('GOAL_PWM')*0.9, id)
holding.Start()
trg = 0

def Search_device(device_name):
  devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
  for device in devices:
    if device.name == device_name:
      print("found "+device_name)
      return device.fn
  print('cannot find '+device_name)
  return None

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

    elif event.code == 304:         # 左ボタン
      channel = 3                     # 
      if event.value == 0:
        holding.FLAG_MOVES[channel]  = False
        holding.DIRECTIONS[channel]  = 0
      elif event.value == 1:
        holding.FLAG_MOVES[channel]  = True
        holding.DIRECTIONS[channel]  = -1

    elif event.code == 318:         # Lボタン
      if event.value == 0:
        holding.p = 10
        holding.v = 10
      elif event.value == 1:        #押してる間速度早く
        holding.p = 20
        holding.v = 20

    elif event.code == 319:         # ZLボタン
      if event.value == 0:
        holding.p = 10
        holding.v = 10
      elif event.value == 1:        #押してる間もっと速度早く
        holding.p = 30
        holding.v = 30

    elif event.code == 308:         # SLボタン
      channel = 4        
      if event.value == 0:
        holding.FLAG_MOVES[channel]  = False
        holding.DIRECTIONS[channel]  = 0
      elif event.value == 1:
        holding.FLAG_MOVES[channel]  = True
        holding.DIRECTIONS[channel]  = 1

    elif event.code == 309:         # SRボタン
      channel = 4       
      if event.value == 0:
        holding.FLAG_MOVES[channel]  = False
        holding.DIRECTIONS[channel]  = 0
      elif event.value == 1:
        holding.FLAG_MOVES[channel]  = True
        holding.DIRECTIONS[channel]  = -1

    


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

for id in range(len(dxl)):
  #dxl[id].PrintStatus()
  #dxl[id].PrintHardwareErrSt()
  dxl[id].DisableTorque()
dxl[0].Quit()