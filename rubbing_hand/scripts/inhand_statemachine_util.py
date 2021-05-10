#!/usr/bin/python
#coding: utf-8

import numpy as np
import threading
import rospy
from rub import PID
import time
from state_machine3 import TStateMachine

import roslib
roslib.load_manifest('rubbing_hand')
from rubbing_hand.srv import *
from rubbing_hand.msg import *

class Inhand:
    def __init__(self, rubbing, sub_fv_filtered1_objinfo, sub_fv_smaf):
        self.is_running= False
        self.rubbing = rubbing
        self.sub_fv_filtered1_objinfo = sub_fv_filtered1_objinfo
        self.sub_fv_smaf = sub_fv_smaf

        #get the angle of object
        self.get_theta = lambda: -np.degrees(self.sub_fv_filtered1_objinfo.req.obj_orientation)
        #get the angle velocity of object
        self.get_omega = lambda: -self.sub_fv_smaf.req.data[1]
        # self.get_omega = lambda: np.degrees(self.sub_fv_filtered1_objinfo.req.d_obj_orientation)
        self.get_alpha = lambda: -self.sub_fv_smaf.req.data[4]
        #get the slip of object
        self.get_slip = lambda: sum(self.sub_fv_filtered1_objinfo.req.mv_s)
        self.MV = 0
        self.omega_d = 0

        self.target_angle = 60.
        # self.min_gstep = 0.01
        self.th_slip = 0.0001
        self.target_omega = 15.
        # ex. MV_input  = [neutral_min, neutral_max]
        self.MV_i = [-5, 0]
        # ex. MV_output = [open, close]
        self.MV_o = [0.008, -0.05]

        self.hz = 60
        # self.tmp_pub = rospy.Publisher(rospy.get_namespace()+"tmp", Float64, queue_size=1)

        self.debug_array = []

        self.rubbing.itv_min=5

        # publisher設定
        self.inhand_pub = rospy.Publisher("inhand", inhand, queue_size=1)
        self.pub_is_running = True
        self.Start_pub()

    #インハンドマニピュレーションを開始する関数
    #Maniloop関数をスレッドで実行
    def Start(self):
        # self.Stop()
        self.thread= threading.Thread(name='inhand', target=self.Maniloop)
        self.is_running= True
        self.thread.start()

    #すべてのスレッドを終了させる関数
    #プログラムの終了前に呼び出そう
    def Stop(self):
        if self.is_running:
            self.is_running= False
            # self.thread.join()
        if self.pub_is_running:
            self.pub_is_running = False

    #インハンドマニピュレーションでつかったデータをpublishする関数をスレッドで開始
    #initで呼び出してる
    def Start_pub(self):
        pub_thread = threading.Thread(name='inhand_msg', target=self.publish_inhand_data)
        pub_thread.start()

    # 初速度vel_0から最高速度vel_tまで時間timeで一定加速度で変化する速度軌跡を生成,末尾−1
    def Create_trapezoidal_array(self, vel_t, time, vel_0 = 0):
        acc = float(vel_t-vel_0) / time
        array = [t*acc + vel_0 for t in range(time)]
        # # 番兵
        # array.append(-1)

        return array

    #シグナム関数
    #今は使ってないかな
    def sgn_(self, d_omega):
        max_=0.001
        min_=-0.5
        d_pos = -d_omega*0.01
        if d_pos>max_:
            d_pos = max_
        elif d_pos<min_:
            d_pos = min_
        return d_pos

    #インハンドマニピュレーションでつかったデータをpublishする関数
    #スレッドで使うことを想定
    def publish_inhand_data(self):
        r = rospy.Rate(self.hz)
        while self.pub_is_running and not rospy.is_shutdown():
            inhand_msg = self.generate_inhand_msg()
            self.inhand_pub.publish(inhand_msg)
            r.sleep()

    #インハンドマニピュレーションでつかったデータをまとめて、inhand_msgを生成する
    #publish_inhand_data用の関数
    def generate_inhand_msg(self):
        inhand_msg = inhand()

        inhand_msg.header.stamp = rospy.Time.now()
        inhand_msg.interval = self.rubbing.interval
        inhand_msg.MV = self.MV
        inhand_msg.mv_s = self.sub_fv_filtered1_objinfo.req.mv_s
        inhand_msg.obj_orientation = -self.sub_fv_filtered1_objinfo.req.obj_orientation
        inhand_msg.obj_orientation_filtered = self.get_theta()
        inhand_msg.d_obj_orientation_filtered = self.get_omega()
        inhand_msg.target_obj_orientation = self.target_angle
        inhand_msg.target_d_obj_orientation = self.target_omega
        inhand_msg.omega_d = self.omega_d
        inhand_msg.th_slip = self.th_slip
        inhand_msg.MV_i = self.MV_i
        inhand_msg.MV_o = self.MV_o
        inhand_msg.debag = self.debug_array

        return inhand_msg

    #d_omegaから操作量MVを決定
    def change_MV(self, d_omega, omega):
        if d_omega <= self.MV_i[0]:
            MV = self.MV_o[0]
        elif self.MV_i[0] < d_omega <= self.MV_i[1]:
            MV = 0
        elif self.MV_i[1] < d_omega:
            MV = self.MV_o[1]
        else:
            MV = 0
        
        return MV

    #開く速度を入力値から得る
    #速度変えるたびにコード書き直してノード起動し直すのが面倒だったのでこうした
    def Set_open_step(self):
        input_data = raw_input("input open step: ")
        
        self.MV_o[0] = float(input_data)
        print("set: ", self.MV_o[0])

    def calculate_omega_d(self):
        omega_trg = self.target_omega
        omega = self.get_omega()
        omega_d = omega - omega_trg
        self.omega_d = omega_d

        return omega_d

    def Substitution_MV(self, MV):
        self.MV = MV

    def Action_close(self):
        MV = self.MV_o[1]
        print("close")

        if self.get_alpha() > 500:
            MV += -0.1
            print("too fast")

        self.MV = MV
        self.rubbing.Go2itv(self.rubbing.interval-0.2, self.MV)
        

    def Maniloop(self):
        self.Set_open_step()


        time_start = time.time()
        r = rospy.Rate(self.hz)

        #初期の指間距離保存
        grasp_itv = self.rubbing.interval

        #get the initial angle
        avg_angle = 0
        for i in range(60):
            avg_angle += self.get_theta()
            r.sleep()
        avg_angle /= 60
        theta0 = avg_angle

        thread_cond = lambda: self.is_running and not rospy.is_shutdown()

        print("start inhand manipulation! initial angle=", theta0)

        last_omega_d = 0
        rotation_f = False
        error2 = 0
        error = 0

        def GetStartTime():
            global start_time
            start_time= int(time.time())

        # states= {
        #     'start': [
        #     ('entry',lambda: Print('start inhand manipulation')),
        #     ('else','judge'),
        #     ],
        #     'judge': [
        #     ('entry',lambda: self.calculate_omega_d()),
        #     (lambda: self.omega_d <= self.MV_i[0],'open'),
        #     (lambda: self.MV_i[1] < self.omega_d,'close'),
        #     (lambda: self.MV_i[0] < self.omega_d <= self.MV_i[1],'stay'),
        #     ('else','judge',lambda: Print('judge failed, omega_d:'+str(self.omega_d))),
        #     ],
        #     'open': [
        #     ('entry',lambda: (self.Substitution_MV(self.MV_o[0]), self.rubbing.Go2itv(self.rubbing.interval+0.2, self.MV))),
        #     (lambda: self.get_theta()>self.target_angle,'finish',lambda: Print('over target theta!')),
        #     (lambda: not self.calculate_omega_d() <= self.MV_i[0],'judge'),
        #     (lambda: not self.rubbing.go2itv_f,'wait',lambda: Print('to wait')),
        #     ('else','open'),
        #     ],
        #     'close': [
        #     ('entry',lambda: (self.Substitution_MV(self.MV_o[1]), self.rubbing.Go2itv(self.rubbing.interval-0.2, self.MV))),
        #     (lambda: self.get_theta()>self.target_angle,'finish',lambda: Print('over target theta!')),
        #     (lambda: not self.MV_i[1] < self.calculate_omega_d(),'judge'),
        #     (lambda: not self.rubbing.go2itv_f,'wait'),
        #     ('else','close'),
        #     ],
        #     'wait': [
        #     ('entry',lambda:(self.Substitution_MV(0), GetStartTime())),
        #     (lambda: self.get_theta()>self.target_angle,'finish',lambda: Print('over target theta!')),
        #     (lambda: self.calculate_omega_d() >= self.MV_i[1],'judge'),
        #     (lambda: (int(time.time())-start_time)>=2,'judge'),
        #     ('else','wait'),
        #     ],
        #     'stay': [
        #     ('entry',lambda: self.Substitution_MV(0)),
        #     (lambda: self.get_theta()>self.target_angle,'finish',lambda: Print('over target theta!')),
        #     (lambda: not self.MV_i[0] < self.calculate_omega_d() <= self.MV_i[1],'judge'),
        #     ('else','stay'),
        #     ],
        #     'finish': [
        #     ('entry',lambda: (self.Substitution_MV(0), Print('Finishing state machine'))),
        #     ('else','.exit'),
        #     ],
        # }

        # states= {
        #     'start': [
        #     ('entry',lambda: Print('start inhand manipulation')),
        #     ('else','judge'),
        #     ],
        #     'judge': [
        #     ('entry',lambda: self.calculate_omega_d()),
        #     (lambda: self.omega_d <= self.MV_i[0],'open'),
        #     (lambda: self.MV_i[1] < self.omega_d,'close'),
        #     (lambda: self.MV_i[0] < self.omega_d <= self.MV_i[1],'stay'),
        #     ('else','judge',lambda: Print('judge failed, omega_d:'+str(self.omega_d))),
        #     ],
        #     'open': [
        #     ('entry',lambda: (self.Substitution_MV(self.MV_o[0]), self.rubbing.Go2itv_sin(self.rubbing.interval+5, self.MV, 0.5, 5.))),
        #     (lambda: self.get_theta()>self.target_angle,'finish',lambda: Print('over target theta!')),
        #     (lambda: not self.calculate_omega_d() <= self.MV_i[0],'judge'),
        #     (lambda: not self.rubbing.go2itv_f,'judge'),
        #     ('else','open'),
        #     ],
        #     'close': [
        #     ('entry',lambda: (self.Substitution_MV(self.MV_o[1]),self.rubbing.Set_interval(self.rubbing.interval-1), self.rubbing.Go2itv(self.rubbing.interval-10, self.MV))),
        #     (lambda: self.get_theta()>self.target_angle,'finish',lambda: Print('over target theta!')),
        #     (lambda: not self.MV_i[1] < self.calculate_omega_d(),'judge'),
        #     (lambda: not self.rubbing.go2itv_f,'wait'),
        #     ('else','close'),
        #     ],
        #     'wait': [
        #     ('entry',lambda:(self.Substitution_MV(0), GetStartTime())),
        #     (lambda: self.get_theta()>self.target_angle,'finish',lambda: Print('over target theta!')),
        #     (lambda: self.calculate_omega_d() >= self.MV_i[1],'judge'),
        #     (lambda: (int(time.time())-start_time)>=2,'judge'),
        #     ('else','wait'),
        #     ],
        #     'stay': [
        #     ('entry',lambda: self.Substitution_MV(0)),
        #     (lambda: self.get_theta()>self.target_angle,'finish',lambda: Print('over target theta!')),
        #     (lambda: not self.MV_i[0] < self.calculate_omega_d() <= self.MV_i[1],'judge'),
        #     ('else','stay'),
        #     ],
        #     'finish': [
        #     ('entry',lambda: (self.Substitution_MV(0), Print('Finishing state machine'))),
        #     ('else','.exit'),
        #     ],
        # }

        states= {
            'start': [
            ('entry',lambda: Print('start inhand manipulation')),
            ('else','judge'),
            ],
            'judge': [
            ('entry',lambda: self.calculate_omega_d()),
            (lambda: self.omega_d <= self.MV_i[0],'open'),
            (lambda: self.MV_i[1] < self.omega_d,'close'),
            (lambda: self.MV_i[0] < self.omega_d <= self.MV_i[1],'stay'),
            ('else','judge',lambda: Print('judge failed, omega_d:'+str(self.omega_d))),
            ],
            'open': [
            ('entry',lambda: (self.Substitution_MV(self.MV_o[0]), self.rubbing.Pulse(self.MV, 1.5, 5., 1.))),
            (lambda: not self.rubbing.go2itv_f,'judge'),
            ('else','open'),
            ],
            'close': [
            ('entry',lambda: (self.Substitution_MV(self.MV_o[1]),self.rubbing.Set_interval(self.rubbing.interval), self.rubbing.Go2itv(self.rubbing.interval-10, self.MV))),
            (lambda: self.get_theta()>self.target_angle,'finish',lambda: Print('over target theta!')),
            (lambda: not self.MV_i[1] < self.calculate_omega_d(),'judge'),
            (lambda: not self.rubbing.go2itv_f,'wait'),
            ('else','close'),
            ],
            'wait': [
            ('entry',lambda:(self.Substitution_MV(0), GetStartTime())),
            (lambda: self.get_theta()>self.target_angle,'finish',lambda: Print('over target theta!')),
            (lambda: self.calculate_omega_d() >= self.MV_i[1],'judge'),
            (lambda: (int(time.time())-start_time)>=2,'judge'),
            ('else','wait'),
            ],
            'stay': [
            ('entry',lambda: self.Substitution_MV(0)),
            (lambda: self.get_theta()>self.target_angle,'finish',lambda: Print('over target theta!')),
            (lambda: not self.MV_i[0] < self.calculate_omega_d() <= self.MV_i[1],'judge'),
            ('else','stay'),
            ],
            'finish': [
            ('entry',lambda: (self.Substitution_MV(0), Print('Finishing state machine'))),
            ('else','.exit'),
            ],
        }

        sm= TStateMachine(states,'start', debug=False)
        sm.Run()

        self.rubbing.Set_interval(25)
        rospy.sleep(0.5)
        avg_angle = 0
        for i in range(60):
            avg_angle += self.get_theta()
            r.sleep()
        avg_angle /= 60
        elapsed_time = time.time() - time_start
        # hyoka = omega_d_i/(time.time()-rotation_start)
        print("elapsed time=", elapsed_time, ", last angle=", avg_angle, ", target=", self.target_angle, ", diff=", avg_angle-self.target_angle, ", error=", error/60, ", error2=", error2/60)
        

        self.is_running = False

def modify_rad_curve(theta):
    k = 0.7267534839
    return np.degrees(np.arctan(k*np.tan(theta)))

def Go2itv_client(data1, data2):
  rospy.wait_for_service('/Go2itv')
  try:
    Go2itv = rospy.ServiceProxy('/Go2itv', Set2Float64)
    Go2itv(data1, data2)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def Print(*args):
  print ' '.join(map(str,args))