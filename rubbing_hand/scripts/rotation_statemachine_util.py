#!/usr/bin/python
#coding: utf-8

from __future__ import print_function
import numpy as np
import threading
import rospy
import time, datetime
from state_machine3 import TStateMachine
import matplotlib.pyplot as plt

import roslib
roslib.load_manifest('rubbing_hand')
from rubbing_hand.srv import *
from rubbing_hand.msg import *

class Log():
    def __init__(self, length = 101):
        self.log = []
        self.length = length

    def storing(self, data):
        self.log.append(data)
        if len(self.log)>self.length: self.log.pop(0)
    
    def get_log(self):
        return self.log

    def clear(self):
        self.log = []

    def clear2(self):
        self.log = [self.log[-1]]

class Rotation:
    def __init__(self, rubbing, sub_fv_filtered1_objinfo, sub_fv_smaf):
        self.is_running= False
        self.rubbing = rubbing
        self.sub_fv_filtered1_objinfo = sub_fv_filtered1_objinfo
        self.sub_fv_smaf = sub_fv_smaf

        #get the angle of object
        self.get_theta = lambda: -self.sub_fv_smaf.req.data[0]
        #get the angle velocity of object
        self.get_omega = lambda: -self.sub_fv_smaf.req.data[1]
        self.get_raw_omega = lambda: np.degrees(self.sub_fv_filtered1_objinfo.req.d_obj_orientation)
        self.get_alpha = lambda: -self.sub_fv_smaf.req.data[4]
        #get the slip of object
        self.get_slip = lambda: sum(self.sub_fv_filtered1_objinfo.req.mv_s)
        self.MV = 0
        self.omega_d = 0

        #角速度のログ取り．最近の平均角速度の算出に使う
        self.log = {}
        self.log["angular_velocity"] = Log(10)
        self.log["angle"] = Log(1000)

        self.remaining_time = 1e6

        self.target_angle = 60.
        self.target_omega = 15.
        # ex. MV_input  = [neutral_min, neutral_max]
        self.MV_i = [-5, 0]
        # ex. MV_output = [open, close]
        self.MV_o = [2.0, -2.]

        self.hz = 60
        # self.tmp_pub = rospy.Publisher(rospy.get_namespace()+"tmp", Float64, queue_size=1)

        self.debug_array = []
        self.process_f = 0

        # publisher設定
        self.rotation_pub = rospy.Publisher("rotation", rotation, queue_size=1)
        self.pub_is_running = True
        self.Start_pub()

        self.margin_time_2stop = 1./60*6

        self.amp = 1.
        self.dec_f = [0, 0]

        self.sin_hz = 5.
        self.d_ratio = 0.1

        self.grasp_itv = 20
        self.angle_margin = 3

        self.sm = None
        self.terminate_f = False

        # close_type,0:parallel, 1: distal, -1: proximal
        self.close_type = 0
        self.deg = {"distal": 5, "proximal": -5, "parallel": 0}
        self.v_deg = 5.

    def Init(self):
        self.dec_f = [0, 0]
        self.log["angular_velocity"].clear()
        self.log["angle"].clear()
        self.remaining_time = 1e6
        self.time_stamp2stop = time.time()
        self.process_f = 0
        self.last_itv = self.grasp_itv
        self.terminate_f = False
        # plt_setup()

    #インハンドマニピュレーションを開始する関数
    #Maniloop関数をスレッドで実行
    def Start(self):
        if not self.is_running:
            self.thread= threading.Thread(name='rotation', target=self.Maniloop)
            self.is_running= True
            self.Init()
            self.thread.start()

    def Stop_run(self):
        if self.is_running:
            if self.sm is not None:
                self.sm.is_running = False
                self.is_running = False

    #すべてのスレッドを終了させる関数
    #プログラムの終了前に呼び出そう
    def Stop(self):
        self.Stop_run()
        if self.pub_is_running:
            self.pub_is_running = False

    #インハンドマニピュレーションでつかったデータをpublishする関数をスレッドで開始
    #initで呼び出してる
    def Start_pub(self):
        pub_thread = threading.Thread(name='rotationmsg', target=self.publish_rotation_data)
        pub_thread.start()

    #インハンドマニピュレーションでつかったデータをpublishする関数
    #スレッドで使うことを想定
    def publish_rotation_data(self):
        r = rospy.Rate(self.hz)
        while self.pub_is_running and not rospy.is_shutdown():
            rotation_msg = self.generate_rotation_msg()
            self.rotation_pub.publish(rotation_msg)
            r.sleep()

    #インハンドマニピュレーションでつかったデータをまとめて、inhand_msgを生成する
    #publish_rotation_data用の関数
    def generate_rotation_msg(self):
        rotation_msg = rotation()

        rotation_msg.header.stamp = rospy.Time.now()
        rotation_msg.interval = self.rubbing.interval
        rotation_msg.MV = self.MV
        rotation_msg.degree_of_finger = self.rubbing.degree_of_finger
        rotation_msg.offset_degree_of_finger = self.rubbing.offset_degree_of_finger
        rotation_msg.MV_degree_of_finger = self.rubbing.MV_degree_of_finger
        rotation_msg.degree_of_surface = self.rubbing.degree_of_surface
        rotation_msg.pos_of_surface = self.rubbing.surface_pos
        rotation_msg.mv_s = self.sub_fv_filtered1_objinfo.req.mv_s
        rotation_msg.obj_orientation = -self.sub_fv_filtered1_objinfo.req.obj_orientation
        rotation_msg.obj_orientation_filtered = self.get_theta()
        rotation_msg.d_obj_orientation_filtered = self.get_omega()
        rotation_msg.target_obj_orientation = self.target_angle
        rotation_msg.target_d_obj_orientation = self.target_omega
        rotation_msg.omega_d = self.omega_d
        rotation_msg.MV_i = self.MV_i
        rotation_msg.MV_o = self.MV_o
        rotation_msg.process_f = self.process_f
        rotation_msg.debag = self.debug_array

        return rotation_msg

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

    #開く速度[mm/sec]を入力値から得る
    #速度変えるたびにコード書き直してノード起動し直すのが面倒だったのでこうした
    def Set_open_step(self):
        input_data = raw_input("input open velocity[mm/s]: ")
        
        self.MV_o[0] = float(input_data)
        print("set: ", self.MV_o[0])

    def calculate_omega_d(self):
        omega_trg = self.target_omega
        omega = abs(self.get_omega())
        omega_d = omega - omega_trg
        self.omega_d = omega_d

        return omega_d

    def Substitution_MV(self, MV):
        self.MV = MV
        
    def Substitution_process(self, f):
        self.process_f = f

    def Substitution_close_type(self, t):
        self.close_type = t

    def break_terminate_f(self):
        self.terminate_f = False

    # open stateのエントリーアクション，振動しながら開く
    def Action_sin_open(self):
        self.MV = self.MV_o[0]
        self.last_itv = self.rubbing.interval
        if self.dec_f[1]:
            self.MV = 0
        self.dec_f = [0, 0]
        # print(self.MV, self.amp, self.sin_hz, 1., self.d_ratio)
        self.rubbing.Pulse_deformed(self.MV, self.amp, self.sin_hz, 1., self.d_ratio)

    def Action_linear_open(self):
        self.MV = self.MV_o[0]
        self.last_itv = self.rubbing.interval
        self.rubbing.Go2itv(self.rubbing.interval+10, self.MV)
    
    def Decretion_amp(self):
        d_omega = self.calculate_omega_d()

        if d_omega <= self.MV_i[0]:
            pass
        elif self.MV_i[0] < d_omega <= self.MV_i[1]:
            self.dec_f[0] = 1
        elif self.MV_i[1] < d_omega:
            self.dec_f[1] = 1

    def Process_always(self):
        # ------------------------------------------------------------------
        self.log["angle"].storing(self.get_theta())
        y = self.log["angle"].get_log()
        if len(y) > 10:
            if all(abs(np.array(y[-5:])) < self.angle_margin):
                # print(np.array(y[-60:]) < self.angle_margin)
                self.terminate_f = True
        # ------------------------------------------------------------------
        # theta = self.get_theta()
        # if self.close_type == 1:
        #     set_degree = min(theta/30*self.deg["distal"],self.deg["distal"])
        # elif self.close_type == -1:
        #     set_degree = max(-theta/30*self.deg["proximal"],self.deg["proximal"])
        #     print(set_degree)
        # else:
        #     set_degree = 0
        # self.rubbing.Set_degree(set_degree)

    def Maniloop(self):
        states_sin= {
            'start': [
            ('entry',lambda: (self.Substitution_MV(0), self.Substitution_process(2), Print('start rotation manipulation'))),
            ('else','judge_type'),
            ],
            'judge_type': [
            ('entry', lambda: ((self.rubbing.Go2deg(self.deg["distal"], self.v_deg), self.Substitution_close_type(1)) if (self.get_theta() >= 0) else (self.rubbing.Go2deg(self.deg["proximal"], self.v_deg), self.Substitution_close_type(-1)))),
            (lambda: not self.rubbing.go2deg_f,'judge'),
            ('else','judge_type'),
            ],
            'judge': [
            ('entry',lambda: self.calculate_omega_d()),
            (lambda: self.omega_d <= self.MV_i[0],'open'),
            (lambda: self.MV_i[1] < self.omega_d,'close'),
            (lambda: self.MV_i[0] < self.omega_d <= self.MV_i[1],'stay'),
            ('else','judge',lambda: Print('judge failed, omega_d:'+str(self.omega_d))),
            ],
            'open': [
            ('entry',lambda: self.Action_sin_open()),
            (lambda: not self.rubbing.go2itv_f,'judge'),
            ('else','open', lambda: self.Decretion_amp()),
            ],
            'close': [
            ('entry',lambda: (self.Substitution_MV(self.MV_o[1]),self.rubbing.Set_interval(self.rubbing.interval), self.rubbing.Go2itv(self.rubbing.interval-10, self.MV))),
            (lambda: not self.MV_i[1] < self.calculate_omega_d(),'judge'),
            (lambda: not self.rubbing.go2itv_f,'close'),
            ('else','close'),
            ],
            'stay': [
            ('entry',lambda: self.Substitution_MV(0)),
            (lambda: not self.MV_i[0] < self.calculate_omega_d() <= self.MV_i[1],'judge'),
            ('else','stay'),
            ],
            'finish': [
            ('entry',lambda: (Print('Finishing state machine'))),
            ('else','.exit'),
            ],
            'stop': [
            ('entry',lambda: (self.Substitution_MV(0), self.Substitution_process(0), self.rubbing.Set_interval(self.grasp_itv), self.rubbing.Set_degree(0), self.Substitution_close_type(0), Print('stop object rotation'), rospy.sleep(0.5))),
            (lambda: not abs(self.get_theta()) <= self.angle_margin, "judge_type"),
            ('else','finish'),
            ],
            'always': [
            ('deny',["start", "finish", "stop",'judge_type']),
            ("process", lambda: self.Process_always()),
            (lambda: self.get_theta() >= 0 if self.close_type==-1 else (self.get_theta() < 0 if self.close_type==1 else False), "judge_type"),
            (lambda: self.terminate_f, 'stop', lambda: (Print('Held vertical for 1 sec.'), self.break_terminate_f())),
            ]
        }

        self.sm= TStateMachine(states_sin,'start', debug=False)
        # self.Set_open_step()


        time_start = time.time()
        r = rospy.Rate(self.hz)

        #初期の指間距離保存
        self.grasp_itv = self.rubbing.interval

        #get the initial angle
        avg_angle = 0
        for i in range(60):
            avg_angle += self.get_theta()
            r.sleep()
        avg_angle /= 60
        theta0 = avg_angle

        print("initial angle: {}, open velocity: {}, frequency: {}, amplitude: {}, duty ratio: {}".format(theta0, self.MV_o[0], self.sin_hz, self.amp, self.d_ratio))


        self.process_f = 2
        ret = self.sm.Run()

        if ret: # 外部から強制終了
            self.process_f = 3
            self.rubbing.Set_interval(self.grasp_itv)
            print("強制終了しました")
        else:   # 正常終了
            self.process_f = 1
            avg_angle = 0
            for i in range(60):
                avg_angle += self.get_theta()
                r.sleep()
            self.process_f = 0
            avg_angle /= 60
            elapsed_time = time.time() - time_start
            print("elapsed time=", elapsed_time, ", last angle=", avg_angle, ", target=", self.target_angle, ", diff=", avg_angle-self.target_angle)
        

        self.is_running = False

def Print(*args):
  print(' '.join(map(str,args)))