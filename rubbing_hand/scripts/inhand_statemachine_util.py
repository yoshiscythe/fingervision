#!/usr/bin/python
#coding: utf-8

from __future__ import print_function
import numpy as np
import threading
import rospy
from rub import PID
import time, datetime
from state_machine3 import TStateMachine
import matplotlib.pyplot as plt

import roslib
roslib.load_manifest('rubbing_hand')
from rubbing_hand.srv import *
from rubbing_hand.msg import *


def plt_setup():
    global fig, ax

    # plt.switch_backend('agg')

    # 描画用
    fig, ax = plt.subplots()
    fig.canvas.draw()
    fig.show()
    
    
    return 0

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
        # self.min_gstep = 0.01
        self.th_slip = 0.0001
        self.target_omega = 15.
        # ex. MV_input  = [neutral_min, neutral_max]
        self.MV_i = [-5, 0]
        # ex. MV_output = [open, close]
        self.MV_o = [0.5, -2.]

        self.hz = 60
        # self.tmp_pub = rospy.Publisher(rospy.get_namespace()+"tmp", Float64, queue_size=1)

        self.debug_array = []
        self.process_f = 0

        # self.rubbing.itv_min=5

        # publisher設定
        self.inhand_pub = rospy.Publisher("inhand", inhand, queue_size=1)
        self.pub_is_running = True
        self.Start_pub()

        self.margin_time_2stop = 1./60*6

        self.amp_first = 3.
        self.dec_f = [0, 0]
        self.amp_dec = 0.

        self.sin_hz = 5.
        self.d_ratio = 0.1

        self.grasp_itv = 20
        self.angle_margin = 3

        self.sm = None

    def Init(self):
        self.amp = self.amp_first
        self.dec_f = [0, 0]
        self.log["angular_velocity"].clear()
        self.log["angle"].clear()
        self.remaining_time = 1e6
        self.time_stamp2stop = time.time()
        self.process_f = 0
        self.last_itv = self.grasp_itv
        # plt_setup()

    #インハンドマニピュレーションを開始する関数
    #Maniloop関数をスレッドで実行
    def Start(self):
        if not self.is_running:
            self.thread= threading.Thread(name='inhand', target=self.Maniloop)
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
        pub_thread = threading.Thread(name='inhand_msg', target=self.publish_inhand_data)
        pub_thread.start()

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
        inhand_msg.process_f = self.process_f
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

    #開く速度[mm/sec]を入力値から得る
    #速度変えるたびにコード書き直してノード起動し直すのが面倒だったのでこうした
    def Set_open_step(self):
        input_data = raw_input("input open velocity[mm/s]: ")
        
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
        
    def Substitution_process(self, f):
        self.process_f = f
        
    def Decretion_amp(self):
        d_omega = self.calculate_omega_d()

        if d_omega <= self.MV_i[0]:
            pass
        elif self.MV_i[0] < d_omega <= self.MV_i[1]:
            self.dec_f[0] = 1
        elif self.MV_i[1] < d_omega:
            self.dec_f[1] = 1

    # open stateのエントリーアクション，振動しながら開く
    def Action_sin_open(self):
        self.MV = self.MV_o[0]
        self.last_itv = self.rubbing.interval
        if self.dec_f:
            if self.dec_f[1]:
                self.amp = max(self.amp - self.amp_dec, 0)
                self.MV = 0
            else:
                self.amp = min(self.amp + self.amp_dec, self.amp_first)
        self.dec_f = [0, 0]
        # print(self.MV, self.amp, self.sin_hz, 1., self.d_ratio)
        self.rubbing.Pulse_deformed(self.MV, self.amp, self.sin_hz, 1., self.d_ratio)

    def Action_linear_open(self):
        self.MV = self.MV_o[0]
        self.last_itv = self.rubbing.interval
        self.rubbing.Go2itv(self.rubbing.interval+10, self.MV)

    def Process_always(self):
        # ------------------------------------------------------------------
        self.log["angle"].storing(self.get_theta())
        if self.calculate_omega_d() > self.MV_i[1]:
            estimated_angle_, remaining_steps_ = self.ApproximateData(self.log["angle"], False)
            remaining_time_ = remaining_steps_/self.hz
        else:
            self.log["angle"].clear2()
            estimated_angle_, remaining_time_ = 0, 1e6
        mean_angular_velocity_=0
        # ------------------------------------------------------------------

        # # ------------------------------------------------------------------
        # mean_angular_velocity_ = self.get_omega()
        # mean_angular_velocity_ = max(1e-3, mean_angular_velocity_)
        # angle = self.get_theta()
        # remaining_time_ = (self.target_angle-angle)/mean_angular_velocity_
        # estimated_angle_ = self.get_theta() + mean_angular_velocity_*self.margin_time_2stop
        # # ------------------------------------------------------------------

        self.remaining_time = remaining_time_
        self.estimated_angle = estimated_angle_
        debug_estimated_angle = estimated_angle_
        mean_angular_velocity = mean_angular_velocity_
        self.debug_array = [mean_angular_velocity, debug_estimated_angle]
        # print(mean_angular_velocity, self.remaining_time)

    def ApproximateData(self, y_log, plotGraph = False):
        y = np.array(y_log.get_log())
        if len(y) < 3:
            return 0, 1e6
        
        # x_array = [-len(y_array), -len(y_array)+1, ... , 0]
        x = np.flip(np.arange(len(y))*-1)
        x_over = np.append(x, np.arange(1,6))

        # print(x,y)

        #近似式の係数
        res1=np.polyfit(x, y, 1)
        res2=np.polyfit(x, y, 2)
        # res3=np.polyfit(x, y, 3)
        #近似式の計算
        y1 = np.poly1d(res1)(x_over) #1次
        y2 = np.poly1d(res2)(x_over) #2次
        # y3 = np.poly1d(res3)(x_over) #3次

        p = np.poly1d(res2)
        y_trg = self.target_angle
        x_trg = max((p - y_trg).roots)
        if isinstance(x_trg, complex):
            x_trg = 1e6

        # if x_trg/self.hz < self.margin_time_2stop:
        #     plotGraph = True
        
        if plotGraph:
            # global fig, ax
            # # ax.draw_artist(ax.patch)
            # ax.clear()
            # org =  ax.scatter(x, y, label='org data')
            # ax.plot(x_over, y1, label='1st order')
            # ax.plot(x_over, y2, label='2nd order')
            # ax.plot(x_over, y3, label='3rd order')
            # ax.legend()
            # ax.draw_artist(ax)
            # fig.canvas.draw()
            # # fig.canvas.update()
            # # fig.canvas.blit(ax.bbox)
            # fig.canvas.flush_events()


            plt.clf()
            plt.scatter(x, y, label='org data')
            plt.plot(x_over, y1, label='1st order')
            plt.plot(x_over, y2, label='2nd order')
            # plt.plot(x_over, y3, label='3rd order')
            plt.legend()
            data_directory = "/home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/data/0718"
            plt.savefig(data_directory+"/CAVS/estimate/CAVS"+datetime.datetime.today().strftime("%Y-%m-%d-%H-%M-%S")+"_estimate.png")
            plt.clf()
            
        # print(y1[-1], y2[-1], y3[-1])
        
        return y2[-1], x_trg

    def Maniloop(self):
        # 操作：指を一定の速度で開く.物体が回転したら（目標角速度を超えたら）指を安定把持まで一瞬で閉じる．
        # 目的：指をきゅっと閉めてからどのくらいで物体の回転が止まるか調べるのだ．\Delta t(v) を調べるのじゃ．
        states_dtdetector= {
            'start': [
            ('entry',lambda: (self.Substitution_MV(0), self.Substitution_process(2), Print('start inhand manipulation'))),
            ('else','open'),
            ],
            'continue': [
            ('else','open'),
            ],
            'open': [
            ('entry',lambda: self.Action_linear_open()),
            (lambda: not self.rubbing.go2itv_f,'continue'),
            ('else','open'),
            ],
            'finish': [
            ('entry',lambda: (Print('Finishing state machine'))),
            ('else','.exit'),
            ],
            'stop': [
            ('entry',lambda: (self.Substitution_MV(0), self.Substitution_process(0), self.rubbing.Set_interval(self.grasp_itv), Print('stop object rotation'), rospy.sleep(0.5))),
            ('else','finish'),
            ],
            'always': [
            ('deny',["start", "finish", "stop"]),
            (lambda: self.get_omega()>self.target_omega,'stop',lambda: Print('over target omega! in always state')),
            ]
        }

        states_sin= {
            'start': [
            ('entry',lambda: (self.Substitution_MV(0), self.Substitution_process(2), Print('start inhand manipulation'))),
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
            ('entry',lambda: self.Action_sin_open()),
            (lambda: not self.rubbing.go2itv_f,'judge'),
            ('else','open', lambda: self.Decretion_amp()),
            ],
            'close': [
            ('entry',lambda: (self.Substitution_MV(self.MV_o[1]),self.rubbing.Set_interval(self.rubbing.interval), self.rubbing.Go2itv(self.rubbing.interval-10, self.MV))),
            (lambda: not self.MV_i[1] < self.calculate_omega_d(),'judge'),
            (lambda: not self.rubbing.go2itv_f,'wait'),
            ('else','close'),
            ],
            'wait': [
            ('entry',lambda:(self.Substitution_MV(0), GetStartTime())),
            (lambda: self.calculate_omega_d() >= self.MV_i[1],'judge'),
            (lambda: (int(time.time())-start_time)>=2,'judge'),
            ('else','wait'),
            ],
            'stay': [
            ('entry',lambda: self.Substitution_MV(0)),
            (lambda: not self.MV_i[0] < self.calculate_omega_d() <= self.MV_i[1],'judge'),
            ('else','stay'),
            ],
            'finish': [
            ('entry',lambda: (Print('Finishing state machine'))),
            (lambda: self.target_angle - self.get_theta() > self.angle_margin,'judge',lambda: (Print('remain {:.2f}, restart!'.format(self.target_angle - self.get_theta())), self.Substitution_process(2), self.rubbing.Set_interval(self.last_itv))),
            ('else','.exit'),
            ],
            'stop': [
            ('entry',lambda: (self.Substitution_MV(0), self.Substitution_process(0), self.rubbing.Set_interval(self.grasp_itv), Print('stop object rotation'), rospy.sleep(0.5))),
            ('else','finish'),
            ],
            'always': [
            ('deny',["start", "finish", "stop"]),
            ("process", lambda: self.Process_always()),
            (lambda: self.margin_time_2stop > self.remaining_time,'stop',lambda: Print('over target theta is estimated!')),
            (lambda: self.get_theta()>self.target_angle,'stop',lambda: Print('over target theta! in always state')),
            ]
        }

        states_linear= {
            'start': [
            ('entry',lambda: (self.Substitution_MV(0), self.Substitution_process(2), Print('start inhand manipulation'))),
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
            ('entry',lambda: self.Action_linear_open()),
            (lambda: not self.MV_i[0] >= self.calculate_omega_d(),'judge'),
            (lambda: not self.rubbing.go2itv_f,'judge'),
            ('else','open'),
            ],
            'close': [
            ('entry',lambda: (self.Substitution_MV(self.MV_o[1]),self.rubbing.Set_interval(self.rubbing.interval), self.rubbing.Go2itv(self.rubbing.interval-10, self.MV))),
            (lambda: not self.MV_i[1] < self.calculate_omega_d(),'judge'),
            (lambda: not self.rubbing.go2itv_f,'judge'),
            ('else','close'),
            ],
            'stay': [
            ('entry',lambda: self.Substitution_MV(0)),
            (lambda: not self.MV_i[0] < self.calculate_omega_d() <= self.MV_i[1],'judge'),
            ('else','stay'),
            ],
            'finish': [
            ('entry',lambda: (Print('Finishing state machine'))),
            (lambda: self.target_angle - self.get_theta() > self.angle_margin,'judge',lambda: (Print('remain {:.2f}, restart!'.format(self.target_angle - self.get_theta())), self.Substitution_process(2), self.rubbing.Set_interval(self.last_itv))),
            ('else','.exit'),
            ],
            'stop': [
            ('entry',lambda: (self.Substitution_MV(0), self.Substitution_process(0), self.rubbing.Set_interval(self.grasp_itv), Print('stop object rotation'), rospy.sleep(0.5))),
            ('else','finish'),
            ],
            'always': [
            ('deny',["start", "finish", "stop"]),
            ("process", lambda: self.Process_always()),
            (lambda: self.margin_time_2stop > self.remaining_time,'stop',lambda: Print('over target theta is estimated!')),
            (lambda: self.get_theta()>self.target_angle,'stop',lambda: Print('over target theta! in always state')),
            ]
        }

        states_sin_over= {
            'start': [
            ('entry',lambda: (self.Substitution_MV(0), self.Substitution_process(2), Print('start inhand manipulation'))),
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
            ('entry',lambda: self.Action_sin_open()),
            (lambda: not self.rubbing.go2itv_f,'judge'),
            ('else','open', lambda: self.Decretion_amp()),
            ],
            'close': [
            ('entry',lambda: (self.Substitution_MV(self.MV_o[1]),self.rubbing.Set_interval(self.rubbing.interval), self.rubbing.Go2itv(self.rubbing.interval-10, self.MV))),
            (lambda: not self.MV_i[1] < self.calculate_omega_d(),'judge'),
            (lambda: not self.rubbing.go2itv_f,'wait'),
            ('else','close'),
            ],
            'wait': [
            ('entry',lambda:(self.Substitution_MV(0), GetStartTime())),
            (lambda: self.calculate_omega_d() >= self.MV_i[1],'judge'),
            (lambda: (int(time.time())-start_time)>=2,'judge'),
            ('else','wait'),
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
            ('entry',lambda: (self.Substitution_MV(0), self.Substitution_process(0), self.rubbing.Set_interval(self.grasp_itv), Print('stop object rotation'), rospy.sleep(0.5))),
            ('else','finish'),
            ],
            'always': [
            ('deny',["start", "finish", "stop"]),
            ("process", None),
            (lambda: self.get_theta()>self.target_angle,'stop',lambda: Print('over target theta! in always state')),
            ]
        }
        self.sm= TStateMachine(states_linear,'start', debug=False)
        self.Set_open_step()


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

        thread_cond = lambda: self.is_running and not rospy.is_shutdown()

        print("initial angle: {}, open velocity: {}, frequency: {}, amplitude: {}, duty ratio: {}".format(theta0, self.MV_o[0], self.sin_hz, self.amp_first, self.d_ratio))

        last_omega_d = 0
        rotation_f = False
        error2 = 0
        error = 0

        def GetStartTime():
            global start_time
            start_time= int(time.time())

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
            # hyoka = omega_d_i/(time.time()-rotation_start)
            print("elapsed time=", elapsed_time, ", last angle=", avg_angle, ", target=", self.target_angle, ", diff=", avg_angle-self.target_angle)
        

        self.is_running = False

def Go2itv_client(data1, data2):
  rospy.wait_for_service('/Go2itv')
  try:
    Go2itv = rospy.ServiceProxy('/Go2itv', Set2Float64)
    Go2itv(data1, data2)
  except rospy.ServiceException, e:
    print("Service call failed: {}".format(e))

def Print(*args):
  print(' '.join(map(str,args)))