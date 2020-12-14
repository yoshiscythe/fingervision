#!/usr/bin/python
#coding: utf-8

import numpy as np
import yaml

class Rubbing():
    def __init__(self):
        #根本関節間の距離
        self.finger_interval = 65.0
        #根本リンクの長さ
        self.link_length = 63.0
        #指先関節から指中心(カメラ？)までの距離joint to finger(笑)
        self.distance_j2f = 32.0
        #仮想面の傾き(degree)
        self.degree_of_surface = 0

        #二指の中間面の中心からの移動
        self.surface_pos = 0
        #二指間の距離
        self.interval = 30.0

        self.deg_per_pos = 360.0/4096
        self.pos_per_deg = 4096.0/360

        self.filename = ""
        self.init_pos_r = 0
        self.init_pos_l = 0
        self.init_pos_r_dist = 0
        self.init_pos_l_dist = 0
        self.offset_r_dist = 0
        self.offset_l_dist = 0

        self.running = 0

    #条件下での各指の根本角度を計算する
    def calculation_degree(self):
        #右指の根本角度、開く方向を正
        right_finger_deg = np.rad2deg(np.arccos((self.finger_interval/2-self.surface_pos-self.interval/2)/self.link_length))
        #左指の根本角度、開く方向を正
        left_finger_deg = np.rad2deg(np.arccos((self.finger_interval/2+self.surface_pos-self.interval/2)/self.link_length))
        return right_finger_deg, left_finger_deg

    def pos2deg(self, now_pos_r, now_pos_l):
        abs_pos_r = self.init_pos_r - now_pos_r
        abs_deg_r = abs_pos_r*self.deg_per_pos + 90
        abs_pos_l = now_pos_l - self.init_pos_l
        abs_deg_l = abs_pos_l*self.deg_per_pos + 90
        return abs_deg_r, abs_deg_l

    def deg2pos(self, deg_r, deg_l):
        pos_r = ( 90 - deg_r)*self.pos_per_deg + self.init_pos_r
        pos_l = (-90 + deg_l)*self.pos_per_deg + self.init_pos_l
        return pos_r, pos_l

    def calculation_proportional(self, now_pos_r, now_pos_l):
        right_finger_deg, left_finger_deg = self.calculation_degree()
        abs_deg_r, abs_deg_l = self.pos2deg(now_pos_r, now_pos_l)

        pro_r = right_finger_deg - abs_deg_r
        pro_l = left_finger_deg - abs_deg_l
        return pro_r, pro_l

    def range_check(self):
        range_max = int(-self.finger_interval/2 + self.interval/2 + self.link_length)
        if self.surface_pos > range_max:
            self.surface_pos = range_max
        elif self.surface_pos < -range_max:
            self.surface_pos = -range_max

    def calculation_pos_dist(self):
        pos_r_dist = self.init_pos_r_dist + self.offset_r_dist
        pos_l_dist = self.init_pos_l_dist + self.offset_l_dist

        return pos_r_dist, pos_l_dist

    def read_initial_position(self):
        with open(self.filename) as file:
          obj = yaml.safe_load(file)
          self.init_pos_r = obj['init_pos_r']
          self.init_pos_l = obj['init_pos_l']
          self.init_pos_r_dist = obj['init_pos_r_dist']
          self.init_pos_l_dist = obj['init_pos_l_dist']

    def write_initial_position(self, clb_init_pos_l, clb_init_pos_r, clb_init_pos_l_dist, clb_init_pos_r_dist):
        obj = { 'init_pos_r': clb_init_pos_r,
                'init_pos_l': clb_init_pos_l,
                'init_pos_r_dist': clb_init_pos_r_dist,
                'init_pos_l_dist': clb_init_pos_l_dist}
        with open(self.filename, 'w') as file:
            yaml.dump(obj, file)

class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.targetPos=0.
        self.clear()

    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.delta_time = 0.1
        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0
        self.output = 0.0

    def update(self, feedback_value):
        error = self.targetPos - feedback_value
        delta_error = error - self.last_error  
        self.PTerm = self.Kp * error  #PTermを計算
        self.ITerm += error * self.delta_time  #ITermを計算

        if (self.ITerm > self.windup_guard):  #ITermが大きくなりすぎたとき様
            self.ITerm = self.windup_guard
        if(self.ITerm < -self.windup_guard):
           self.ITerm = -self.windup_guard
           
        self.DTerm = delta_error / self.delta_time  #DTermを計算
        self.last_error = error
        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
    
    def setTargetPosition(self, targetPos):
        self.targetPos = targetPos

# test=Rubbing()
# test.init_pos_l = 200
# test.init_pos_r = 3300
# print test.calculation_degree()
# print test.calculation_proportional(3300, 200)
# print test.calculation_proportional(3100, 600)
# print test.pos2deg(3300, 200)
# print test.pos2deg(3100, 600)