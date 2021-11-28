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

        #二指の中間面の中心からの移動
        self.surface_pos = 0
        #二指間の距離
        self.interval = 20.
        #仮想面の傾き(degree)
        self.degree_of_surface = 0.0
        #指先の傾き
        self.degree_of_finger = 0.0

        self.deg_per_pos = 360.0/4096
        self.pos_per_deg = 4096.0/360

        self.pos_per_itv = -535./86.7

        self.filename = ""
        self.init_pos_r = 0
        self.init_pos_l = 0
        self.init_pos_r_dist = 0
        self.init_pos_l_dist = 0
        self.offset_r_dist = 0
        self.offset_l_dist = 0
        self.go2itv_f = 0

        self.itv_max = 180
        self.itv_min = 16

        self.running = 0
        self.control_f = False

        self.holding = None
        self.last_dynamixel_hz = None

    def get_current_dynamixel_hz(self):
        value = self.holding.topic_hz.get_hz("/dynamixel_data")

        if value is None:
            return self.last_dynamixel_hz
        
        value = value[0]
        self.last_dynamixel_hz = value

        return value

    #条件下での各指の根本角度を計算する
    def calculation_degree(self):
        theta = np.deg2rad(self.degree_of_surface)
        phi = np.deg2rad(self.degree_of_finger)
        #右指の根本角度、開く方向を正
        right_finger_deg = np.rad2deg(np.arccos((self.finger_interval/2-self.surface_pos-(self.interval/2)*np.cos(theta)+self.distance_j2f*np.sin(theta-phi))/self.link_length))
        #左指の根本角度、開く方向を正
        left_finger_deg = np.rad2deg(np.arccos((self.finger_interval/2+self.surface_pos-(self.interval/2)*np.cos(theta)-self.distance_j2f*np.sin(theta+phi))/self.link_length))
        return right_finger_deg, left_finger_deg

    #エンコーダのモータポジションから指間距離を計算
    def calculation_interval_from_pos(self, now_pos_r, now_pos_l, pos_r_dist, pos_l_dist):
        phi_r, phi_l = np.deg2rad(self.calculation_deg_from_pos_dist(pos_r_dist, pos_l_dist))
        deg_r, deg_l = self.pos2deg(now_pos_r, now_pos_l)
        deg_r, deg_l = np.deg2rad(deg_r), np.deg2rad(deg_l)
        interval_encode = self.finger_interval - self.link_length*np.cos(deg_r) - self.link_length*np.cos(deg_l) - self.distance_j2f*np.sin(phi_r) - self.distance_j2f*np.sin(phi_l)
        return interval_encode

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

    # def calculation_proportional(self, now_pos_r, now_pos_l):
    #     right_finger_deg, left_finger_deg = self.calculation_degree()
    #     abs_deg_r, abs_deg_l = self.pos2deg(now_pos_r, now_pos_l)

    #     pro_r = right_finger_deg - abs_deg_r
    #     pro_l = left_finger_deg - abs_deg_l
    #     return pro_r, pro_l

    def itv2pos_rhp12rn(self, itv):
        pos = itv*self.pos_per_itv + self.init_pos
        return pos

    def range_check(self):
        range_max = int(-self.finger_interval/2 + self.interval/2 + self.link_length)
        if self.surface_pos > range_max:
            self.surface_pos = range_max
        elif self.surface_pos < -range_max:
            self.surface_pos = -range_max

        itv_max = self.itv_max
        itv_min = self.itv_min
        if self.interval > itv_max:
            self.interval = itv_max
        elif self.interval < itv_min:
            self.interval = itv_min

    def calculation_pos_dist(self):
        theta = self.degree_of_surface
        phi = self.degree_of_finger

        pos_r_dist = self.init_pos_r_dist + (theta-phi)*self.pos_per_deg
        pos_l_dist = self.init_pos_l_dist + (theta+phi)*self.pos_per_deg

        return pos_r_dist, pos_l_dist
    
    # theta=0 とする
    def calculation_deg_from_pos_dist(self,pos_r_dist,pos_l_dist):

        phi_r  = (self.init_pos_r_dist - pos_r_dist)*self.deg_per_pos
        phi_l  = -(self.init_pos_l_dist - pos_l_dist)*self.deg_per_pos

        return phi_r, phi_l

    def read_initial_position(self):
        with open(self.filename) as file:
          obj = yaml.safe_load(file)
          self.init_pos_r = obj['init_pos_r']
          self.init_pos_l = obj['init_pos_l']
          self.init_pos_r_dist = obj['init_pos_r_dist']
          self.init_pos_l_dist = obj['init_pos_l_dist']
        
    def read_initial_position_rhp12rn(self):
        with open(self.filename) as file:
          obj = yaml.safe_load(file)
          self.init_pos = obj['init_pos']

    def write_initial_position(self, clb_init_pos_l, clb_init_pos_r, clb_init_pos_l_dist, clb_init_pos_r_dist):
        obj = { 'init_pos_r': clb_init_pos_r,
                'init_pos_l': clb_init_pos_l,
                'init_pos_r_dist': clb_init_pos_r_dist,
                'init_pos_l_dist': clb_init_pos_l_dist}
        with open(self.filename, 'w') as file:
            yaml.dump(obj, file)

    def write_initial_position_rhp12rn(self, clb_init_pos):
        obj = { 'init_pos': clb_init_pos}
        with open(self.filename, 'w') as file:
            yaml.dump(obj, file)

    # 目標インターバルの変更
    # インターバルが変更されると操作済みフラグを立てる。
    # Update()で毎ステップの終わりに折る
    def Set_interval(self, data):
        max_itv = self.itv_max
        min_itv = self.itv_min
        
        if data > max_itv:
            data = max_itv
        if data < min_itv:
            data = min_itv
        self.interval = data
        self.control_f = True
        self.go2itv_f = False

        return True

    # Go2itv()用の目標距離アップデート
    # go2itv_arrayの先頭から順に目標値へセット
    # Set_interval()でインターバルが変更されてるとgo2itvは終了
    def update_interval(self):
        if self.go2itv_f:
            # 指定した位置の要素を削除し、その値を取得
            set_data, self.go2itv_array = self.go2itv_array[0], self.go2itv_array[1:]

            # 番兵もしくはインターバルの変更を感知すると終了
            if set_data == -1 or self.control_f:
                self.go2itv_f = 0
                return

            self.interval = set_data

    def update_degfinger(self):
        # # --------------------------------------------------
        # # CAVS
        # min_deg = 1.
        # max_deg = 8.0
        # self.degree_of_finger = self.interval*(-0.44) + 18.2
        # # --------------------------------------------------

        # # --------------------------------------------------
        # # CAVS thick
        # min_deg = 0.
        # max_deg = 6.0
        # self.degree_of_finger = self.interval*(-0.44) + 20.2
        # # --------------------------------------------------

        # -----------------------------------------------------
        # FLAT
        min_deg = 1
        max_deg = 7
        self.degree_of_finger = self.interval*(-0.585) + 18.2
        # -----------------------------------------------------

        # # -----------------------------------------------------
        # # FLAT thick
        # min_deg = 2
        # max_deg = 11
        # self.degree_of_finger = self.interval*(-0.434) + 18.8
        # # -----------------------------------------------------

        if self.degree_of_finger > max_deg:
            self.degree_of_finger = max_deg
        if self.degree_of_finger < min_deg:
            self.degree_of_finger = min_deg

    def Update(self):
        self.update_interval()
        self.range_check()
        self.update_degfinger()
        self.control_f = False


    # 現在の指間距離から入力した指間距離itv_goalまで、入力された速度runvel[mm/sec]で動かす
    # 指間距離のアレイを生成してupdate_interval()によって順に動かす
    # itvはintervalの略のつもり
    def Go2itv(self, itv_goal, runvel= 0.01):
        hz = self.get_current_dynamixel_hz()
        itv_start = self.interval
        if itv_start > itv_goal:
            runvel = -abs(runvel)
        runstep = runvel/float(hz)
        self.go2itv_array = np.arange(itv_start, itv_goal, runstep)
        self.go2itv_array = np.append(self.go2itv_array, [itv_goal, -1])
        self.go2itv_array = self.go2itv_array[1:]
        self.go2itv_f = 1

        return True

    # 指定回数(num)だけ振動しながら開く
    # 振動の，前半（開く）と後半（閉じる）の比を変更可能．
    # 線形関数とsin関数の合成で軌道生成
    # 線形パラ：runvel 傾き[mm/s]
    # sinパラ： A 振幅(入力は波の下限から上限までの振幅．関数内では2で割って通常の振幅として使ってる)[mm] f 周波数[/s] d_ratio 全体に対して後半の比(0~1)
    # d_ratio:
    # 0.5で普通の振動．f=1[s]の場合，d_ratio=0.3にすると0.7秒で開いて0.3秒で閉じる感じ．
    # 指間距離のアレイを生成してupdate_interval()によって順に動かす
    # itvはintervalの略のつもり
    def Pulse_deformed(self, runvel = 0.01, A = 0.1, f = 5, num = 1, d_ratio = 0.5):
        A = A/2.
        itv_start = self.interval
        hz = float(self.get_current_dynamixel_hz())
        # print(hz)
        itv_goal = itv_start + (runvel/f)*num
        rad_from_t = lambda t: np.pi*f*(t%(1./f))/(1-d_ratio) if (t%(1./f))*f < 1- d_ratio else np.pi*f*(t%(1./f))/d_ratio + (2 - 1./d_ratio)*np.pi
        run_time = float(num)/f
        # print(A)
        time = np.arange(0, run_time, 1/hz)

        if itv_start > itv_goal:
            linear_sign = -1
        else:
            linear_sign = 1
        traj_linear = [runvel*t for t in time]
        traj_sin_angle = [rad_from_t(t) for t in time]
        traj_sin = -A*np.cos(traj_sin_angle) + A
        go2itv_array = traj_linear + traj_sin + itv_start
        go2itv_array= np.append(go2itv_array, [itv_goal, -1])
        self.go2itv_array = go2itv_array[1:]
        self.go2itv_f = 1

        # print(go2itv_array)

        return True

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
        # if abs(delta_error) < 1.: delta_error = 0
        self.PTerm = self.Kp * error  #PTermを計算
        self.ITerm += error * self.delta_time  #ITermを計算

        if (self.ITerm > self.windup_guard):  #ITermが大きくなりすぎたとき様
            self.ITerm = self.windup_guard
        if(self.ITerm < -self.windup_guard):
           self.ITerm = -self.windup_guard
           
        self.DTerm = delta_error / self.delta_time  #DTermを計算

        # if self.DTerm>0: self.DTerm /= 10

        self.last_error = error
        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

        return self.output
    
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