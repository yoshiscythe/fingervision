Header header

# --------dynamixel--------

# ダイナミクセルへ渡した目標指間距離
# 取得した指間距離へ操作量を加えている
# interval = got_interval + MV
float64 interval

# 指間距離へ加える操作量(Manipulated Variable)
float64 MV

# -------------------------

# --------fingervision-----

# Array of slips (slip distribution), which is a serialized list of 3x3 matrix.
# Each cell in the 3x3 matrix is the sum of moving pixels in the cell.
float32[] mv_s

# objectの角度 [deg]
float64 obj_orientation

# objectの角度 [deg]  obj_orientationをsmaでフィルターしてる（filter_node参照）
float64 obj_orientation_filtered

# objectの角速度 [deg/s]　obj_orientation_filteredの差分をとったものをsmaでフィルターしてる（filter_node参照）
float64 d_obj_orientation_filtered

# -------------------------

# ---------inhand----------

# 目標角度
float64 target_obj_orientation

# 目標角速度
float64 target_d_obj_orientation

# 目標角速度と取得した角速度の差
# d_obj_orientation_filtered - target_d_obj_orientation
float64 omega_d

# mv_sの和からすべり判定をする際のしきい値
float64 th_slip

# 取得した角速度d_obj_orientation_filteredと目標角速度target_d_obj_orientationとの差から操作量MVを決めるパラメータ
# MV_input  = [neutral_min, neutral_max , drop]
# MV_output = [open, close, quick_close]
# drop < d_obj_orientation_filtered : quick_close
# d_omega <= neutral_min : open
# neutral_min < d_omega <= neutral_max : 0
# neutral_max < d_omega : close
float64[] MV_i
float64[] MV_o

# マニピュレーション実行区間を表すフラグ
# 0: してない， 1:マニピュレーション終了後の数秒間， 2:マニピュレーション中
int32 process_f

# -------------------------

# なんでも入れていいよ
float64[] debag