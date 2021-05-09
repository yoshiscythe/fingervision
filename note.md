# 研究ノート

## How to use rubbing_hand

- FingerVision起動  
    1.  mjpgstreamer起動    
        ~~~sh
        $ ./mjpg_streamer -i "./input_uvc.so -f -1 -r 320x240 -d /dev/video4 -n" -o "./output_http.so -w ./www -p 8080"
        ~~~
        ダイレクトリンク    
        http://localhost:8080/?action=stream&dummy=file.mjpg

    2.  FingerVisionのローンチファイル実行  
        ~~~sh
        $ roslaunch rubbing_hand fv_1_filtered.launch 
        ~~~

    3.  フィルターノード起動
        ~~~sh
        $ rosrun rubbing_hand filter_node.py
        ~~~

- ロボットハンド起動
    1.  コントローラBluetooth接続
        ~~~sh
        $ bluetoothctl
        [bluetooth]# power on
        [bluetooth]# connect B8:78:26:F7:86:9A
        [Joy-Con (L)]# exit
        ~~~

    2.  ダイナミクセルノード起動
        ~~~sh
        $ rosrun rubbing_hand dynamixel_thick_node.py 
        ~~~

-   トピック確認    
    ~~~sh
    $ rosrun plotjuggler plotjuggler
    ~~~

##  2021/04/26
物体の角度の2階微分値（角加速度）を扱いたい。   
時系列データ微分法を考える。    
word：数値微分、後退差分

### これまでの微分法    
単純に差分を取ったものを移動平均フィルタで平滑化し使用

## 2021/05/09
回転が起きてから指を閉じてもなかなか回転を抑制できない．
よって，回転が起きる前にその回転を抑制したい．  
指を開く動きを，振動しながら開くモーションにする．
線形+sin関数で軌道を生成し，開いて閉じてを繰り返しながら少しずつ指間距離を広げる．