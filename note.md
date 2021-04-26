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