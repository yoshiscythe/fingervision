#!/bin/bash

device_num=${1:-2}	# 第一パラメータ，デフォルト値2
video_device="/dev/video"${device_num}

echo "${video_device}" "$1" "${device_num}"

prepath="/home/suzuki/prg/mjpg-streamer2/mjpg-streamer-experimental"
$prepath/mjpg_streamer -i $prepath"/input_uvc.so -f -1 -r 320x240 -d "$video_device" -n" -o $prepath"/output_http.so -w ./www -p 8080"
