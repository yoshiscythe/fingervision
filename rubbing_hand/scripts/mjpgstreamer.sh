#!/bin/bash

prepath="/home/suzuki/prg/mjpg-streamer2/mjpg-streamer-experimental"
$prepath/mjpg_streamer -i $prepath"/input_uvc.so -f -1 -r 320x240 -d /dev/video2 -n" -o $prepath"/output_http.so -w ./www -p 8080"