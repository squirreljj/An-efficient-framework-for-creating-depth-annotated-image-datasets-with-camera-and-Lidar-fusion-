#!/bin/bash

cd tools
time python3 imgbin_to_png.py
time python3 pointcloud_reader.py
time roslaunch floam floam_rsbpearl32.launch
cd ../preprocess/build
time ./pose_transform "/home/ljj/work/lidar-cam-calibration/start.yaml"
time ./preprocess "/home/ljj/work/lidar-cam-calibration/start.yaml"

