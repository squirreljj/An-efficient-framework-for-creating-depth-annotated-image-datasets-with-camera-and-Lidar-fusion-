#!/usr/bin/env python3

import yaml
import numpy as np
import cv2
import glob
from os import path
import threading
#parameter modify
with open("../start.yaml", "r") as file:
    data = yaml.load(file, Loader=yaml.Loader)
input_folder=data["input_folder"]
output_folder=data["output_folder"]
def process_file(input_path, output_path, height, width, color_fmt_str):
    ext = input_path.split('.')[-1]
    iname = input_path.split('.'+ext)[0].split('/')[-1]
    oname = iname[0:3]+"."+iname[3:]+".png"
    if color_fmt_str == 'yuv420sp':
        img_blob = np.fromfile(input_path, dtype='uint8').reshape([int(1.5*height), width, 1])
        img_blob = cv2.cvtColor(img_blob, cv2.COLOR_YUV2BGR_NV21)
    elif color_fmt_str == 'yuyv':
        img_blob = np.fromfile(input_path, dtype='uint8').reshape([height, width, 2])
        img_blob = cv2.cvtColor(img_blob, cv2.COLOR_YUV2BGR_Y422)
    cv2.imwrite(output_path+"/"+oname, img_blob)

if __name__ == '__main__':
    import sys
    from pathlib import Path
    # get args options
    #pinhole left
    input_dir, output_dir = ".", "-png"
    resolution_str = "640,480"
    color_fmt_str = "yuv400"
    input_dir=input_folder+"/CameraGroup-FRONT-YUYV/cam0/"
    output_dir=output_folder+"/pinhole/left/"
    height, width = 976,1392
    color_fmt_str="yuyv"
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    threads = []
    for input_path in glob.glob(input_dir+"/*.*"):
        output_path = output_dir
        t = threading.Thread(target=process_file, args=(input_path, output_path, height, width, color_fmt_str))
        t.start()
        threads.append(t)
    for t in threads:
        t.join()
    #pinhole right
    input_dir2=input_folder+"/CameraGroup-FRONT-YUYV/cam1/"
    output_dir2=output_folder+"/pinhole/right/"
    height2, width2 = 976,1392
    color_fmt_str2="yuyv"
    Path(output_dir2).mkdir(parents=True, exist_ok=True)
    threads2 = []
    for input_path2 in glob.glob(input_dir2+"/*.*"):
        output_path2 = output_dir2
        t = threading.Thread(target=process_file, args=(input_path2, output_path2, height2, width2, color_fmt_str2))
        t.start()
        threads2.append(t)
    for t in threads2:
        t.join()
    #fisheye left
    input_dir3=input_folder+"/CameraGroup-UP-YUV420SP_NV21/cam0/"
    output_dir3=output_folder+"/fisheye/left/"
    height3, width3 = 1126,1120
    color_fmt_str3="yuv420sp"
    Path(output_dir3).mkdir(parents=True, exist_ok=True)
    threads3 = []
    for input_path3 in glob.glob(input_dir3+"/*.*"):
        output_path3 = output_dir3
        t = threading.Thread(target=process_file, args=(input_path3, output_path3, height3, width3, color_fmt_str3))
        t.start()
        threads3.append(t)
    for t in threads3:
        t.join()
    #fisheye right 
    input_dir4=input_folder+"/CameraGroup-UP-YUV420SP_NV21/cam1/"
    output_dir4=output_folder+"/fisheye/right/"
    height4, width4 = 1126,1120
    color_fmt_str4="yuv420sp"
    Path(output_dir4).mkdir(parents=True, exist_ok=True)
    threads4 = []
    for input_path4 in glob.glob(input_dir4+"/*.*"):
        output_path4 = output_dir4
        t = threading.Thread(target=process_file, args=(input_path4, output_path4, height4, width4, color_fmt_str4))
        t.start()
        threads4.append(t)
    for t in threads4:
        t.join()



