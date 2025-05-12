#!/bin/bash

source /opt/ros/humble/setup.bash

aprilgrid_path="/root/basalt/utils/aprilgrid_10x7_pro.json" 

# Note: Ask person Name, Date 

# Step 1 Converting mcap to euroc

python3 /root/basalt/utils/convert2euroc_hawk_mono.py -r ${1}


#Step 2 Camera Calibration

cam_dataset_path="${1}/euroc/1a_1"
cam_result_path="${1}/calibs/1a_1"

[ ! -d "$cam_result_path" ] && mkdir -p "$cam_result_path"

/root/basalt/build/calibrate_camera --dataset-path $cam_dataset_path --dataset-type euroc --aprilgrid $aprilgrid_path --result-path $cam_result_path --cam-types pinhole pinhole-radtan8