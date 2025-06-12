#!/bin/bash
set -e
source /opt/ros/humble/setup.bash

aprilgrid_path="/app/calibration_project/basalt/utils/aprilgrid_10x7_pro.json" 

# calib_dir="/big_disk/hawk_calib_recs"
cd /big_disk/hawk_calib_recs
#-----------------------------------------------------------------------------------------------------------------------------------------------------------------

# Note: Ask person Name, Date 

#-----------------------------------------------------------------------------------------------------------------------------------------------------------------

# Step 1 Converting mcap to euroc

python3 /app/calibration_project/basalt/utils/convert2euroc_hawk_mono.py -r ${1}
#-----------------------------------------------------------------------------------------------------------------------------------------------------------------

#Step 2 Camera Calibration

cam_dataset_path="${1}/euroc/1a_1"
cam_result_path="${1}/calibs/1a_1"

[ ! -d "$cam_result_path" ] && mkdir -p "$cam_result_path"

/app/calibration_project/basalt/build/calibrate_camera --dataset-path $cam_dataset_path --dataset-type euroc --aprilgrid $aprilgrid_path --result-path $cam_result_path --cam-types pinhole-radtan8 pinhole-radtan8

#-----------------------------------------------------------------------------------------------------------------------------------------------------------------

#Step 4 Camera-IMU Calibration

cam_imu_dataset_path="${1}/euroc/1b_1"
cam_imu_result_path="${1}/calibs/1b_1"

[ ! -d "$cam_imu_result_path" ] && mkdir -p "$cam_imu_result_path"

# Step 4a Copying 1a calibration.json file to 1b calibration path

calibration_file_1a="${cam_result_path}/calibration.json"
if [ -f "$calibration_file_1a" ]; then
    cp "$calibration_file_1a" "$cam_imu_result_path"
    echo "Copied calibration.json from $cam_result_path to $cam_imu_result_path"
else
    echo "Error: calibration.json not found in $cam_result_path"
    exit 1
fi

/app/calibration_project/basalt/build/calibrate_imu --dataset-path $cam_imu_dataset_path --dataset-type euroc --aprilgrid $aprilgrid_path --result-path $cam_imu_result_path
#-----------------------------------------------------------------------------------------------------------------------------------------------------------------

# Step 5: Uploading calibration.json to AWS S3 bucket

mv "$cam_imu_result_path/calibration.json" "$cam_imu_result_path/${1}.json"
aws s3 cp "$cam_imu_result_path/${1}.json" s3://bonsai-remote-assets/cameras/hawk_calibs/