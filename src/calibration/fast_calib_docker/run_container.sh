#!/bin/bash
set -euo pipefail

# DISPLAY="${DISPLAY:-:0}"
# xhost +local:docker

docker run -itd \
    --name=extrinsic_calibration \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/scanar/calibration_bags/calib_target.bag:/root/data/calib_target.bag" \
    --volume="/home/scanar/calibration_bags/snap_target.jpg:/root/data/snap_target.jpg" \
    --volume="/home/scanar/scan_ar/src/calibration/fast_calib_docker/config/test_qr_params.yaml:/catkin_ws/src/FAST-Calib/config/test_qr_params.yaml" \
    --volume="/home/scanar/scan_ar/src/calibration/fast_calib_docker/launch/calib_robosense_single.launch:/catkin_ws/src/FAST-Calib/launch/calib_robosense_single.launch" \
    --volume="/home/scanar/scan_ar/src/calibration/fast_calib_docker/scripts/distance_filter_tool_rs.py:/catkin_ws/src/FAST-Calib/scripts/distance_filter_tool_rs.py" \
    --volume="/home/scanar/calibration_bags/output:/catkin_ws/src/FAST-Calib/output" \
    fast_calib:noetic 