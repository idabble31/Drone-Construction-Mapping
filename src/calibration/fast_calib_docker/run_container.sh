docker run -it --rm \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/scanar/calibration_bags/calib_target.bag:/root/data/calib_target.bag" \
    --volume="/home/scanar/calibration_bags/snap_target.jpg:/root/data/snap_target.jpg" \
    --volume="/home/scanar/scan_ar/src/calibration/fast_calib_docker/config/test_qr_params.yaml:/catkin_ws/src/FAST-Calib/config/test_qr_params.yaml" \
    --volume="/home/scanar/scan_ar/src/calibration/fast_calib_docker/launch/calib_robosense_single.launch:/catkin_ws/src/FAST-Calib/launch/calib_robosense_single.launch" \
    fast_calib:noetic
