#!/bin/bash

source devel/setup.bash

roslaunch drone_routine main.launch \
    simulation_mode:=true \
    device:=drone \
    device_config:=1L1C \
    enable_data_saver:=true \
    rviz:=0
