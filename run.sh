#!/bin/bash

source devel/setup.bash

roslaunch drone_routine new_main.launch \
    simulation_mode:=false
