#!/bin/bash

source devel/setup.bash

roslaunch drone_routine main.launch \
    simulation_mode:=true
