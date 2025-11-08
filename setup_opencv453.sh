#!/bin/bash

export OpenCV_DIR="$HOME/opencv-4.5.3-install/lib/cmake/opencv4"
export PKG_CONFIG_PATH="$HOME/opencv-4.5.3-install/lib/pkgconfig:$PKG_CONFIG_PATH"
export LD_LIBRARY_PATH="$HOME/opencv-4.5.3-install/lib:$LD_LIBRARY_PATH"
export PATH="$HOME/opencv-4.5.3-install/bin:$PATH"

# export LD_PRELOAD="$HOME/opencv-4.5.3-install/lib/libopencv_core.so.4.5:$HOME/opencv-4.5.3-install/lib/libopencv_imgproc.so.4.5:$HOME/opencv-4.5.3-install/lib/libopencv_imgcodecs.so.4.5:$HOME/opencv-4.5.3-install/lib/libopencv_highgui.so.4.5:$HOME/opencv-4.5.3-install/lib/libopencv_calib3d.so.4.5"

echo "================================================"
echo "Using OpenCV 4.5.3 on Jetson (with LD_PRELOAD)"
echo "================================================"

# source /opt/ros/noetic/setup.bash  # Adjust if using different ROS version

# if [ -f "$HOME/dev/Drone-Construction-Mapping/devel/setup.bash" ]; then
#     source $HOME/dev/Drone-Construction-Mapping/devel/setup.bash
# fi
