#!/bin/bash

# --- ROS 2 Environment Setup ---
source /opt/ros/humble/setup.bash
# Source your workspace so it finds the camera package (if needed)
# source ~/scan_ar/install/setup.bash 

echo "----------------------------------------------------------"
echo "Starting Camera Node (Background)"
echo "----------------------------------------------------------"

# 1. Start ONLY the camera node in the background (&)
ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p video_device:="/dev/video0" \
    -p image_size:="[640,480]" \
    -p camera_frame_id:="camera_link" &

# Capture the Process ID (PID) of the camera so we can kill it later
CAM_PID=$!

# Give the camera 2 seconds to warm up
sleep 2

echo "----------------------------------------------------------"
echo "Starting Calibration Tool"
echo "----------------------------------------------------------"

# 2. Run the Calibration Tool
ros2 run camera_calibration cameracalibrator \
  --size 7x7 \
  --square 0.04 \
  --no-service-check \
  --ros-args \
  -r image:=/image_raw \
  -r camera_info:=/camera_info

# 3. Cleanup: Kill the camera node when calibration closes
echo "Calibration finished. Shutting down camera..."
kill $CAM_PID

# 4. Move Results
if [ -f /tmp/calibrationdata.tar.gz ]; then
    mkdir -p ./calibration_results
    mv /tmp/calibrationdata.tar.gz ./calibration_results/
    echo "Results saved to ./calibration_results/"
fi