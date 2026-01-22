#!/bin/bash

# --- ROS 2 Environment Setup ---
# Ensure the script knows where ROS 2 is installed.
source /opt/ros/humble/setup.bash

echo "----------------------------------------------------------"
echo "Starting ROS 2 Camera Calibration"
echo "Target: 7x5 Inner Corners | Square: 25mm (0.025m)"
echo "----------------------------------------------------------"

# --- Calibration Command ---
# We use 'ros2 run' to launch the python node from the camera_calibration package.
ros2 run camera_calibration cameracalibrator \
  --size 7x5 \
  --square 0.025 \
  --no-service-check \
  --ros-args \
  -r image:=/image_raw \
  -r camera_info:=/camera_info

# ==========================================================
# ARGUMENT EXPLANATIONS:
# ==========================================================
# --size 7x5
#   This defines the INNER CORNERS of your checkerboard, not the squares.
#   For a board with 8 columns and 6 rows of squares, use 7x5.
#
# --square 0.025
#   The physical size of one black square side in METERS. 
#   (Example: 25mm = 0.025m). This sets the scale for the SLAM system.
#
# --no-service-check
#   Tells the tool NOT to wait for the 'set_camera_info' service.
#   Many simple camera drivers don't support this service, and without 
#   this flag, the GUI might hang or crash on startup.
#
# -r image:=/image_raw
#   REMAP: Connects the tool's internal 'image' input to your 
#   actual camera topic (/image_raw).
#
# -r camera_info:=/camera_info
#   REMAP: Connects the tool's metadata output to your camera's 
#   info topic (/camera_info).
# ==========================================================

# --- Post-Calibration Cleanup ---
# The tool saves data to /tmp/calibrationdata.tar.gz by default.
# If you are in Docker, we should move it to your workspace before exit.
if [ -f /tmp/calibrationdata.tar.gz ]; then
    mkdir -p ./calibration_results
    mv /tmp/calibrationdata.tar.gz ./calibration_results/
    echo "Done! Calibration results moved to: ./calibration_results/"
fi