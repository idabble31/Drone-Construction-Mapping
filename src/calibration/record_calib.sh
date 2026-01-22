#!/bin/bash

# Define the storage directory
BAG_DIR="$HOME/calibration_bags"
mkdir -p "$BAG_DIR"

# Generate a filename based on the current date and time
TIMESTAMP=$(date +"%Y_%m_%d_%H_%M_%S")
BAG_NAME="calib_session_$TIMESTAMP"

echo "------------------------------------------------"
echo "Starting Calibration Recording: $BAG_NAME"
echo "Saving to: $BAG_DIR"
echo "Recording: /image_raw, /camera_info, /rslidar_points, /tf_static"
echo "Press Ctrl+C to stop recording when finished."
echo "------------------------------------------------"

# Record the topics in MCAP format
# Note: Ensure ros-humble-rosbag2-storage-mcap (or your distro equivalent) is installed
ros2 bag record -s mcap \
    -o "$BAG_DIR/$BAG_NAME" \
    /image_raw \
    /camera_info \
    /rslidar_points \
    /tf_static