#!/bin/bash

# 1. Configuration - Change if your interface name or IP changes
INTERFACE="enP8p1s0"
STATIC_IP="192.168.1.10/24"
CONFIG_FILE="/home/scanar/scan_ar/gptp_master.conf"
WORKSPACE="/home/scanar/scan_ar"

echo "--- Starting RoboSense Airy PTP Master Setup ---"

# 2. Set Static IP
# Note: This requires sudo. It will prompt for password once.
echo "[1/4] Configuring Static IP $STATIC_IP on $INTERFACE..."
sudo ip addr flush dev $INTERFACE
sudo ip addr add $STATIC_IP dev $INTERFACE
sudo ip link set $INTERFACE up

# 3. Start PTP Master in the background
# We redirect output to a log file so it doesn't clutter your terminal
echo "[2/4] Launching PTP Master (ptp4l) in background..."
sudo ptp4l -f $CONFIG_FILE -i $INTERFACE -m -S > $WORKSPACE/ptp_log.txt 2>&1 &
PTP_PID=$!

# Give PTP a moment to assume Grand Master role
sleep 2
echo "      PTP Master running (PID: $PTP_PID). Logs at $WORKSPACE/ptp_log.txt"

# 4. Source ROS 2 Environment
echo "[3/4] Sourcing ROS 2 and Workspace..."
source /opt/ros/humble/setup.bash
source $WORKSPACE/install/setup.bash

# 5. Launch LiDAR Node
echo "[4/4] Starting rslidar_sdk node..."
ros2 launch rslidar_sdk start.py

# Cleanup: Kill PTP Master when you Ctrl+C the script
trap "echo 'Shutting down...'; sudo kill $PTP_PID; exit" SIGINT SIGTERM
